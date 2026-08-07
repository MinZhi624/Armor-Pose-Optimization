#include "armor_detector/debug/DebugArmorYawPublisher.hpp"
#include "armor_detector/tools/angle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace armor_detector::debug {

    namespace {
        constexpr double kYawContinuityThresholdRad = tools::degToRad(45.0);
        constexpr double kCenterContinuityThresholdPx = 30.0;
        constexpr std::size_t kSlotTtlFrames = 3;

        cv::Point2f recordCenter(const PoseRefineDebugRecord &record) {
            return {static_cast<float>(record.center_x_px), static_cast<float>(record.center_y_px)};
        }

        double nanValue() {
            return std::numeric_limits<double>::quiet_NaN();
        }

        struct MatchCandidate {
            std::size_t slot_index = 0;
            std::size_t record_index = 0;
            double yaw_delta_rad = 0.0;
            double center_delta_px = 0.0;
        };

        bool betterMatch(const MatchCandidate &lhs, const MatchCandidate &rhs) {
            if (lhs.yaw_delta_rad != rhs.yaw_delta_rad) {
                return lhs.yaw_delta_rad < rhs.yaw_delta_rad;
            }
            return lhs.center_delta_px < rhs.center_delta_px;
        }

        bool betterRecordQuality(const PoseRefineDebugRecord *lhs, const PoseRefineDebugRecord *rhs) {
            if (lhs->confidence != rhs->confidence) {
                return lhs->confidence > rhs->confidence;
            }
            return lhs->final_reprojection_error_px < rhs->final_reprojection_error_px;
        }

        std::size_t chooseFillSlot(const std::array<bool, 2> &slot_assigned, const std::array<ArmorYawSlot, 2> &slots) {
            std::size_t best_slot = slots.size();
            for (std::size_t i = 0; i < slots.size(); ++i) {
                if (slot_assigned[i]) {
                    continue;
                }
                if (!slots[i].has_track) {
                    return i;
                }
                if (best_slot == slots.size() || slots[i].last_frame_index < slots[best_slot].last_frame_index) {
                    best_slot = i;
                }
            }
            return best_slot;
        }

        void fillArmor1(armor_interfaces::msg::ArmorYawDebug &msg, const PoseRefineDebugRecord *record) {
            if (record == nullptr) {
                msg.armor_1_origin_yaw_rad = nanValue();
                msg.armor_1_final_yaw_rad = nanValue();
                msg.armor_1_error_delta_px = nanValue();
                return;
            }

            msg.armor_1_origin_yaw_rad = record->initial_yaw_rad;
            msg.armor_1_final_yaw_rad = record->final_yaw_rad;
            msg.armor_1_error_delta_px = record->delta_reprojection_error_px;
        }

        void fillArmor2(armor_interfaces::msg::ArmorYawDebug &msg, const PoseRefineDebugRecord *record) {
            if (record == nullptr) {
                msg.armor_2_origin_yaw_rad = nanValue();
                msg.armor_2_final_yaw_rad = nanValue();
                msg.armor_2_error_delta_px = nanValue();
                return;
            }

            msg.armor_2_origin_yaw_rad = record->initial_yaw_rad;
            msg.armor_2_final_yaw_rad = record->final_yaw_rad;
            msg.armor_2_error_delta_px = record->delta_reprojection_error_px;
        }
    } // namespace

    DebugArmorYawPublisher::DebugArmorYawPublisher(rclcpp::Node &node, DebugLayerState &layer_state) :
        layer_state_(layer_state) {
        publisher_ = node.create_publisher<armor_interfaces::msg::ArmorYawDebug>("/debug/armor_yaw", 10);
    }

    void DebugArmorYawPublisher::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!layer_state_.enabled(DebugLayer::POSE)) {
            return;
        }

        const CurrentRecords current_records = selectCurrentRecords(context.frame_index, data);

        armor_interfaces::msg::ArmorYawDebug msg;
        msg.header.stamp = context.stamp;
        msg.header.frame_id = "gimbal";
        fillArmor1(msg, current_records[0]);
        fillArmor2(msg, current_records[1]);
        publisher_->publish(msg);
    }

    void DebugArmorYawPublisher::expireStaleSlots(std::size_t frame_index) {
        for (auto &slot : slots_) {
            if (slot.has_track && frame_index > slot.last_frame_index &&
                frame_index - slot.last_frame_index >= kSlotTtlFrames) {
                slot.has_track = false;
            }
        }
    }

    DebugArmorYawPublisher::CurrentRecords DebugArmorYawPublisher::selectCurrentRecords(std::size_t frame_index,
                                                                                        const PoseDebugData &data) {
        expireStaleSlots(frame_index);

        CurrentRecords current_records = {nullptr, nullptr};
        std::vector<const PoseRefineDebugRecord *> successful_records;
        successful_records.reserve(data.refine_records.size());
        for (const auto &record : data.refine_records) {
            if (record.success) {
                successful_records.push_back(&record);
            }
        }
        if (successful_records.empty()) {
            return current_records;
        }

        std::vector<MatchCandidate> candidates;
        for (std::size_t slot_index = 0; slot_index < slots_.size(); ++slot_index) {
            const auto &slot = slots_[slot_index];
            if (!slot.has_track) {
                continue;
            }
            for (std::size_t record_index = 0; record_index < successful_records.size(); ++record_index) {
                const auto &record = *successful_records[record_index];
                const double yaw_delta_rad =
                    std::abs(tools::normalizeRadAngle(record.final_yaw_rad - slot.final_yaw_rad));
                const double center_delta_px = cv::norm(recordCenter(record) - slot.center_px);
                if (yaw_delta_rad <= kYawContinuityThresholdRad && center_delta_px <= kCenterContinuityThresholdPx) {
                    candidates.push_back({slot_index, record_index, yaw_delta_rad, center_delta_px});
                }
            }
        }

        std::stable_sort(candidates.begin(), candidates.end(), betterMatch);

        std::array<bool, 2> slot_assigned = {false, false};
        std::vector<bool> record_assigned(successful_records.size(), false);
        for (const auto &candidate : candidates) {
            if (slot_assigned[candidate.slot_index] || record_assigned[candidate.record_index]) {
                continue;
            }
            current_records[candidate.slot_index] = successful_records[candidate.record_index];
            slot_assigned[candidate.slot_index] = true;
            record_assigned[candidate.record_index] = true;
            updateSlot(candidate.slot_index, *successful_records[candidate.record_index], frame_index);
        }

        std::vector<const PoseRefineDebugRecord *> remaining_records;
        for (std::size_t i = 0; i < successful_records.size(); ++i) {
            if (!record_assigned[i]) {
                remaining_records.push_back(successful_records[i]);
            }
        }
        std::stable_sort(remaining_records.begin(), remaining_records.end(), betterRecordQuality);

        for (const auto *record : remaining_records) {
            const std::size_t slot_index = chooseFillSlot(slot_assigned, slots_);
            if (slot_index >= slots_.size()) {
                break;
            }
            current_records[slot_index] = record;
            slot_assigned[slot_index] = true;
            updateSlot(slot_index, *record, frame_index);
        }

        return current_records;
    }

    void DebugArmorYawPublisher::updateSlot(std::size_t slot_index,
                                            const PoseRefineDebugRecord &record,
                                            std::size_t frame_index) {
        auto &slot = slots_[slot_index];
        slot.has_track = true;
        slot.final_yaw_rad = record.final_yaw_rad;
        slot.center_px = recordCenter(record);
        slot.last_frame_index = frame_index;
    }

} // namespace armor_detector::debug
