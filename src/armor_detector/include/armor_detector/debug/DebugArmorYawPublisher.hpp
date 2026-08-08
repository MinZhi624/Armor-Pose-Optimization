#pragma once

#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

#include <armor_interfaces/msg/armor_yaw_debug.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstddef>

namespace armor_detector::debug {

    struct ArmorYawSlot {
        bool has_track = false;
        double final_yaw_rad = 0.0;
        cv::Point2f center_px;
        std::size_t last_frame_index = 0;
    };

    class DebugArmorYawPublisher : public IDebugObserver {
    public:
        DebugArmorYawPublisher(rclcpp::Node &node, DebugLayerState &layer_state);

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;

    private:
        using CurrentRecords = std::array<const PoseRefineDebugRecord *, 2>;

        void expireStaleSlots(std::size_t frame_index);
        CurrentRecords selectCurrentRecords(std::size_t frame_index, const PoseDebugData &data);
        void updateSlot(std::size_t slot_index, const PoseRefineDebugRecord &record, std::size_t frame_index);

        rclcpp::Publisher<armor_interfaces::msg::ArmorYawDebug>::SharedPtr publisher_;
        DebugLayerState &layer_state_;
        std::array<ArmorYawSlot, 2> slots_;
    };

} // namespace armor_detector::debug
