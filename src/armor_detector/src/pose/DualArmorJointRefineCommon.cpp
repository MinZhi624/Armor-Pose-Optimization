#include "DualArmorJointRefineCommon.hpp"

#include "armor_detector/tools/angle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace armor_detector::pose::dual_armor_detail {
    namespace {
        constexpr double kNearZeroDirection = 1e-12;

        bool isUsablePose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
                    return false;
                }
            }
            return tvec[2] > 0.0;
        }

        double imageCenterX(const PoseRefineInput &input) {
            double center_x = 0.0;
            for (const auto &corner : input.image_corners) {
                center_x += static_cast<double>(corner.x);
            }
            return center_x / static_cast<double>(input.image_corners.size());
        }
    } // namespace

    bool findUniquePair(const std::vector<PoseRefineInput> &inputs, PairIndices &pair) {
        std::vector<std::array<std::size_t, 2>> pairs;
        for (std::size_t first = 0; first < inputs.size(); ++first) {
            if (inputs[first].armor_name == ArmorName::NONE) {
                continue;
            }
            bool seen_name = false;
            for (std::size_t previous = 0; previous < first; ++previous) {
                if (inputs[previous].armor_name == inputs[first].armor_name) {
                    seen_name = true;
                    break;
                }
            }
            if (seen_name) {
                continue;
            }

            std::vector<std::size_t> group;
            for (std::size_t index = 0; index < inputs.size(); ++index) {
                if (inputs[index].armor_name == inputs[first].armor_name) {
                    group.push_back(index);
                }
            }
            if (group.size() == 2 && inputs[group[0]].armor_type == inputs[group[1]].armor_type &&
                inputs[group[0]].armor_type != ArmorType::NONE &&
                isUsablePose(inputs[group[0]].initial_rvec, inputs[group[0]].initial_tvec) &&
                isUsablePose(inputs[group[1]].initial_rvec, inputs[group[1]].initial_tvec)) {
                pairs.push_back({group[0], group[1]});
            }
        }
        if (pairs.size() != 1) {
            return false;
        }

        const auto &indices = pairs.front();
        if (imageCenterX(inputs[indices[1]]) < imageCenterX(inputs[indices[0]])) {
            pair = {indices[1], indices[0]};
        }
        else {
            pair = {indices[0], indices[1]};
        }
        return true;
    }

    bool initializeSharedYaw(const ArmorPose &pose_a,
                             const ArmorPose &pose_b,
                             double yaw_offset_rad,
                             double &shared_yaw_rad) {
        const double sin_sum = std::sin(pose_a.ypr_gimbal.x()) + std::sin(pose_b.ypr_gimbal.x() - yaw_offset_rad);
        const double cos_sum = std::cos(pose_a.ypr_gimbal.x()) + std::cos(pose_b.ypr_gimbal.x() - yaw_offset_rad);
        if (!std::isfinite(std::hypot(sin_sum, cos_sum)) || std::hypot(sin_sum, cos_sum) <= kNearZeroDirection) {
            return false;
        }
        shared_yaw_rad = tools::normalizeRadAngle(std::atan2(sin_sum, cos_sum));
        return std::isfinite(shared_yaw_rad);
    }

    bool selectPlusCandidate(bool plus_usable, double plus_cost, bool minus_usable, double minus_cost) {
        constexpr double kCostTieRelativeTolerance = 1e-12;
        if (!plus_usable) {
            return false;
        }
        if (!minus_usable) {
            return true;
        }
        const double tolerance = kCostTieRelativeTolerance * std::max({1.0, std::abs(plus_cost), std::abs(minus_cost)});
        return minus_cost + tolerance >= plus_cost;
    }

    PoseRefineOutput pnpOutput(const PoseRefineInput &input) {
        PoseRefineOutput output;
        output.rvec = input.initial_rvec;
        output.tvec = input.initial_tvec;
        output.success = isUsablePose(output.rvec, output.tvec);
        try {
            output.reprojection_error_px = calculateReprojectionError(input.armor_type,
                                                                      input.image_corners,
                                                                      output.rvec,
                                                                      output.tvec,
                                                                      input.camera_matrix,
                                                                      input.distortion_coefficients);
        }
        catch (const cv::Exception &) {
            output.reprojection_error_px = std::numeric_limits<double>::quiet_NaN();
        }
        return output;
    }

    void setPairPnpFallback(const std::vector<PoseRefineInput> &inputs,
                            const PairIndices &pair,
                            PoseRefineBatchOutput &output,
                            const PoseRefineSolverSummary *solver_summary) {
        output.items[pair.armor_a_index] = pnpOutput(inputs[pair.armor_a_index]);
        output.items[pair.armor_b_index] = pnpOutput(inputs[pair.armor_b_index]);
        if (solver_summary != nullptr) {
            output.items[pair.armor_a_index].solver_summary = *solver_summary;
            output.items[pair.armor_b_index].solver_summary = *solver_summary;
        }
        output.dual_summary.reset();
    }

    void copyFallbackItems(const std::vector<PoseRefineInput> &inputs,
                           const PairIndices &pair,
                           const IPoseRefiner *fallback_refiner,
                           PoseRefineBatchOutput &output) {
        std::vector<std::size_t> indices;
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            if (i != pair.armor_a_index && i != pair.armor_b_index) {
                indices.push_back(i);
            }
        }
        if (indices.empty()) {
            return;
        }
        if (fallback_refiner == nullptr) {
            for (const std::size_t index : indices) {
                output.items[index] = pnpOutput(inputs[index]);
            }
            return;
        }
        std::vector<PoseRefineInput> fallback_inputs;
        fallback_inputs.reserve(indices.size());
        for (const std::size_t index : indices) {
            fallback_inputs.push_back(inputs[index]);
        }
        const PoseRefineBatchOutput fallback_output = fallback_refiner->refine(fallback_inputs);
        for (std::size_t i = 0; i < indices.size(); ++i) {
            output.items[indices[i]] =
                i < fallback_output.items.size() ? fallback_output.items[i] : pnpOutput(inputs[indices[i]]);
        }
    }
} // namespace armor_detector::pose::dual_armor_detail
