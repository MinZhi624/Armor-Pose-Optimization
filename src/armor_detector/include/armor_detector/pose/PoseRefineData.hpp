#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace armor_detector::pose {
    enum class SinglePoseRefineMethod {
        NONE,
        YAW_SEARCH,
        YAW_SEARCH_THEN_DISTANCE,
        POSE_ONLY_BA_6DOF,
        POSE_ONLY_BA_4DOF_XYZ,
        POSE_ONLY_BA_4DOF_YPD,
    };

    enum class DualPoseRefineMethod { NONE, DUAL_ARMOR_BA_3DOF_YPD, DUAL_ARMOR_BA_7DOF_XYZ };

    struct PoseRefineInput {
        std::array<cv::Point2f, 4> image_corners;
        cv::Vec3d initial_rvec = {0.0, 0.0, 0.0};
        cv::Vec3d initial_tvec = {0.0, 0.0, 0.0};
        ArmorType armor_type = ArmorType::NONE;
        ArmorName armor_name = ArmorName::NONE;
        cv::Matx33d camera_matrix = cv::Matx33d::eye();
        cv::Vec<double, 5> distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};
    };

    struct PoseRefineSolverSummary {
        bool available = false;
        double initial_cost = 0.0;
        double final_cost = 0.0;
        int num_iterations = 0;
        std::string termination_type = "not_run";
    };

    struct PoseRefineOutput {
        cv::Vec3d rvec = {0.0, 0.0, 0.0};
        cv::Vec3d tvec = {0.0, 0.0, 0.0};
        bool success = false;
        double reprojection_error_px = 0.0;
        PoseRefineSolverSummary solver_summary;
    };

    struct DualArmorPoseRefineSummary {
        std::size_t armor_a_index = 0;
        std::size_t armor_b_index = 0;
        double shared_pose_yaw_rad = 0.0;
        double distance_a_m = 0.0;
        double distance_b_m = 0.0;
        double armor_a_mean_reprojection_error_px = 0.0;
        double armor_b_mean_reprojection_error_px = 0.0;
        double mean_reprojection_error_px = 0.0;
        PoseRefineSolverSummary solver_summary;
    };

    struct PoseRefineBatchOutput {
        std::vector<PoseRefineOutput> items;
        std::optional<DualArmorPoseRefineSummary> dual_summary;
    };

    std::string toString(SinglePoseRefineMethod method);
    SinglePoseRefineMethod singlePoseRefineMethodFromString(std::string_view name);
    std::string toString(DualPoseRefineMethod method);
    DualPoseRefineMethod dualPoseRefineMethodFromString(std::string_view name);

} // namespace armor_detector::pose
