#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <string>
#include <string_view>
#include <array>

namespace armor_detector::pose {
    enum class PoseRefineMethod {
        NONE,
        YAW_SEARCH
    };

    struct PoseRefineInput {
        std::array<cv::Point2f, 4> image_corners;
        Eigen::Vector3d initial_xyz_gimbal = Eigen::Vector3d::Zero();
        double initial_yaw_rad = 0.0;
        ArmorType armor_type = ArmorType::NONE;
    };

    struct PoseRefineOutput {
        Eigen::Vector3d xyz_gimbal = Eigen::Vector3d::Zero();
        double yaw_rad = 0.0;
        bool success = false;
        double reprojection_error_px = 0.0;
    };

    std::string toString(PoseRefineMethod method);
    PoseRefineMethod poseRefineMethodFromString(std::string_view name); 


} // namespace armor_detector::pose