#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <string_view>

namespace armor_detector::pose {
    enum class PoseRefineMethod { NONE, YAW_SEARCH, POSE_ONLY_BA_6DOF, POSE_ONLY_BA_4DOF_XYZ, POSE_ONLY_BA_4DOF_YPD };

    struct PoseRefineInput {
        std::array<cv::Point2f, 4> image_corners;
        cv::Vec3d initial_rvec = {0.0, 0.0, 0.0};
        cv::Vec3d initial_tvec = {0.0, 0.0, 0.0};
        ArmorType armor_type = ArmorType::NONE;
        cv::Matx33d camera_matrix = cv::Matx33d::eye();
        cv::Vec<double, 5> distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};
    };

    struct PoseRefineOutput {
        cv::Vec3d rvec = {0.0, 0.0, 0.0};
        cv::Vec3d tvec = {0.0, 0.0, 0.0};
        bool success = false;
        double reprojection_error_px = 0.0;
    };

    std::string toString(PoseRefineMethod method);
    PoseRefineMethod poseRefineMethodFromString(std::string_view name);

} // namespace armor_detector::pose
