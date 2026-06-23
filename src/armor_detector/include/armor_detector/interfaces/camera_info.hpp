#pragma once

#include <opencv2/core.hpp>

namespace armor_detector
{

struct CameraInfo
{
    cv::Matx33d camera_matrix;           // 内参 3x3
    cv::Vec<double, 5> distortion_coefficients; // 畸变 1x5
    cv::Matx33d rectification_matrix;    // 校正 3x3
    cv::Matx34d projection_matrix;       // 投影 3x4
};

} // namespace armor_detector
