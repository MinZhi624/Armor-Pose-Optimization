#pragma once

#include <Eigen/Core>
#include <opencv2/core/types.hpp>
#include <vector>

namespace armor_detector::tools {

    // 装甲板完整尺寸 (m)
    inline constexpr double SMALL_ARMOR_WIDTH = 0.135; // 135mm
    inline constexpr double SMALL_ARMOR_HEIGHT = 0.055; // 55mm
    inline constexpr double LARGE_ARMOR_WIDTH = 0.225; // 225mm
    inline constexpr double LARGE_ARMOR_HEIGHT = 0.055; // 55mm

    // 半尺寸 — PnP 用
    inline constexpr double SMALL_HALF_WIDTH = SMALL_ARMOR_WIDTH / 2.0;
    inline constexpr double SMALL_HALF_HEIGHT = SMALL_ARMOR_HEIGHT / 2.0;
    inline constexpr double LARGE_HALF_WIDTH = LARGE_ARMOR_WIDTH / 2.0;
    inline constexpr double LARGE_HALF_HEIGHT = LARGE_ARMOR_HEIGHT / 2.0;

    // pitch 倾角
    inline constexpr double ARMOR_PITCH_DEG = 15.0;
    inline constexpr double ARMOR_PITCH_RAD = ARMOR_PITCH_DEG * M_PI / 180.0;

    // 3D 角点 — 坐标系: X法向量, Y左, Z上
    // 顺序: left_top, right_top, right_bottom, left_bottom (顺时针)
    inline const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
        {0.0f, static_cast<float>(SMALL_HALF_WIDTH), static_cast<float>(SMALL_HALF_HEIGHT)},
        {0.0f, static_cast<float>(-SMALL_HALF_WIDTH), static_cast<float>(SMALL_HALF_HEIGHT)},
        {0.0f, static_cast<float>(-SMALL_HALF_WIDTH), static_cast<float>(-SMALL_HALF_HEIGHT)},
        {0.0f, static_cast<float>(SMALL_HALF_WIDTH), static_cast<float>(-SMALL_HALF_HEIGHT)}};

    inline const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
        {0.0f, static_cast<float>(LARGE_HALF_WIDTH), static_cast<float>(LARGE_HALF_HEIGHT)},
        {0.0f, static_cast<float>(-LARGE_HALF_WIDTH), static_cast<float>(LARGE_HALF_HEIGHT)},
        {0.0f, static_cast<float>(-LARGE_HALF_WIDTH), static_cast<float>(-LARGE_HALF_HEIGHT)},
        {0.0f, static_cast<float>(LARGE_HALF_WIDTH), static_cast<float>(-LARGE_HALF_HEIGHT)}};

} // namespace armor_detector::tools
