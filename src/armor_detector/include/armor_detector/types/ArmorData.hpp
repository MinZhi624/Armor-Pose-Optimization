#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Core>

#include <array>

namespace armor_detector
{

enum class ArmorType {
    SMALL,
    LARGE,
    NONE
};

enum class ArmorName : int {
    ONE = 1,
    TWO = 2,
    THREE = 3,
    FOUR = 4,
    FIVE = 5,
    NONE = 0
};

enum class LightBarColor {
    RED,
    BLUE,
    NONE
};

////////// 基本数据结构 //////////
struct LightBar
{
    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;
    double angle = 0.0;        // 弧度（原 angle_deg，改为弧度匹配参考项目）
    double length = 0.0;
    double width = 0.0;
    int area = 0;
    int id = -1;
    LightBarColor color = LightBarColor::NONE;
};

struct ArmorGeometry
{
    std::array<LightBar, 2> paired_lights; // 按图像x轴排序: 0=left, 1=right
    std::array<cv::Point2f, 4> corners; // 顺序: left_top, right_top, right_bottom, left_bottom
    ArmorType type = ArmorType::NONE;
    // 判断信息
    double angle_diff_deg = 0.0;
    double length_diff = 0.0;
    double y_diff_ratio = 0.0;
    double x_diff_ratio = 0.0;
    double distance_ratio = 0.0;
};

struct ArmorClassification
{
    ArmorName name = ArmorName::NONE;
    float confidence = 0.0f;
    cv::Mat number_roi;
    cv::Mat pattern;
};

struct ArmorPose
{
    Eigen::Vector3d xyz_camera;
    Eigen::Vector3d xyz_gimbal;
    Eigen::Vector3d ypr_camera;
    Eigen::Vector3d ypr_gimbal;
    Eigen::Vector3d ypd_camera;
    Eigen::Vector3d ypd_gimbal;

    float image_distance_to_center = 0.0f;
};
/////////// 聚合数据结构 //////////
struct ArmorCandidate
{
    ArmorGeometry geometry;
};

struct ClassifiedArmor
{
    ArmorGeometry geometry;
    ArmorClassification classification;
};

struct SolvedArmor
{
    ArmorGeometry geometry;
    ArmorClassification classification;
    ArmorPose pose;
};

/////////// DEBUG //////////
enum class RejectReason;
struct RejectedArmor
{

};

} // namespace armor_detector