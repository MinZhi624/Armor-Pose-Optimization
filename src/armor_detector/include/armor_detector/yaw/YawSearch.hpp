#pragma once

#include <functional>

namespace armor_detector::yaw {

    using YawErrorFunction = std::function<double(double)>;

    /// 枚举搜索：在 [center - range, center + range] 范围内均匀采样
    /// @param center_yaw  中心角度（弧度）
    /// @param calculate_error  误差计算函数
    /// @param search_range_rad  搜索范围（弧度）
    /// @param step_rad  步长（弧度）
    /// @return 使误差最小的角度
    double enumerate(double center_yaw, const YawErrorFunction &calculate_error,
                     double search_range_rad, double step_rad);

    /// 三分搜索：在 [initial - range, initial + range] 范围内三分搜索
    /// @param initial_yaw  初始角度（弧度）
    /// @param calculate_error  误差计算函数
    /// @param local_range_rad  局部搜索范围（弧度）
    /// @param iterations  迭代次数
    /// @return 使误差最小的角度
    double ternary(double initial_yaw, const YawErrorFunction &calculate_error,
                   double local_range_rad, int iterations);

    /// 对 center_yaw 做枚举粗搜 + 三分精搜，返回使 error 最小的 yaw。
    double runYawSearch(double center_yaw, const YawErrorFunction &calculate_error);

} // namespace armor_detector::yaw
