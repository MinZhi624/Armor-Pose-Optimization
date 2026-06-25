#pragma once

#include <functional>

namespace armor_detector::yaw {

    using YawErrorFunction = std::function<double(double)>;

    /// 对 center_yaw 做枚举粗搜 + 三分精搜，返回使 error 最小的 yaw。
    double runYawSearch(double center_yaw, const YawErrorFunction &calculate_error);

} // namespace armor_detector::yaw
