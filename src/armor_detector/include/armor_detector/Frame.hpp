#pragma once

#include <opencv2/core.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace armor_detector
{

struct Frame
{
    cv::Mat image;
    builtin_interfaces::msg::Time timestamp;
};

} // namespace armor_detector