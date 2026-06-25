#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>

namespace armor_detector {

    struct Frame {
        cv::Mat image;
        builtin_interfaces::msg::Time timestamp;
    };

} // namespace armor_detector
