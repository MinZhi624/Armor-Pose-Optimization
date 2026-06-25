#pragma once

#include "types/CameraInfo.hpp"
#include "types/Frame.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge header not found. Install ros-<distro>-cv-bridge."
#endif

namespace armor_detector {

    class CameraProvider {
    private:
        CameraInfo camera_info_;

    public:
        void init(); // 从 yaml 加载相机参数
        Frame receiveImage(const sensor_msgs::msg::Image::SharedPtr &msg);
        CameraInfo getCameraInfo();
    };

} // namespace armor_detector
