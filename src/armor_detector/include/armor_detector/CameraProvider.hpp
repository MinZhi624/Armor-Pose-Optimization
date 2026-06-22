#pragma once

#include "CameraInfo.hpp"
#include "Frame.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace armor_detector
{

class CameraProvider
{
private:
    CameraInfo camera_info_;

public:
    void init();  // 从 yaml 加载相机参数
    Frame receiveImage(const sensor_msgs::msg::Image::SharedPtr & msg);
    CameraInfo getCameraInfo();
};

} // namespace armor_detector
