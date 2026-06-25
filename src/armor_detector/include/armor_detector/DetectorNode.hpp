#pragma once

#include "armor_detector/CameraProvider.hpp"
#include "armor_detector/detector/Detector.hpp"
#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugHub.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>
#include <rosbag2_interfaces/srv/set_rate.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>

#include <cstddef>
#include <string>

namespace armor_detector
{

/**
 * @brief Debug 配置参数，从 ROS 参数 debug.* 读取。
 */
struct DebugConfig
{
    bool show = true;
    bool rosbag_control = true;
    std::string rosbag_player_node = "/rosbag2_player";
    bool preprocess = false;
    bool lights = true;
    bool armor_match = false;
    bool classification = false;
    bool pose = false;
    std::size_t stats_interval = 50;
};

/**
 * @brief 装甲板检测 ROS 2 节点。
 *
 * 订阅 /image_raw，执行预处理 → 灯条检测流水线，
 * 通过 debug observer 体系输出可视化与统计。
 */
class DetectorNode : public rclcpp::Node
{
public:
    DetectorNode();
    ~DetectorNode() override;

private:
    void initParameters();
    void initDetectors();
    void initDebug();
    void initRosbagClients();

    void run(const sensor_msgs::msg::Image::SharedPtr & msg);
    void pollDebugKeys();

    // 检测组件
    CameraProvider camera_provider_;
    Detector detector_;
    LightDetector light_detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Debug
    DebugConfig debug_config_;
    debug::DebugGUI debug_gui_;
    debug::DebugHub debug_hub_;
    debug::DebugLayerState layer_state_;
    rclcpp::TimerBase::SharedPtr debug_key_timer_;

    // Rosbag 控制
    rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_paused_client_;
    rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr play_next_client_;
    rclcpp::Client<rosbag2_interfaces::srv::SetRate>::SharedPtr set_rate_client_;

    double playback_rate_ = 1.0;
    std::size_t frame_index_ = 0;
};

} // namespace armor_detector
