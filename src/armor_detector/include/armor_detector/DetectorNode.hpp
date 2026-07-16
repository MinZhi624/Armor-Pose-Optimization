#pragma once

#include "armor_detector/CameraProvider.hpp"
#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugHub.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/DebugPoseMarkerPublisher.hpp"
#include "armor_detector/detector/Detector.hpp"
#include "armor_detector/pose/PoseRefineRunner.hpp"
#include "armor_detector/pose/PoseSolver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>
#include <rosbag2_interfaces/srv/set_rate.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>

#include <cstddef>
#include <memory>
#include <string>

namespace armor_detector {

    /**
     * @brief Debug 配置参数，从 ROS 参数 debug.* 读取。
     */
    struct DebugConfig {
        bool show = true;
        bool rosbag_control = true;
        std::string rosbag_player_node = "/rosbag2_player";
        bool detect_stage_1 = false;
        bool detect_stage_2 = false;
        bool detect_stage_3 = false;
        bool detect_stage_4 = false;
        bool corner_correction = false;
        bool pose = false;
        bool pose_refine = false;
        bool pose_perturb_enabled = true;
        bool pose_perturb_show = false;
        double pose_perturb_dir_yaw_delta_deg = 1.0;
        double pose_perturb_dir_pitch_delta_deg = 1.0;
        double pose_perturb_distance_delta_m = 0.1;
        double pose_perturb_pose_yaw_delta_deg = 5.0;
        bool result = true;
        std::size_t stats_interval = 50;

        // Pose refine CSV
        bool pose_refine_csv_enabled = false;
        std::string pose_refine_csv_root_dir;
        std::string pose_refine_csv_video = "manual";
        std::string pose_refine_csv_corner_method;

        // Pose landscape experiment
        bool pose_landscape_enabled = false;
        std::string pose_landscape_root_dir;
        std::string pose_landscape_video = "manual";
        double pose_landscape_physical_min_distance_m = 1.0;
        double pose_landscape_physical_max_distance_m = 10.0;
        double pose_landscape_half_window_m = 3.0;
        double pose_landscape_distance_step_m = 0.05;
        double pose_landscape_pose_yaw_min_deg = -70.0;
        double pose_landscape_pose_yaw_max_deg = 70.0;
        double pose_landscape_pose_yaw_step_deg = 2.0;

        // Pose refine topic
        bool pose_refine_topic_enabled = true;
    };

    /**
     * @brief 装甲板检测 ROS 2 节点。
     *
     * 订阅 /image_raw，执行预处理 → 灯条检测流水线，
     * 通过 debug observer 体系输出可视化与统计。
     */
    class DetectorNode : public rclcpp::Node {
    public:
        DetectorNode();
        ~DetectorNode() override;

    private:
        void initParameters();
        void initDebug();
        void initRosbagClients();

        void run(const sensor_msgs::msg::Image::SharedPtr &msg);
        void pollDebugKeys();
        void sendPlayNext();
        void schedulePlayNextRetry();

        // 检测组件
        CameraProvider camera_provider_;
        PoseSolver pose_solver_;
        pose::PoseRefineRunner pose_refiner_;

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

        // Auto test / step playback
        std::size_t processed_frame_count_ = 0;
        std::size_t max_frames_ = 0;
        bool exit_on_complete_ = false;
        bool step_playback_ = false;
        bool play_next_in_flight_ = false;
        bool play_next_needed_ = false;
        std::size_t play_next_retry_count_ = 0;
        rclcpp::TimerBase::SharedPtr play_next_timer_;

        std::size_t frame_index_ = 0;

        std::unique_ptr<Detector> detector_;
    };

} // namespace armor_detector
