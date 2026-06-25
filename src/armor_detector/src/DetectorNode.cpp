#include "armor_detector/CameraProvider.hpp"
#include "armor_detector/detector/Detector.hpp"
#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugHub.hpp"
#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/debug/DebugPreprocess.hpp"
#include "armor_detector/debug/DebugLight.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>
#include <rosbag2_interfaces/srv/set_rate.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace armor_detector
{

class DetectorNode : public rclcpp::Node
{
private:
    CameraProvider camera_provider_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // 检测组件
    Detector detector_;
    LightDetector light_detector_;

    // Debug
    debug::DebugGUI debug_gui_;
    debug::DebugHub debug_hub_;
    rclcpp::TimerBase::SharedPtr debug_key_timer_;
    rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_paused_client_;
    rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr play_next_client_;
    rclcpp::Client<rosbag2_interfaces::srv::SetRate>::SharedPtr set_rate_client_;
    std::string debug_gui_window_name_;
    double playback_rate_ = 1.0;
    std::size_t frame_index_ = 0;

    void run(const sensor_msgs::msg::Image::SharedPtr & msg);
    void pollDebugKeys();

public:
    DetectorNode() : Node("armor_detector_node_cpp"),
                     detector_([] {
                         // 临时默认参数，构造函数体中会被 ROS 参数覆盖
                         Detector::Params p;
                         return p;
                     }()),
                     light_detector_([] {
                         LightDetector::Params p;
                         return p;
                     }())
    {
        // 声明参数
        this->declare_parameter<bool>("debug_gui_enabled", true);
        this->declare_parameter<bool>("debug_rosbag_control_enabled", true);
        this->declare_parameter<std::string>("debug_rosbag_player_node", "/rosbag2_player");
        this->declare_parameter<std::string>("debug_gui_window_name", "debug_show");

        // 预处理参数
        this->declare_parameter<int>("gray_threshold", 100);
        this->declare_parameter<int>("color_threshold", 100);
        this->declare_parameter<std::string>("target_color", "BLUE");

        // 灯条参数
        this->declare_parameter<int>("min_contours_area", 30);
        this->declare_parameter<double>("min_contours_ratio", 0.06);
        this->declare_parameter<double>("max_contours_ratio", 0.5);

        // 读取参数
        bool debug_gui_enabled = this->get_parameter("debug_gui_enabled").as_bool();
        bool rosbag_control_enabled = this->get_parameter("debug_rosbag_control_enabled").as_bool();
        std::string rosbag_player_node = this->get_parameter("debug_rosbag_player_node").as_string();
        debug_gui_window_name_ = this->get_parameter("debug_gui_window_name").as_string();

        // 用 ROS 参数重建检测器
        std::string target_color_str = this->get_parameter("target_color").as_string();
        LightBarColor target_color = (target_color_str == "RED") ? LightBarColor::RED : LightBarColor::BLUE;

        Detector::Params det_params;
        det_params.gray_threshold = this->get_parameter("gray_threshold").as_int();
        det_params.color_threshold = this->get_parameter("color_threshold").as_int();
        det_params.target_color = target_color;
        detector_ = Detector(det_params);

        LightDetector::Params light_params;
        light_params.min_contours_area = this->get_parameter("min_contours_area").as_int();
        light_params.min_contours_ratio = static_cast<float>(this->get_parameter("min_contours_ratio").as_double());
        light_params.max_contours_ratio = static_cast<float>(this->get_parameter("max_contours_ratio").as_double());
        light_detector_ = LightDetector(light_params);

        camera_provider_.init();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); }
        );

        // Debug GUI
        debug_gui_.setEnabled(debug_gui_enabled);
        debug_gui_.start();

        // Debug Observer 注册
        debug_hub_.addObserver(std::make_shared<debug::DebugPreprocessView>(debug_gui_));
        debug_hub_.addObserver(std::make_shared<debug::DebugLightView>(debug_gui_));

        // Rosbag service clients
        if (rosbag_control_enabled) {
            toggle_paused_client_ = this->create_client<rosbag2_interfaces::srv::TogglePaused>(
                rosbag_player_node + "/toggle_paused");
            play_next_client_ = this->create_client<rosbag2_interfaces::srv::PlayNext>(
                rosbag_player_node + "/play_next");
            set_rate_client_ = this->create_client<rosbag2_interfaces::srv::SetRate>(
                rosbag_player_node + "/set_rate");
        }

        // 按键轮询 timer
        debug_key_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(15),
            [this]() { pollDebugKeys(); }
        );

        RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
        RCLCPP_INFO(this->get_logger(), "按键操作: [q/ESC]退出  [Space/p]暂停/继续  [n/→]单步  [s]保存ROI  [+/-]加速/减速");
    }

    ~DetectorNode() override
    {
        debug_gui_.stop();
    }
};

void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);

    // 构建 debug 上下文
    debug::DebugFrameContext ctx;
    ctx.frame_index = frame_index_++;
    ctx.stamp = frame.timestamp;
    ctx.source_bgr = frame.image;
    ctx.display_bgr = frame.image.clone();

    // 预处理
    cv::Mat img_thre = detector_.preprocess(frame.image);
    debug_hub_.onPreprocess(ctx, detector_.getPreprocessDebugData());

    // 灯条检测
    auto lights = light_detector_.findLights(img_thre, frame.image);
    debug_hub_.onLights(ctx, light_detector_.getLightDebugData());

    // 一次性提交最终显示图像
    debug_gui_.setFrame(debug_gui_window_name_, ctx.display_bgr);
}

void DetectorNode::pollDebugKeys()
{
    auto events = debug_gui_.takeKeyEvents();
    for (const auto & event : events) {
        switch (event.action) {
        case debug::DebugKeyAction::EXIT:
            debug_gui_.stop();
            rclcpp::shutdown();
            break;

        case debug::DebugKeyAction::PAUSE_TOGGLE:
            if (!toggle_paused_client_ || !toggle_paused_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag toggle_paused service not ready");
                continue;
            }
            (void)toggle_paused_client_->async_send_request(
                std::make_shared<rosbag2_interfaces::srv::TogglePaused::Request>());
            break;

        case debug::DebugKeyAction::STEP_FRAME:
            if (!play_next_client_ || !play_next_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag play_next service not ready");
                continue;
            }
            (void)play_next_client_->async_send_request(
                std::make_shared<rosbag2_interfaces::srv::PlayNext::Request>());
            break;

        case debug::DebugKeyAction::SAVE_ROI:
            debug_hub_.onKey(event);
            break;

        case debug::DebugKeyAction::PLAYBACK_RATE_UP:
        case debug::DebugKeyAction::PLAYBACK_RATE_DOWN: {
            if (!set_rate_client_ || !set_rate_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag set_rate service not ready");
                continue;
            }
            double new_rate = (event.action == debug::DebugKeyAction::PLAYBACK_RATE_UP)
                                  ? playback_rate_ * 1.1
                                  : playback_rate_ / 1.1;
            new_rate = std::clamp(new_rate, 0.1, 10.0);
            auto req = std::make_shared<rosbag2_interfaces::srv::SetRate::Request>();
            req->rate = new_rate;
            (void)set_rate_client_->async_send_request(req);
            playback_rate_ = new_rate;
            RCLCPP_INFO(this->get_logger(), "设置播放倍率: %.2fx", playback_rate_);
            break;
        }

        default:
            break;
        }
    }
}

} // namespace armor_detector

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
