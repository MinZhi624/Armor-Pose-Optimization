#include "armor_detector/CameraProvider.hpp"
#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugHub.hpp"
#include "armor_detector/debug/DebugData.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>

namespace armor_detector
{

class DetectorNode : public rclcpp::Node
{
private:
    CameraProvider camera_provider_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    debug::DebugGUI debug_gui_;
    debug::DebugHub debug_hub_;
    rclcpp::TimerBase::SharedPtr debug_key_timer_;
    rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_paused_client_;
    rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr play_next_client_;
    std::string debug_gui_window_name_;

    void run(const sensor_msgs::msg::Image::SharedPtr & msg);
    void pollDebugKeys();

public:
    DetectorNode() : Node("armor_detector_node_cpp")
    {
        // 声明参数
        this->declare_parameter<bool>("debug_gui_enabled", true);
        this->declare_parameter<bool>("debug_rosbag_control_enabled", true);
        this->declare_parameter<std::string>("debug_rosbag_player_node", "/rosbag2_player");
        this->declare_parameter<std::string>("debug_gui_window_name", "debug_show");

        bool debug_gui_enabled = this->get_parameter("debug_gui_enabled").as_bool();
        bool rosbag_control_enabled = this->get_parameter("debug_rosbag_control_enabled").as_bool();
        std::string rosbag_player_node = this->get_parameter("debug_rosbag_player_node").as_string();
        debug_gui_window_name_ = this->get_parameter("debug_gui_window_name").as_string();

        camera_provider_.init();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); }
        );

        // Debug GUI
        debug_gui_.setEnabled(debug_gui_enabled);
        debug_gui_.start();

        // Rosbag service clients
        if (rosbag_control_enabled) {
            toggle_paused_client_ = this->create_client<rosbag2_interfaces::srv::TogglePaused>(
                rosbag_player_node + "/toggle_paused");
            play_next_client_ = this->create_client<rosbag2_interfaces::srv::PlayNext>(
                rosbag_player_node + "/play_next");
        }

        // 按键轮询 timer，每 15ms
        debug_key_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(15),
            [this]() { pollDebugKeys(); }
        );

        RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
        RCLCPP_INFO(this->get_logger(), "按键操作: [q/ESC]退出  [Space/p]暂停/继续  [n/→]单步  [s]保存ROI");
    }

    ~DetectorNode() override
    {
        debug_gui_.stop();
    }
};

void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);
    debug_gui_.setFrame(debug_gui_window_name_, frame.image);
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
