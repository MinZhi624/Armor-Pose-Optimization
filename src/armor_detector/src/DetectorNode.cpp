#include "armor_detector/DetectorNode.hpp"
#include "armor_detector/debug/DebugLayerController.hpp"
#include "armor_detector/debug/DebugLight.hpp"
#include "armor_detector/debug/DebugPreprocess.hpp"
#include "armor_detector/debug/DebugTiming.hpp"

#include <algorithm>
#include <memory>

namespace armor_detector
{

DetectorNode::DetectorNode()
    : Node("armor_detector_node_cpp"),
      detector_(Detector::Params{}),
      light_detector_(LightDetector::Params{})
{
    initParameters();
    initDetectors();
    camera_provider_.init();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); });

    initDebug();
    initRosbagClients();

    RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
}

DetectorNode::~DetectorNode()
{
    debug_gui_.stop();
}

// ===================== init 方法 =====================

void DetectorNode::initParameters()
{
    // Debug 配置
    debug_config_.show = this->declare_parameter<bool>("debug.show", true);
    debug_config_.rosbag_control = this->declare_parameter<bool>("debug.rosbag_control", true);
    debug_config_.rosbag_player_node =
        this->declare_parameter<std::string>("debug.rosbag_player_node", "/rosbag2_player");
    debug_config_.preprocess = this->declare_parameter<bool>("debug.preprocess", false);
    debug_config_.lights = this->declare_parameter<bool>("debug.lights", true);
    debug_config_.armor_match = this->declare_parameter<bool>("debug.armor_match", false);
    debug_config_.classification = this->declare_parameter<bool>("debug.classification", false);
    debug_config_.pose = this->declare_parameter<bool>("debug.pose", false);
    debug_config_.stats_interval =
        static_cast<std::size_t>(this->declare_parameter<int>("debug.stats_interval", 50));
}

void DetectorNode::initDetectors()
{
    std::string target_color_str = this->declare_parameter<std::string>("target_color", "BLUE");
    LightBarColor target_color =
        (target_color_str == "RED") ? LightBarColor::RED : LightBarColor::BLUE;

    Detector::Params det_params;
    det_params.gray_threshold = this->declare_parameter<int>("gray_threshold", 100);
    det_params.color_threshold = this->declare_parameter<int>("color_threshold", 100);
    det_params.target_color = target_color;
    detector_ = Detector(det_params);

    LightDetector::Params light_params;
    light_params.min_contours_area = this->declare_parameter<int>("min_contours_area", 30);
    light_params.min_contours_ratio =
        static_cast<float>(this->declare_parameter<double>("min_contours_ratio", 0.06));
    light_params.max_contours_ratio =
        static_cast<float>(this->declare_parameter<double>("max_contours_ratio", 0.5));
    light_detector_ = LightDetector(light_params);
}

void DetectorNode::initDebug()
{
    if (!debug_config_.show) {
        RCLCPP_INFO(this->get_logger(), "Debug GUI 已禁用 (debug.show=false)");
        return;
    }

    debug_gui_.setEnabled(true);
    debug_gui_.start();

    // 初始化图层状态（从 config 读取初始值）
    layer_state_.setEnabled(debug::DebugLayer::PREPROCESS, debug_config_.preprocess);
    layer_state_.setEnabled(debug::DebugLayer::LIGHTS, debug_config_.lights);
    layer_state_.setEnabled(debug::DebugLayer::ARMOR_MATCH, debug_config_.armor_match);
    layer_state_.setEnabled(debug::DebugLayer::CLASSIFICATION, debug_config_.classification);
    layer_state_.setEnabled(debug::DebugLayer::POSE, debug_config_.pose);

    // 始终注册 timing observer
    debug_hub_.addObserver(
        std::make_shared<debug::DebugTiming>(debug_config_.stats_interval));

    // 注册图层 view observer（内部根据 layer state 决定是否绘制）
    debug_hub_.addObserver(
        std::make_shared<debug::DebugPreprocessView>(debug_gui_, layer_state_));
    debug_hub_.addObserver(
        std::make_shared<debug::DebugLightView>(debug_gui_, layer_state_));

    // 注册图层控制器（处理数字键）
    debug_hub_.addObserver(
        std::make_shared<debug::DebugLayerController>(layer_state_));

    // 按键轮询 timer
    debug_key_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(15),
        [this]() { pollDebugKeys(); });

    RCLCPP_INFO(this->get_logger(),
                "按键操作: [q/ESC]退出  [Space/p]暂停/继续  [n/→]单步  "
                "[s]保存ROI  [+/-]加速/减速  [1-5]切换图层");
}

void DetectorNode::initRosbagClients()
{
    if (!debug_config_.rosbag_control) {
        return;
    }
    const auto & node = debug_config_.rosbag_player_node;
    toggle_paused_client_ = this->create_client<rosbag2_interfaces::srv::TogglePaused>(
        node + "/toggle_paused");
    play_next_client_ = this->create_client<rosbag2_interfaces::srv::PlayNext>(
        node + "/play_next");
    set_rate_client_ = this->create_client<rosbag2_interfaces::srv::SetRate>(
        node + "/set_rate");
}

// ===================== 主流程 =====================

void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);

    debug::DebugFrameContext ctx;
    ctx.frame_index = frame_index_++;
    ctx.stamp = frame.timestamp;
    ctx.source_bgr = frame.image;
    ctx.display_bgr = frame.image.clone();

    debug_hub_.onFrameStart(ctx);

    cv::Mat img_thre = detector_.preprocess(frame.image);
    debug_hub_.onPreprocess(ctx, detector_.getPreprocessDebugData());

    auto lights = light_detector_.findLights(img_thre, frame.image);
    debug_hub_.onLights(ctx, light_detector_.getLightDebugData());
    (void)lights;

    debug_hub_.onFrameEnd(ctx);

    if (debug_config_.show) {
        debug_gui_.setFrame("debug_show", ctx.display_bgr);
    }
}

// ===================== 按键处理 =====================

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

        case debug::DebugKeyAction::TOGGLE_LAYER:
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
