#include "armor_detector/DetectorNode.hpp"
#include "armor_detector/debug/DebugArmorMatch.hpp"
#include "armor_detector/debug/DebugClassification.hpp"
#include "armor_detector/debug/DebugLayerController.hpp"
#include "armor_detector/debug/DebugLight.hpp"
#include "armor_detector/debug/DebugPreprocess.hpp"
#include "armor_detector/debug/DebugResult.hpp"
#include "armor_detector/debug/DebugTiming.hpp"
#include "armor_detector/debug/DebugYolo.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <memory>

namespace armor_detector {

    DetectorNode::DetectorNode() : Node("armor_detector_node_cpp") {
        initParameters();
        camera_provider_.init();
        pose_solver_.init(camera_provider_.getCameraInfo());

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(10), [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); });

        initDebug();
        initRosbagClients();

        // Step 模式：等待 service ready 后发送第一帧请求
        if (step_playback_) {
            RCLCPP_INFO(this->get_logger(), "Step 播放模式: 等待 play_next service...");
            play_next_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
                if (play_next_client_ && play_next_client_->service_is_ready()) {
                    play_next_timer_->cancel();
                    sendPlayNext();
                }
            });
        }

        RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
    }

    DetectorNode::~DetectorNode() {
        debug_gui_.stop();
    }

    // ===================== init 方法 =====================

    void DetectorNode::initParameters() {
        // Debug 配置
        debug_config_.show = this->declare_parameter<bool>("debug.show", true);
        debug_config_.rosbag_control = this->declare_parameter<bool>("debug.rosbag_control", true);
        debug_config_.rosbag_player_node =
            this->declare_parameter<std::string>("debug.rosbag_player_node", "/rosbag2_player");
        debug_config_.detect_stage_1 = this->declare_parameter<bool>("debug.detect_stage_1", false);
        debug_config_.detect_stage_2 = this->declare_parameter<bool>("debug.detect_stage_2", false);
        debug_config_.detect_stage_3 = this->declare_parameter<bool>("debug.detect_stage_3", false);
        debug_config_.detect_stage_4 = this->declare_parameter<bool>("debug.detect_stage_4", false);
        debug_config_.pose = this->declare_parameter<bool>("debug.pose", false);
        debug_config_.result = this->declare_parameter<bool>("debug.result", true);
        debug_config_.stats_interval =
            static_cast<std::size_t>(this->declare_parameter<int>("debug.stats_interval", 50));

        // Playback 配置
        std::string playback_mode = this->declare_parameter<std::string>("playback.mode", "realtime");
        step_playback_ = (playback_mode == "step");
        max_frames_ = static_cast<std::size_t>(this->declare_parameter<int>("playback.max_frames", 0));
        exit_on_complete_ = this->declare_parameter<bool>("playback.exit_on_complete", false);

        // --- Backend selection ---
        declare_parameter("detector.backend", "traditional");
        declare_parameter("detector.target_color", "RED");
        std::string backend_str;
        std::string target_color_str;
        get_parameter("detector.backend", backend_str);
        get_parameter("detector.target_color", target_color_str);
        LightBarColor target_color = (target_color_str == "RED") ? LightBarColor::RED : LightBarColor::BLUE;

        auto package_share = ament_index_cpp::get_package_share_directory("armor_detector");

        Detector::Params detector_params;
        detector_params.target_color = target_color;

        if (backend_str == "traditional") {
            detector_params.backend_type = DetectionBackend::TRADITIONAL;

            declare_parameter("detector.traditional.preprocess.gray_threshold", 100);
            declare_parameter("detector.traditional.preprocess.color_threshold", 100);
            declare_parameter("detector.traditional.light.min_contours_area", 30);
            declare_parameter("detector.traditional.light.min_contours_ratio", 0.06);
            declare_parameter("detector.traditional.light.max_contours_ratio", 0.5);
            declare_parameter("detector.traditional.armor.max_angle_diff", 15.0);
            declare_parameter("detector.traditional.armor.min_length_ratio", 0.7);
            declare_parameter("detector.traditional.armor.min_x_diff_ratio", 0.75);
            declare_parameter("detector.traditional.armor.max_y_diff_ratio", 1.0);
            declare_parameter("detector.traditional.armor.max_distance_ratio", 0.8);
            declare_parameter("detector.traditional.armor.min_distance_ratio", 0.1);
            declare_parameter("detector.traditional.number.model_path", "model/number_cnn.onnx");
            declare_parameter("detector.traditional.number.threshold", 0.5);

            get_parameter("detector.traditional.preprocess.gray_threshold",
                          detector_params.traditional.preprocess.gray_threshold);
            get_parameter("detector.traditional.preprocess.color_threshold",
                          detector_params.traditional.preprocess.color_threshold);
            get_parameter("detector.traditional.light.min_contours_area",
                          detector_params.traditional.light.min_contours_area);
            get_parameter("detector.traditional.light.min_contours_ratio",
                          detector_params.traditional.light.min_contours_ratio);
            get_parameter("detector.traditional.light.max_contours_ratio",
                          detector_params.traditional.light.max_contours_ratio);
            get_parameter("detector.traditional.armor.max_angle_diff",
                          detector_params.traditional.armor.max_angle_diff);
            get_parameter("detector.traditional.armor.min_length_ratio",
                          detector_params.traditional.armor.min_length_ratio);
            get_parameter("detector.traditional.armor.min_x_diff_ratio",
                          detector_params.traditional.armor.min_x_diff_ratio);
            get_parameter("detector.traditional.armor.max_y_diff_ratio",
                          detector_params.traditional.armor.max_y_diff_ratio);
            get_parameter("detector.traditional.armor.max_distance_ratio",
                          detector_params.traditional.armor.max_distance_ratio);
            get_parameter("detector.traditional.armor.min_distance_ratio",
                          detector_params.traditional.armor.min_distance_ratio);
            std::string num_model_relative;
            get_parameter("detector.traditional.number.model_path", num_model_relative);
            detector_params.traditional.number.model_path = package_share + "/" + num_model_relative;
            float num_threshold;
            get_parameter("detector.traditional.number.threshold", num_threshold);
            detector_params.traditional.number.threshold = num_threshold;
        }
        else if (backend_str == "yolo") {
            detector_params.backend_type = DetectionBackend::YOLO;

            declare_parameter("detector.yolo.model_path", "model/robot_0526.onnx");
            declare_parameter("detector.yolo.device", "CPU");
            declare_parameter("detector.yolo.input_size", 640);
            declare_parameter("detector.yolo.score_threshold", 0.65);
            declare_parameter("detector.yolo.nms_threshold", 0.45);
            declare_parameter("detector.yolo.min_confidence", 0.65);
            declare_parameter("detector.yolo.max_raw_candidates", 100);
            declare_parameter("detector.yolo.use_roi", false);
            declare_parameter("detector.yolo.roi.x", 0);
            declare_parameter("detector.yolo.roi.y", 0);
            declare_parameter("detector.yolo.roi.width", -1);
            declare_parameter("detector.yolo.roi.height", -1);

            std::string yolo_model_relative;
            get_parameter("detector.yolo.model_path", yolo_model_relative);
            detector_params.yolo.model_path = package_share + "/" + yolo_model_relative;
            get_parameter("detector.yolo.device", detector_params.yolo.device);
            get_parameter("detector.yolo.input_size", detector_params.yolo.input_size);
            get_parameter("detector.yolo.score_threshold", detector_params.yolo.score_threshold);
            get_parameter("detector.yolo.nms_threshold", detector_params.yolo.nms_threshold);
            get_parameter("detector.yolo.min_confidence", detector_params.yolo.min_confidence);
            get_parameter("detector.yolo.max_raw_candidates", detector_params.yolo.max_raw_candidates);
            get_parameter("detector.yolo.use_roi", detector_params.yolo.use_roi);
            get_parameter("detector.yolo.roi.x", detector_params.yolo.roi_x);
            get_parameter("detector.yolo.roi.y", detector_params.yolo.roi_y);
            get_parameter("detector.yolo.roi.width", detector_params.yolo.roi_width);
            get_parameter("detector.yolo.roi.height", detector_params.yolo.roi_height);
        }
        else {
            throw std::runtime_error("Unknown detector backend: " + backend_str);
        }

        detector_ = std::make_unique<Detector>(detector_params);
        RCLCPP_INFO(get_logger(), "Detector backend: %s", backend_str.c_str());
    }

    void DetectorNode::initDebug() {
        // 初始化图层状态（从 config 读取初始值）
        layer_state_.setEnabled(debug::DebugLayer::DETECT_STAGE_1, debug_config_.detect_stage_1);
        layer_state_.setEnabled(debug::DebugLayer::DETECT_STAGE_2, debug_config_.detect_stage_2);
        layer_state_.setEnabled(debug::DebugLayer::DETECT_STAGE_3, debug_config_.detect_stage_3);
        layer_state_.setEnabled(debug::DebugLayer::DETECT_STAGE_4, debug_config_.detect_stage_4);
        layer_state_.setEnabled(debug::DebugLayer::POSE, debug_config_.pose);
        layer_state_.setEnabled(debug::DebugLayer::RESULT, debug_config_.result);

        // 非 GUI observer：始终注册
        debug_hub_.addObserver(std::make_shared<debug::DebugTiming>(debug_config_.stats_interval));
        debug_hub_.addObserver(std::make_shared<debug::DebugPoseMarkerPublisher>(*this, layer_state_));

        // GUI observer：仅 debug.show=true 时注册
        if (!debug_config_.show) {
            RCLCPP_INFO(this->get_logger(), "Debug GUI 已禁用 (debug.show=false)");
            return;
        }

        debug_gui_.setEnabled(true);
        debug_gui_.start();

        debug_hub_.addObserver(std::make_shared<debug::DebugPreprocessView>(debug_gui_, layer_state_));
        debug_hub_.addObserver(std::make_shared<debug::DebugLightView>(debug_gui_, layer_state_));
        debug_hub_.addObserver(std::make_shared<debug::DebugArmorMatchView>(debug_gui_, layer_state_));
        debug_hub_.addObserver(std::make_shared<debug::DebugResultView>(debug_gui_, layer_state_));
        debug_hub_.addObserver(std::make_shared<debug::DebugClassificationView>(debug_gui_, layer_state_));
        debug_hub_.addObserver(std::make_shared<debug::DebugYoloView>(debug_gui_, layer_state_));
        debug_hub_.addObserver(std::make_shared<debug::DebugLayerController>(layer_state_));

        debug_key_timer_ = this->create_wall_timer(std::chrono::milliseconds(15), [this]() { pollDebugKeys(); });

        RCLCPP_INFO(this->get_logger(),
                    "按键操作: [q/ESC]退出  [Space/p]暂停/继续  [n/→]单步  "
                    "[s]保存ROI  [+/-]加速/减速  [1-6]切换图层");
    }

    void DetectorNode::initRosbagClients() {
        if (!debug_config_.rosbag_control) {
            return;
        }
        const auto &node = debug_config_.rosbag_player_node;
        toggle_paused_client_ = this->create_client<rosbag2_interfaces::srv::TogglePaused>(node + "/toggle_paused");
        play_next_client_ = this->create_client<rosbag2_interfaces::srv::PlayNext>(node + "/play_next");
        set_rate_client_ = this->create_client<rosbag2_interfaces::srv::SetRate>(node + "/set_rate");
    }

    // ===================== 主流程 =====================

    void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr &msg) {
        Frame frame = camera_provider_.receiveImage(msg);

        debug::DebugFrameContext ctx;
        ctx.frame_index = frame_index_++;
        ctx.stamp = frame.timestamp;
        ctx.source_bgr = frame.image;

        // Only clone display_bgr when debug is enabled
        if (debug_config_.show) {
            ctx.display_bgr = frame.image.clone();
        }

        debug_hub_.onFrameStart(ctx);

        // Unified detection via backend
        DetectionResult result = detector_->detect(frame.image);
        debug_hub_.onDetection(ctx, result.debug);

        auto solved = pose_solver_.solve(result.armors);
        debug_hub_.onPoseSolved(ctx, pose_solver_.getPoseDebugData());

        debug_hub_.onFrameEnd(ctx);

        if (debug_config_.show) {
            debug_gui_.setFrame("debug_show", ctx.display_bgr);
        }

        // 帧计数与 step 播放
        ++processed_frame_count_;

        if (processed_frame_count_ == 1) {
            RCLCPP_INFO(this->get_logger(), "第一帧处理完成");
        }

        if (max_frames_ > 0 && processed_frame_count_ >= max_frames_) {
            RCLCPP_INFO(this->get_logger(), "auto test completed: %zu frames", processed_frame_count_);
            rclcpp::shutdown();
            return;
        }

        if (step_playback_) {
            play_next_needed_ = true;
            sendPlayNext();
        }
    }

    // ===================== Step 播放 =====================

    void DetectorNode::sendPlayNext() {
        if (play_next_in_flight_ || !play_next_client_ || !play_next_client_->service_is_ready()) {
            return;
        }
        play_next_in_flight_ = true;
        play_next_needed_ = false;
        (void)play_next_client_->async_send_request(
            std::make_shared<rosbag2_interfaces::srv::PlayNext::Request>(),
            [this](rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedFuture future) {
                play_next_in_flight_ = false;
                if (!future.get()->success) {
                    RCLCPP_ERROR(this->get_logger(), "play_next 失败 (frame=%zu)", processed_frame_count_);
                    rclcpp::shutdown();
                    return;
                }
                if (play_next_needed_) {
                    sendPlayNext();
                }
            });
    }

    // ===================== 按键处理 =====================

    void DetectorNode::pollDebugKeys() {
        auto events = debug_gui_.takeKeyEvents();
        for (const auto &event : events) {
            switch (event.action) {
                case debug::DebugKeyAction::EXIT:
                    debug_gui_.stop();
                    rclcpp::shutdown();
                    break;

                case debug::DebugKeyAction::PAUSE_TOGGLE:
                    if (!toggle_paused_client_ || !toggle_paused_client_->service_is_ready()) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 2000, "rosbag toggle_paused service not ready");
                        continue;
                    }
                    (void)toggle_paused_client_->async_send_request(
                        std::make_shared<rosbag2_interfaces::srv::TogglePaused::Request>());
                    break;

                case debug::DebugKeyAction::STEP_FRAME:
                    if (!play_next_client_ || !play_next_client_->service_is_ready()) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 2000, "rosbag play_next service not ready");
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
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(), *this->get_clock(), 2000, "rosbag set_rate service not ready");
                        continue;
                    }
                    double new_rate = (event.action == debug::DebugKeyAction::PLAYBACK_RATE_UP) ? playback_rate_ * 1.1
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

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
