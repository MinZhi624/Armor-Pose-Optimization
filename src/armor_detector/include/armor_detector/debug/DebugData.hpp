#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace armor_detector::debug {

    /**
     * @brief 单帧 debug 上下文。
     *
     * source_bgr 只作为观察输入使用，Observer 不应反向修改主流程图像。
     * cv::Mat 生命周期与 clone 规则统一遵守 docs/Conventions.md。
     */
    struct DebugFrameContext {
        std::size_t frame_index = 0;
        builtin_interfaces::msg::Time stamp;
        cv::Mat source_bgr;
        cv::Mat display_bgr; // 显示用图像（Observer 可在其上绘制叠加图层）
    };

    /**
     * @brief 可切换的 debug 图层标识。
     *
     * 用于 DebugLayerState 和数字键 (1-7) 切换。
     * timing 不是图层——它在 debug.show=true 时始终启用。
     */
    enum class DebugLayer {
        UNKNOWN,
        DETECT_STAGE_1, // traditional: preprocess; yolo: letterbox input
        DETECT_STAGE_2, // traditional: lights; yolo: raw candidates after score threshold
        DETECT_STAGE_3, // traditional: armor match; yolo: NMS/filter result
        DETECT_STAGE_4, // traditional: classification; yolo: final detections
        CORNER_CORRECTION,
        POSE,
        POSE_REFINE,
        RESULT,
    };

    /**
     * @brief 按键事件。DebugKeyHandler 负责从 cv::waitKey 的 raw key 转换得到。
     */
    enum class DebugKeyAction {
        NONE,
        EXIT,
        PAUSE_TOGGLE,
        SAVE_ROI,
        STEP_FRAME,
        PLAYBACK_RATE_UP,
        PLAYBACK_RATE_DOWN,
        TOGGLE_LAYER,
    };

    struct DebugKeyEvent {
        DebugKeyAction action = DebugKeyAction::NONE;
        int raw_key = -1;
        DebugLayer layer = DebugLayer::UNKNOWN;
    };

    /**
     * @brief 阶段耗时记录。
     */
    struct StageTiming {
        std::string name;
        double elapsed_ms = 0.0;
    };

    /**
     * @brief 预处理阶段中间图像。
     */
    struct PreprocessDebugData {
        cv::Mat gray;
        cv::Mat img_thre;
        cv::Mat color_mask;
    };

    enum class DebugRejectReason {
        UNKNOWN,
        TOO_SMALL,
        TOO_LARGE,
        BAD_RATIO,
        BAD_ANGLE,
        BAD_COLOR,
        LOW_CONFIDENCE,
        PNP_FAILED,
        BAD_LENGTH_RATIO,
        BAD_X_DIFF,
        BAD_Y_DIFF,
        BAD_DISTANCE,
        DUPLICATE,
        TYPE_MISMATCH,
    };

    struct RejectedLight {
        LightBar light;
        DebugRejectReason reason = DebugRejectReason::UNKNOWN;
        std::string detail;
    };

    struct LightDebugData {
        std::vector<LightBar> accepted_lights;
        std::vector<RejectedLight> rejected_lights;
    };

    struct RejectedArmor {
        ArmorGeometry geometry;
        DebugRejectReason reason = DebugRejectReason::UNKNOWN;
        std::string detail;
    };

    struct ArmorMatchDebugData {
        std::vector<ArmorCandidate> candidates;
        std::vector<RejectedArmor> rejected_armors;
    };

    struct ClassificationDebugData {
        std::vector<DetectedArmor> classified_armors;
        std::vector<cv::Mat> number_rois;
    };

    struct PosePerturbationProjectedCorners {
        bool available = false;
        std::array<cv::Point2f, 4> dir_yaw_corners{};
        std::array<cv::Point2f, 4> dir_pitch_corners{};
        std::array<cv::Point2f, 4> distance_corners{};
        std::array<cv::Point2f, 4> pose_yaw_corners{};
    };

    struct PoseRefineDebugRecord {
        std::size_t armor_index = 0;

        int armor_name = 0;
        std::string armor_type;
        double confidence = 0.0;

        double center_x_px = 0.0;
        double center_y_px = 0.0;

        std::string method;
        bool success = false;

        Eigen::Vector3d initial_xyz_gimbal = Eigen::Vector3d::Zero();
        Eigen::Vector3d final_xyz_gimbal = Eigen::Vector3d::Zero();
        Eigen::Vector3d delta_xyz_gimbal = Eigen::Vector3d::Zero();

        double initial_dir_yaw_rad = 0.0;
        double final_dir_yaw_rad = 0.0;
        double delta_dir_yaw_rad = 0.0;

        double initial_dir_pitch_rad = 0.0;
        double final_dir_pitch_rad = 0.0;
        double delta_dir_pitch_rad = 0.0;

        double initial_distance_m = 0.0;
        double final_distance_m = 0.0;
        double delta_distance_m = 0.0;

        double initial_yaw_rad = 0.0;
        double final_yaw_rad = 0.0;
        double delta_yaw_rad = 0.0;

        double initial_reprojection_error_px = 0.0;
        double final_reprojection_error_px = 0.0;
        double delta_reprojection_error_px = 0.0;

        double initial_reproj_sum_px = 0.0;
        double final_reproj_sum_px = 0.0;
        double delta_reproj_sum_px = 0.0;

        double initial_reproj_mean_px = 0.0;
        double final_reproj_mean_px = 0.0;
        double delta_reproj_mean_px = 0.0;

        double ba_model_initial_reproj_mean_px = 0.0;
        double ba_model_final_reproj_mean_px = 0.0;

        bool has_solver_summary = false;
        double initial_cost = 0.0;
        double final_cost = 0.0;
        double delta_cost = 0.0;
        int num_iterations = 0;
        std::string termination_type = "not_run";

        bool has_projected_corners = false;
        std::array<cv::Point2f, 4> projected_corners{};
        PosePerturbationProjectedCorners perturbation_projected_corners;
    };

    struct PoseLandscapeMetric {
        bool valid = false;
        std::string status = "not_evaluated";
        double cost = std::numeric_limits<double>::quiet_NaN();
        double mean_residual_px = std::numeric_limits<double>::quiet_NaN();
    };

    struct PoseLandscapeGridPoint {
        std::size_t distance_index = 0;
        std::size_t pose_yaw_index = 0;
        double distance_m = std::numeric_limits<double>::quiet_NaN();
        double pose_yaw_rad = std::numeric_limits<double>::quiet_NaN();
        bool valid = false;
        std::string status = "not_evaluated";
        double cost = std::numeric_limits<double>::quiet_NaN();
        double mean_residual_px = std::numeric_limits<double>::quiet_NaN();
    };

    struct PoseLandscapeMarker {
        std::string name;
        bool available = false;
        bool inside_grid = false;
        std::string status = "not_evaluated";
        double distance_m = std::numeric_limits<double>::quiet_NaN();
        double pose_yaw_rad = std::numeric_limits<double>::quiet_NaN();
        PoseLandscapeMetric actual_metric;
        PoseLandscapeMetric slice_metric;
        double direction_delta_yaw_rad = std::numeric_limits<double>::quiet_NaN();
        double direction_delta_pitch_rad = std::numeric_limits<double>::quiet_NaN();
    };

    /**
     * @brief 单个 PnP 样本的固定视线 d-pose_yaw landscape。
     *
     * 只保留数值、字符串和角点值；不保存 cv::Mat 或跨帧图像引用。
     */
    struct PoseLandscapeSample {
        bool valid = false;
        std::string status = "not_analyzed";

        std::size_t armor_index = 0;
        int armor_name = 0;
        std::string armor_type;
        double confidence = 0.0;
        std::array<cv::Point2f, 4> image_corners{};
        std::array<double, 9> camera_matrix{};
        std::array<double, 5> distortion_coefficients{};

        double fixed_dir_yaw_rad = std::numeric_limits<double>::quiet_NaN();
        double fixed_dir_pitch_rad = std::numeric_limits<double>::quiet_NaN();
        double pnp_distance_m = std::numeric_limits<double>::quiet_NaN();
        double pnp_pose_yaw_rad = std::numeric_limits<double>::quiet_NaN();

        double physical_min_distance_m = std::numeric_limits<double>::quiet_NaN();
        double physical_max_distance_m = std::numeric_limits<double>::quiet_NaN();
        double requested_distance_min_m = std::numeric_limits<double>::quiet_NaN();
        double requested_distance_max_m = std::numeric_limits<double>::quiet_NaN();
        double actual_distance_min_m = std::numeric_limits<double>::quiet_NaN();
        double actual_distance_max_m = std::numeric_limits<double>::quiet_NaN();
        double distance_step_m = std::numeric_limits<double>::quiet_NaN();
        double requested_pose_yaw_min_rad = std::numeric_limits<double>::quiet_NaN();
        double requested_pose_yaw_max_rad = std::numeric_limits<double>::quiet_NaN();
        double actual_pose_yaw_min_rad = std::numeric_limits<double>::quiet_NaN();
        double actual_pose_yaw_max_rad = std::numeric_limits<double>::quiet_NaN();
        double pose_yaw_step_rad = std::numeric_limits<double>::quiet_NaN();
        std::size_t distance_count = 0;
        std::size_t pose_yaw_count = 0;
        double huber_loss_scale_px = std::numeric_limits<double>::quiet_NaN();

        bool yaw_search_success = false;
        std::string yaw_search_status = "not_run";
        bool ba_success = false;
        std::string ba_status = "not_run";
        bool ba_solver_summary_available = false;
        double ba_initial_cost = std::numeric_limits<double>::quiet_NaN();
        double ba_final_cost = std::numeric_limits<double>::quiet_NaN();
        int ba_num_iterations = 0;
        std::string ba_termination_type = "not_run";

        double yaw_search_elapsed_ms = 0.0;
        double ba_elapsed_ms = 0.0;
        double grid_elapsed_ms = 0.0;
        double scan_elapsed_ms = 0.0;

        std::vector<PoseLandscapeGridPoint> grid;
        std::vector<PoseLandscapeMarker> markers;
    };

    struct PoseDebugData {
        std::vector<SolvedArmor> solved_armors;
        std::vector<PoseRefineDebugRecord> refine_records;
        std::vector<PoseLandscapeSample> landscape_samples;
        std::vector<StageTiming> timings;
    };
} // namespace armor_detector::debug
