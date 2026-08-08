#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/detector/DetectionBackend.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <array>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace armor_detector::debug {

    // ---- Corner correction types (must precede DetectionDebugData) ----

    struct CornerCorrectionRecord {
        std::array<cv::Point2f, 4> original_corners;
        std::array<cv::Point2f, 4> output_corners;
        bool accepted = true;
        bool corrected = false;
        std::string method;
        std::string detail;

        LightBar left_raw_light;
        LightBar right_raw_light;
        LightBar left_output_light;
        LightBar right_output_light;
        bool has_raw_lights = false;
        bool has_output_lights = false;

        cv::Mat left_gray_roi;
        cv::Mat right_gray_roi;
        cv::Mat left_pca_viz;
        cv::Mat right_pca_viz;
    };

    struct CornerCorrectionDebugData {
        double elapsed_ms = 0.0;

        // records[i] corresponds to the input armor candidate before corner correction/filtering.
        std::vector<CornerCorrectionRecord> records;
    };

    // ---- Backend-specific debug ----

    struct TraditionalDetectionDebug {
        PreprocessDebugData preprocess;
        LightDebugData lights;
        ArmorMatchDebugData armor_match;
        ClassificationDebugData classification;
    };

    struct YoloDetectionDebug {
        cv::Mat stage1_source_roi;
        cv::Mat stage1_letterbox;
        std::vector<DetectedArmor> stage2_score_filtered;
        std::vector<DetectedArmor> stage3_nms_kept;
        std::vector<RejectedArmor> stage3_nms_rejected;
        std::vector<RejectedArmor> stage3_filter_rejected;
        std::vector<DetectedArmor> stage4_backend_armors;
    };

    struct DetectionDebugData {
        DetectionBackend backend;
        std::vector<DetectedArmor> output_armors;
        std::vector<StageTiming> timings;
        TraditionalDetectionDebug traditional;
        YoloDetectionDebug yolo;
        CornerCorrectionDebugData corner_correction;
    };

} // namespace armor_detector::debug
