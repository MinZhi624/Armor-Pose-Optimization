#pragma once

#include "armor_detector/detector/DetectionBackend.hpp"
#include "armor_detector/types/ArmorData.hpp"
#include "armor_detector/debug/DebugData.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace armor_detector::debug {

    struct TraditionalDetectionDebug {
        PreprocessDebugData preprocess;
        LightDebugData lights;
        ArmorMatchDebugData armor_match;
        ClassificationDebugData classification;
    };

    struct YoloDetectionDebug {
        cv::Mat stage1_source_roi;
        cv::Mat stage1_letterbox;
        std::vector<ClassifiedArmor> stage2_score_filtered;
        std::vector<ClassifiedArmor> stage3_nms_kept;
        std::vector<RejectedArmor> stage3_nms_rejected;
        std::vector<RejectedArmor> stage3_filter_rejected;
        std::vector<ClassifiedArmor> stage4_final;
    };

    struct DetectionDebugData {
        DetectionBackend backend;
        std::vector<ClassifiedArmor> final_armors;
        std::vector<StageTiming> timings;
        TraditionalDetectionDebug traditional;
        YoloDetectionDebug yolo;
    };

} // namespace armor_detector::debug
