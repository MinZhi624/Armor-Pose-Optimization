#pragma once

#include "armor_detector/NumberClassifier.hpp"
#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/detector/ArmorDetector.hpp"
#include "armor_detector/detector/IArmorDetectorBackend.hpp"
#include "armor_detector/detector/LightDetector.hpp"

#include <opencv2/core.hpp>

namespace armor_detector {

    class TraditionalArmorDetectorBackend : public IArmorDetectorBackend {
    public:
        struct Params {
            struct {
                int gray_threshold = 100;
                int color_threshold = 100;
                LightBarColor target_color = LightBarColor::BLUE;
            } preprocess;
            LightDetector::Params light;
            ArmorDetector::Params armor;
            NumberClassifier::Params number;
        };

        explicit TraditionalArmorDetectorBackend(const Params &params);

        DetectionResult detect(const cv::Mat &img_bgr) override;
        DetectionBackend backend() const override {
            return DetectionBackend::TRADITIONAL;
        }

    private:
        cv::Mat preprocess(const cv::Mat &img_bgr);

        Params params_;
        LightDetector light_detector_;
        ArmorDetector armor_detector_;
        NumberClassifier number_classifier_;
        debug::PreprocessDebugData preprocess_debug_;
    };

} // namespace armor_detector
