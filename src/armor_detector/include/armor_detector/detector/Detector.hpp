#pragma once

#include "armor_detector/types/ArmorData.hpp"
#include "armor_detector/debug/DebugData.hpp"

#include <opencv2/core.hpp>

namespace armor_detector
{

class Detector
{
public:
    struct Params {
        int gray_threshold = 100;
        int color_threshold = 100;
        LightBarColor target_color = LightBarColor::BLUE;
    };

    explicit Detector(const Params & params);

    cv::Mat preprocess(const cv::Mat & img_bgr);

    const debug::PreprocessDebugData & getPreprocessDebugData() const { return preprocess_debug_; }

private:
    Params params_;
    debug::PreprocessDebugData preprocess_debug_;
};

} // namespace armor_detector
