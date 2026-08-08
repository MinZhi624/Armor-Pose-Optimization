#pragma once

#include "armor_detector/detector/DetectionBackend.hpp"
#include "armor_detector/detector/DetectionResult.hpp"

#include <opencv2/core.hpp>

namespace armor_detector {

    class IArmorDetectorBackend {
    public:
        virtual ~IArmorDetectorBackend() = default;
        virtual DetectionResult detect(const cv::Mat &img_bgr) = 0;
        virtual DetectionBackend backend() const = 0;
    };

} // namespace armor_detector
