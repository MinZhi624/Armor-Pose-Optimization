#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/detector/LightBarCorrector.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace armor_detector {

    class LightDetector {
    public:
        struct Params {
            int min_contours_area = 30;
            float min_contours_ratio = 0.06f;
            float max_contours_ratio = 0.5f;
        };

        explicit LightDetector(const Params &params);

        std::vector<LightBar> findLights(const cv::Mat &img_thre, const cv::Mat &img_bgr);

        const debug::LightDebugData &getLightDebugData() const {
            return light_debug_;
        }

    private:
        bool checkLightGeometry(const std::vector<cv::Point> &contour) const;

        static LightBarColor findLightColor(const cv::Mat &img_bgr,
                                           const cv::RotatedRect &rect,
                                           const std::vector<cv::Point> &contour);

        static void createLightBar(LightBar &light, const std::vector<cv::Point> &contour);

        Params params_;
        LightBarCorrector corrector_;
        debug::LightDebugData light_debug_;
    };

} // namespace armor_detector
