#pragma once

#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/detector/LightBarCorrector.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace armor_detector {

    struct ArmorCornerCorrectionResult {
        std::vector<DetectedArmor> armors;
        debug::CornerCorrectionDebugData debug;
    };

    struct ArmorGeometryCheckParams {
        bool enabled = true;
        double max_angle_diff_deg = 30.0;
        double min_length_ratio = 0.45;
        double min_x_diff_ratio = 0.45;
        double max_y_diff_ratio = 1.5;
        double min_distance_ratio = 0.05;
        double max_distance_ratio = 1.2;
    };

    struct CornerCorrectionParams {
        std::string method = "fit_ellipse";
        float roi_scale = 1.25f;
        int gray_threshold = 100;
        float max_endpoint_distance_px = 15.0f;
        LightBarColor target_color = LightBarColor::BLUE;
        LightGeometryParams light;
        ArmorGeometryCheckParams geometry;
    };

    class ArmorCornerCorrector {
    public:
        explicit ArmorCornerCorrector(const CornerCorrectionParams &params);

        ArmorCornerCorrectionResult correctAll(const std::vector<DetectedArmor> &armors, const cv::Mat &bgr_img) const;

    private:
        CornerCorrectionParams params_;

        LightBarCorrectionMethod parseMethod(const std::string &method_str) const;

        cv::Rect computeLightROI(const LightBar &light, const cv::Size &img_size) const;

        LightBarCorrectionInput buildCorrectionInput(const LightBar &light, const cv::Mat &gray_img) const;

        debug::CornerCorrectionRecord correctOne(const DetectedArmor &armor,
                                                 const cv::Mat &,
                                                 const cv::Mat &gray_img,
                                                 LightBarCorrectionMethod method) const;
    };

} // namespace armor_detector
