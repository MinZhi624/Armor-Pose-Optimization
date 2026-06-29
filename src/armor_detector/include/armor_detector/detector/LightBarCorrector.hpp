#pragma once

#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>
#include <string>

namespace armor_detector {

    struct LightBarCorrectionInput {
        LightBar seed_light; // full-image coordinates, color inherited from upstream detection
        cv::Mat gray_roi; // single-light local ROI
        cv::Point2f roi_offset; // ROI top-left in full-image coordinates
        int gray_threshold = 100;
        float search_half_width_px = 0.0f;
        LightGeometryParams geometry;
    };

    enum class LightBarCorrectionMethod {
        MIN_AREA_RECT,
        FIT_ELLIPSE,
        PCA_GRADIENT,
    };

    struct LightBarCorrectionResult {
        bool corrected = false;
        LightBar raw_light;
        LightBar output_light;
        std::string method;
        std::string detail;
        cv::Mat debug_gray_roi;
        cv::Mat debug_pca_viz;
    };

    class LightBarCorrector {
    public:
        /// Dispatch: apply correction method to a local light ROI.
        LightBarCorrectionResult correct(const LightBarCorrectionInput &input,
                                         LightBarCorrectionMethod method = LightBarCorrectionMethod::FIT_ELLIPSE) const;

    private:
        LightBarCorrectionResult correctByMinAreaRect(const LightBarCorrectionInput &input) const;
        LightBarCorrectionResult correctByEllipse(const LightBarCorrectionInput &input) const;
        LightBarCorrectionResult correctByPCAGradient(const LightBarCorrectionInput &input) const;
    };

} // namespace armor_detector
