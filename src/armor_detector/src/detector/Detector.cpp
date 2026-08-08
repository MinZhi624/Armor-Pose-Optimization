#include "armor_detector/detector/Detector.hpp"
#include "armor_detector/detector/IArmorDetectorBackend.hpp"
#include "armor_detector/detector/TraditionalBackend.hpp"
#include "armor_detector/detector/YoloBackend.hpp"

#include <chrono>
#include <stdexcept>

namespace armor_detector {

    Detector::Detector(const Params &params) :
        backend_type_(params.backend_type), corrector_(params.corner_correction) {
        if (params.backend_type == DetectionBackend::TRADITIONAL) {
            TraditionalArmorDetectorBackend::Params trad_params;
            trad_params.preprocess.gray_threshold = params.traditional.preprocess.gray_threshold;
            trad_params.preprocess.color_threshold = params.traditional.preprocess.color_threshold;
            trad_params.preprocess.target_color = params.target_color;
            trad_params.light.min_contours_area = params.traditional.light.min_contours_area;
            trad_params.light.min_contours_ratio = params.traditional.light.min_contours_ratio;
            trad_params.light.max_contours_ratio = params.traditional.light.max_contours_ratio;
            trad_params.armor.max_angle_diff = params.traditional.armor.max_angle_diff;
            trad_params.armor.min_length_ratio = params.traditional.armor.min_length_ratio;
            trad_params.armor.min_x_diff_ratio = params.traditional.armor.min_x_diff_ratio;
            trad_params.armor.max_y_diff_ratio = params.traditional.armor.max_y_diff_ratio;
            trad_params.armor.max_distance_ratio = params.traditional.armor.max_distance_ratio;
            trad_params.armor.min_distance_ratio = params.traditional.armor.min_distance_ratio;
            trad_params.armor.target_color = params.target_color;
            trad_params.number.model_path = params.traditional.number.model_path;
            trad_params.number.confidence_threshold = params.traditional.number.threshold;

            backend_ = std::make_unique<TraditionalArmorDetectorBackend>(trad_params);
        }
        else if (params.backend_type == DetectionBackend::YOLO) {
            YoloParams yolo_params;
            yolo_params.model_path = params.yolo.model_path;
            yolo_params.device = params.yolo.device;
            yolo_params.input_size = params.yolo.input_size;
            yolo_params.score_threshold = params.yolo.score_threshold;
            yolo_params.nms_threshold = params.yolo.nms_threshold;
            yolo_params.min_confidence = params.yolo.min_confidence;
            yolo_params.max_raw_candidates = params.yolo.max_raw_candidates;
            yolo_params.use_roi = params.yolo.use_roi;
            yolo_params.roi =
                cv::Rect(params.yolo.roi_x, params.yolo.roi_y, params.yolo.roi_width, params.yolo.roi_height);

            backend_ = std::make_unique<YoloArmorDetectorBackend>(yolo_params, params.target_color);
        }
        else {
            throw std::runtime_error("Unknown detector backend");
        }
    }

    Detector::~Detector() = default;

    DetectionResult Detector::detect(const cv::Mat &img_bgr) {
        if (!backend_) {
            throw std::runtime_error("Detector backend not initialized");
        }

        using Clock = std::chrono::steady_clock;
        using ms = std::chrono::duration<double, std::milli>;

        // 1. Backend detection
        auto detect_start = Clock::now();
        DetectionResult result = backend_->detect(img_bgr);

        // 2. Corner correction (after backend, before PnP)
        auto correction_start = Clock::now();
        auto correction = corrector_.correctAll(result.armors, img_bgr);
        auto correction_end = Clock::now();

        const double correction_elapsed_ms = std::chrono::duration_cast<ms>(correction_end - correction_start).count();
        const double detect_total_ms = std::chrono::duration_cast<ms>(correction_end - detect_start).count();

        result.armors = correction.armors;
        result.debug.output_armors = correction.armors;
        result.debug.corner_correction = correction.debug;
        result.debug.corner_correction.elapsed_ms = correction_elapsed_ms;

        bool has_detect_total = false;
        for (auto &timing : result.debug.timings) {
            if (timing.name == "detect_total") {
                timing.elapsed_ms = detect_total_ms;
                has_detect_total = true;
                break;
            }
        }
        if (!has_detect_total) {
            result.debug.timings.push_back({"detect_total", detect_total_ms});
        }
        result.debug.timings.push_back({"corner_correction", correction_elapsed_ms});

        return result;
    }

    DetectionBackend Detector::backend() const {
        return backend_type_;
    }

} // namespace armor_detector
