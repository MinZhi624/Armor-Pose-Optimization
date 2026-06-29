#include "armor_detector/detector/ArmorCornerCorrector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace armor_detector {

    namespace {
        constexpr float EPS = 1e-6f;

        LightBar
        makeLightFromEndpoints(const cv::Point2f &top, const cv::Point2f &bottom, LightBarColor color, int id) {
            LightBar light;
            light.top = top;
            light.bottom = bottom;
            light.center = (top + bottom) * 0.5f;
            light.length = cv::norm(bottom - top);
            light.width = 0.0;
            light.area = 0;
            light.id = id;
            light.color = color;
            if (light.length > EPS) {
                const cv::Point2f axis = bottom - top;
                light.angle = std::atan2(axis.y, axis.x);
                light.rect = cv::RotatedRect(light.center,
                                             cv::Size2f(0.0f, static_cast<float>(light.length)),
                                             static_cast<float>(light.angle * 180.0 / CV_PI));
            }
            return light;
        }

        bool hasUsableEndpoints(const LightBar &light) {
            return cv::norm(light.bottom - light.top) > EPS;
        }
    } // namespace

    ArmorCornerCorrector::ArmorCornerCorrector(const CornerCorrectionParams &params) : params_(params) {
    }

    LightBarCorrectionMethod ArmorCornerCorrector::parseMethod(const std::string &method_str) const {
        if (method_str == "min_area_rect")
            return LightBarCorrectionMethod::MIN_AREA_RECT;
        if (method_str == "pca_gradient")
            return LightBarCorrectionMethod::PCA_GRADIENT;
        return LightBarCorrectionMethod::FIT_ELLIPSE;
    }

    cv::Rect ArmorCornerCorrector::computeLightROI(const LightBar &light, const cv::Size &img_size) const {
        const float length = static_cast<float>(cv::norm(light.bottom - light.top));
        if (length <= EPS || img_size.width <= 0 || img_size.height <= 0) {
            return cv::Rect();
        }

        const float half_band = length * params_.light.max_contours_ratio * 0.5f;
        const float min_x = std::min(light.top.x, light.bottom.x) - half_band;
        const float max_x = std::max(light.top.x, light.bottom.x) + half_band;
        const float min_y = std::min(light.top.y, light.bottom.y) - half_band;
        const float max_y = std::max(light.top.y, light.bottom.y) + half_band;

        int x1 = std::max(0, static_cast<int>(std::floor(min_x)));
        int y1 = std::max(0, static_cast<int>(std::floor(min_y)));
        int x2 = std::min(img_size.width, static_cast<int>(std::ceil(max_x)));
        int y2 = std::min(img_size.height, static_cast<int>(std::ceil(max_y)));
        x1 = std::min(x1, img_size.width);
        y1 = std::min(y1, img_size.height);
        x2 = std::max(x2, 0);
        y2 = std::max(y2, 0);
        return cv::Rect(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
    }

    LightBarCorrectionInput ArmorCornerCorrector::buildCorrectionInput(const LightBar &light,
                                                                       const cv::Mat &gray_img) const {
        LightBarCorrectionInput input;
        input.seed_light = light;
        input.gray_threshold = params_.gray_threshold;
        input.search_half_width_px = static_cast<float>(light.length) * params_.light.max_contours_ratio * 0.5f;
        input.geometry = params_.light;

        cv::Rect roi = computeLightROI(light, gray_img.size());
        if (!roi.empty()) {
            input.gray_roi = gray_img(roi);
            input.roi_offset = cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y));
        }
        return input;
    }

    debug::CornerCorrectionRecord
    ArmorCornerCorrector::correctOne(const DetectedArmor &armor, const cv::Mat &, const cv::Mat &gray_img) const {
        debug::CornerCorrectionRecord record;
        record.original_corners = armor.geometry.corners;
        record.output_corners = armor.geometry.corners;
        record.corrected = false;
        record.method = params_.method;

        LightBar left_seed = armor.geometry.paired_lights[0];
        LightBar right_seed = armor.geometry.paired_lights[1];
        if (!hasUsableEndpoints(left_seed)) {
            left_seed = makeLightFromEndpoints(
                armor.geometry.corners[0], armor.geometry.corners[3], left_seed.color, left_seed.id);
        }
        if (!hasUsableEndpoints(right_seed)) {
            right_seed = makeLightFromEndpoints(
                armor.geometry.corners[1], armor.geometry.corners[2], right_seed.color, right_seed.id);
        }
        if (!hasUsableEndpoints(left_seed)) {
            record.detail = "invalid_left_seed";
            return record;
        }
        if (!hasUsableEndpoints(right_seed)) {
            record.detail = "invalid_right_seed";
            return record;
        }

        record.has_raw_lights = true;
        record.left_raw_light = left_seed;
        record.right_raw_light = right_seed;

        LightBarCorrector corrector;
        LightBarCorrectionMethod method = parseMethod(params_.method);
        auto left_input = buildCorrectionInput(left_seed, gray_img);
        auto right_input = buildCorrectionInput(right_seed, gray_img);
        auto lr = corrector.correct(left_input, method);
        auto rr = corrector.correct(right_input, method);
        record.left_gray_roi = lr.debug_gray_roi;
        record.right_gray_roi = rr.debug_gray_roi;
        record.left_pca_viz = lr.debug_pca_viz;
        record.right_pca_viz = rr.debug_pca_viz;
        if (!lr.corrected) {
            record.detail = "light_correction_failed:left:" + lr.detail;
            return record;
        }
        if (!rr.corrected) {
            record.detail = "light_correction_failed:right:" + rr.detail;
            return record;
        }

        LightBar lo = lr.output_light;
        LightBar ro = rr.output_light;
        std::array<cv::Point2f, 4> oc = {lo.top, ro.top, ro.bottom, lo.bottom};
        float td = 0.0f;
        for (int i = 0; i < 4; ++i)
            td += cv::norm(oc[i] - record.original_corners[i]);
        if (td > params_.max_endpoint_distance_px) {
            record.detail = "distance_reject";
            return record;
        }

        record.output_corners = oc;
        record.has_output_lights = true;
        record.left_output_light = lo;
        record.right_output_light = ro;
        record.corrected = true;
        record.detail = "success";
        return record;
    }

    ArmorCornerCorrectionResult ArmorCornerCorrector::correctAll(const std::vector<DetectedArmor> &armors,
                                                                 const cv::Mat &bgr_img) const {
        ArmorCornerCorrectionResult result;
        result.armors = armors;
        if (bgr_img.empty()) {
            for (const auto &a : armors) {
                debug::CornerCorrectionRecord r;
                r.original_corners = a.geometry.corners;
                r.output_corners = a.geometry.corners;
                r.method = params_.method;
                r.detail = "empty_image";
                result.debug.records.push_back(r);
            }
            return result;
        }
        if (!params_.enabled) {
            for (std::size_t i = 0; i < armors.size(); ++i) {
                debug::CornerCorrectionRecord r;
                r.original_corners = armors[i].geometry.corners;
                r.output_corners = armors[i].geometry.corners;
                r.method = params_.method;
                r.detail = "disabled";
                result.debug.records.push_back(r);
            }
            return result;
        }
        cv::Mat gray_img;
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
        for (std::size_t i = 0; i < armors.size(); ++i) {
            auto record = correctOne(armors[i], bgr_img, gray_img);
            result.debug.records.push_back(record);
            if (record.corrected) {
                result.armors[i].geometry.corners = record.output_corners;
                result.armors[i].geometry.paired_lights = {record.left_output_light, record.right_output_light};
            }
        }
        return result;
    }

} // namespace armor_detector
