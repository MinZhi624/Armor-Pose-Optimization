#include "armor_detector/detector/ArmorCornerCorrector.hpp"
#include "armor_detector/tools/angle.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <utility>

namespace armor_detector {

    namespace {
        constexpr float EPS = 1e-6f;

        struct GeometryMetrics {
            double angle_diff_deg = 0.0;
            double length_diff = 0.0;
            double x_diff_ratio = 0.0;
            double y_diff_ratio = 0.0;
            double distance_ratio = 0.0;
        };

        bool isFinitePoint(const cv::Point2f &point) {
            return std::isfinite(point.x) && std::isfinite(point.y);
        }

        void setDetail(std::string *detail, const std::string &message) {
            if (detail) {
                *detail = message;
            }
        }

        std::string thresholdDetail(const char *name, double value, const char *op, double limit) {
            std::ostringstream oss;
            oss << name << "=" << value << " " << op << " " << limit;
            return oss.str();
        }

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
            return isFinitePoint(light.top) && isFinitePoint(light.bottom) && cv::norm(light.bottom - light.top) > EPS;
        }

        bool computeGeometryMetrics(const LightBar &left,
                                    const LightBar &right,
                                    GeometryMetrics *metrics,
                                    std::string *detail) {
            if (!hasUsableEndpoints(left)) {
                setDetail(detail, "invalid_left_light");
                return false;
            }
            if (!hasUsableEndpoints(right)) {
                setDetail(detail, "invalid_right_light");
                return false;
            }

            const cv::Point2f left_axis = left.bottom - left.top;
            const cv::Point2f right_axis = right.bottom - right.top;
            const double left_length = cv::norm(left_axis);
            const double right_length = cv::norm(right_axis);
            const cv::Point2f left_center = (left.top + left.bottom) * 0.5f;
            const cv::Point2f right_center = (right.top + right.bottom) * 0.5f;

            if (right_center.x - left_center.x <= EPS) {
                setDetail(detail, "light_order");
                return false;
            }

            const double max_len = std::max(left_length, right_length);
            const double min_len = std::min(left_length, right_length);
            const double mean_length = (left_length + right_length) * 0.5;
            if (max_len <= EPS || mean_length <= EPS) {
                setDetail(detail, "zero_length");
                return false;
            }

            const double left_angle = std::atan2(left_axis.y, left_axis.x);
            const double right_angle = std::atan2(right_axis.y, right_axis.x);
            double angle_diff_deg = std::abs(tools::radToDeg(tools::normalizeRadAngle(left_angle - right_angle)));
            angle_diff_deg = std::min(angle_diff_deg, 180.0 - angle_diff_deg);

            const double global_x_diff = right_center.x - left_center.x;
            const double global_y_diff = right_center.y - left_center.y;
            const double sinx = std::sin(left_angle);
            const double cosx = std::cos(left_angle);
            const double local_x = std::abs(-global_x_diff * sinx + global_y_diff * cosx);
            const double local_y = std::abs(global_x_diff * cosx + global_y_diff * sinx);
            const double distance = cv::norm(left_center - right_center);
            if (distance <= EPS) {
                setDetail(detail, "zero_distance");
                return false;
            }

            GeometryMetrics computed;
            computed.angle_diff_deg = angle_diff_deg;
            computed.length_diff = min_len / max_len;
            computed.x_diff_ratio = local_x / mean_length;
            computed.y_diff_ratio = local_y / mean_length;
            computed.distance_ratio = mean_length / distance;

            if (!std::isfinite(computed.angle_diff_deg) || !std::isfinite(computed.length_diff) ||
                !std::isfinite(computed.x_diff_ratio) || !std::isfinite(computed.y_diff_ratio) ||
                !std::isfinite(computed.distance_ratio)) {
                setDetail(detail, "non_finite_geometry");
                return false;
            }

            if (metrics) {
                *metrics = computed;
            }
            return true;
        }

        bool checkArmorGeometry(const LightBar &left,
                                const LightBar &right,
                                ArmorType type,
                                const ArmorGeometryCheckParams &params,
                                GeometryMetrics *metrics,
                                std::string *detail) {
            if (!params.enabled) {
                return true;
            }

            GeometryMetrics computed;
            if (!computeGeometryMetrics(left, right, &computed, detail)) {
                return false;
            }
            if (metrics) {
                *metrics = computed;
            }

            if (type == ArmorType::NONE) {
                setDetail(detail, "type=NONE");
                return false;
            }
            if (computed.angle_diff_deg > params.max_angle_diff_deg) {
                setDetail(detail, thresholdDetail("angle", computed.angle_diff_deg, ">", params.max_angle_diff_deg));
                return false;
            }
            if (computed.length_diff < params.min_length_ratio) {
                setDetail(detail, thresholdDetail("len_ratio", computed.length_diff, "<", params.min_length_ratio));
                return false;
            }
            if (computed.x_diff_ratio < params.min_x_diff_ratio) {
                setDetail(detail, thresholdDetail("x_diff", computed.x_diff_ratio, "<", params.min_x_diff_ratio));
                return false;
            }
            if (computed.y_diff_ratio > params.max_y_diff_ratio) {
                setDetail(detail, thresholdDetail("y_diff", computed.y_diff_ratio, ">", params.max_y_diff_ratio));
                return false;
            }
            if (computed.distance_ratio < params.min_distance_ratio ||
                computed.distance_ratio > params.max_distance_ratio) {
                std::ostringstream oss;
                oss << "dist_ratio=" << computed.distance_ratio << " not in [" << params.min_distance_ratio << ", "
                    << params.max_distance_ratio << "]";
                setDetail(detail, oss.str());
                return false;
            }
            return true;
        }

        void applyGeometryMetrics(ArmorGeometry &geometry, const GeometryMetrics &metrics) {
            geometry.angle_diff_deg = metrics.angle_diff_deg;
            geometry.length_diff = metrics.length_diff;
            geometry.x_diff_ratio = metrics.x_diff_ratio;
            geometry.y_diff_ratio = metrics.y_diff_ratio;
            geometry.distance_ratio = metrics.distance_ratio;
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
        record.accepted = true;
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
            record.accepted = false;
            record.detail = "geometry_reject:invalid_left_seed";
            return record;
        }
        if (!hasUsableEndpoints(right_seed)) {
            record.accepted = false;
            record.detail = "geometry_reject:invalid_right_seed";
            return record;
        }

        record.has_raw_lights = true;
        record.left_raw_light = left_seed;
        record.right_raw_light = right_seed;

        LightBar candidate_left = left_seed;
        LightBar candidate_right = right_seed;
        std::array<cv::Point2f, 4> candidate_corners = record.original_corners;

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

        if (lr.corrected && rr.corrected) {
            LightBar lo = lr.output_light;
            LightBar ro = rr.output_light;
            std::array<cv::Point2f, 4> oc = {lo.top, ro.top, ro.bottom, lo.bottom};
            float td = 0.0f;
            for (int i = 0; i < 4; ++i)
                td += cv::norm(oc[i] - record.original_corners[i]);
            if (td <= params_.max_endpoint_distance_px) {
                candidate_left = lo;
                candidate_right = ro;
                candidate_corners = oc;
                record.output_corners = oc;
                record.has_output_lights = true;
                record.left_output_light = lo;
                record.right_output_light = ro;
                record.corrected = true;
                record.detail = "success";
            }
            else {
                record.detail = "raw_used:distance_reject";
            }
        }
        else if (!lr.corrected) {
            record.detail = "raw_used:light_correction_failed:left:" + lr.detail;
        }
        else {
            record.detail = "raw_used:light_correction_failed:right:" + rr.detail;
        }

        GeometryMetrics metrics;
        std::string geometry_detail;
        if (!checkArmorGeometry(
                candidate_left, candidate_right, armor.geometry.type, params_.geometry, &metrics, &geometry_detail)) {
            record.accepted = false;
            record.detail = "geometry_reject:" + geometry_detail;
            return record;
        }

        record.accepted = true;
        record.output_corners = candidate_corners;
        if (record.detail.empty()) {
            record.detail = record.corrected ? "success" : "raw_used";
        }
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
                r.accepted = true;
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
                r.accepted = true;
                r.method = params_.method;
                r.detail = "disabled";
                result.debug.records.push_back(r);
            }
            return result;
        }
        cv::Mat gray_img;
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
        result.armors.clear();
        for (std::size_t i = 0; i < armors.size(); ++i) {
            auto record = correctOne(armors[i], bgr_img, gray_img);
            if (record.accepted) {
                DetectedArmor output = armors[i];
                const LightBar *metrics_left = nullptr;
                const LightBar *metrics_right = nullptr;
                if (record.corrected) {
                    output.geometry.corners = record.output_corners;
                    output.geometry.paired_lights = {record.left_output_light, record.right_output_light};
                    metrics_left = &record.left_output_light;
                    metrics_right = &record.right_output_light;
                }
                else if (record.has_raw_lights) {
                    metrics_left = &record.left_raw_light;
                    metrics_right = &record.right_raw_light;
                }

                if (metrics_left && metrics_right) {
                    GeometryMetrics metrics;
                    std::string detail;
                    if (computeGeometryMetrics(*metrics_left, *metrics_right, &metrics, &detail)) {
                        applyGeometryMetrics(output.geometry, metrics);
                    }
                }
                result.armors.push_back(std::move(output));
            }
            result.debug.records.push_back(record);
        }
        return result;
    }

} // namespace armor_detector
