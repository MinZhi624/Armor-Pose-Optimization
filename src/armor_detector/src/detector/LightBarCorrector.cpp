#include "armor_detector/detector/LightBarCorrector.hpp"
#include "armor_detector/tools/angle.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace armor_detector {

    namespace {
        constexpr float EPS = 1e-6f;
        constexpr float MAX_LOCAL_ANGLE_DIFF_RAD = 0.7853982f; // 45 deg
        constexpr float MIN_PROJECTION_OVERLAP_RATIO = 0.35f;

        cv::Point2f normalized(const cv::Point2f &v) {
            const float len = static_cast<float>(cv::norm(v));
            if (len <= EPS) {
                return cv::Point2f(0.0f, 0.0f);
            }
            return v * (1.0f / len);
        }

        cv::Point2f seedAxisTopToBottom(const LightBar &light) {
            cv::Point2f axis = normalized(light.bottom - light.top);
            if (cv::norm(axis) > EPS) {
                return axis;
            }
            return cv::Point2f(static_cast<float>(std::cos(light.angle)), static_cast<float>(std::sin(light.angle)));
        }

        cv::Point2f seedAxisBottomToTop(const LightBar &light) {
            cv::Point2f axis = normalized(light.top - light.bottom);
            if (cv::norm(axis) > EPS) {
                return axis;
            }
            return cv::Point2f(static_cast<float>(-std::cos(light.angle)), static_cast<float>(-std::sin(light.angle)));
        }

        std::vector<cv::Point> toFullImageContour(const std::vector<cv::Point> &local_contour,
                                                  const cv::Point2f &roi_offset) {
            const cv::Point offset(static_cast<int>(std::round(roi_offset.x)),
                                   static_cast<int>(std::round(roi_offset.y)));
            std::vector<cv::Point> full_contour;
            full_contour.reserve(local_contour.size());
            for (const auto &pt : local_contour) {
                full_contour.push_back(pt + offset);
            }
            return full_contour;
        }

        float angleDiffRad(const cv::Point2f &lhs, const cv::Point2f &rhs) {
            const float lhs_len = static_cast<float>(cv::norm(lhs));
            const float rhs_len = static_cast<float>(cv::norm(rhs));
            if (lhs_len <= EPS || rhs_len <= EPS) {
                return std::numeric_limits<float>::max();
            }
            float dot = (lhs.x * rhs.x + lhs.y * rhs.y) / (lhs_len * rhs_len);
            dot = std::clamp(std::abs(dot), 0.0f, 1.0f);
            return std::acos(dot);
        }

        float projectionOverlapRatio(const std::vector<cv::Point> &contour,
                                     const LightBar &seed_light,
                                     const cv::Point2f &axis) {
            if (contour.empty()) {
                return 0.0f;
            }

            const auto project = [&](const cv::Point2f &pt) { return pt.x * axis.x + pt.y * axis.y; };

            const float seed_a = project(seed_light.top);
            const float seed_b = project(seed_light.bottom);
            const float seed_min = std::min(seed_a, seed_b);
            const float seed_max = std::max(seed_a, seed_b);

            float contour_min = std::numeric_limits<float>::max();
            float contour_max = -std::numeric_limits<float>::max();
            for (const auto &pt : contour) {
                const float value = project(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
                contour_min = std::min(contour_min, value);
                contour_max = std::max(contour_max, value);
            }

            const float overlap = std::min(seed_max, contour_max) - std::max(seed_min, contour_min);
            if (overlap <= 0.0f) {
                return 0.0f;
            }
            const float seed_len = std::max(seed_max - seed_min, EPS);
            const float contour_len = std::max(contour_max - contour_min, EPS);
            return overlap / std::min(seed_len, contour_len);
        }

        float medianValue(std::vector<float> values) {
            if (values.empty()) {
                return 0.0f;
            }

            // 奇数情况下
            const std::size_t mid = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
            const float upper = values[mid];
            if (values.size() % 2 != 0) {
                return upper;
            }

            // 偶数情况下
            std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid - 1), values.end());
            return (values[mid - 1] + upper) * 0.5f;
        }

        struct EndpointProjection {
            float axis_offset = 0.0f;
            float normal_offset = 0.0f;
        };

        struct WeightedPCAResult {
            cv::Point2f centroid;
            cv::Point2f axis;
            std::vector<cv::Point2f> points;
        };

        bool computeWeightedPCAAxis(const cv::Mat &roi, float threshold, WeightedPCAResult &result) {
            result = {};
            result.points.reserve(static_cast<std::size_t>(roi.rows * roi.cols));
            std::vector<float> weights;
            weights.reserve(static_cast<std::size_t>(roi.rows * roi.cols));

            double sum_w = 0.0;
            double sum_x = 0.0;
            double sum_y = 0.0;
            for (int y = 0; y < roi.rows; ++y) {
                for (int x = 0; x < roi.cols; ++x) {
                    const float value = roi.at<float>(y, x);
                    const float weight = std::max(0.0f, value - threshold);
                    if (weight <= EPS) {
                        continue;
                    }

                    result.points.emplace_back(static_cast<float>(x), static_cast<float>(y));
                    weights.push_back(weight);
                    sum_w += weight;
                    sum_x += weight * static_cast<double>(x);
                    sum_y += weight * static_cast<double>(y);
                }
            }

            if (result.points.size() < 6 || sum_w <= EPS) {
                return false;
            }

            result.centroid = cv::Point2f(static_cast<float>(sum_x / sum_w), static_cast<float>(sum_y / sum_w));

            double cxx = 0.0;
            double cxy = 0.0;
            double cyy = 0.0;
            for (std::size_t i = 0; i < result.points.size(); ++i) {
                const cv::Point2f delta = result.points[i] - result.centroid;
                const double weight = static_cast<double>(weights[i]);
                cxx += weight * static_cast<double>(delta.x) * static_cast<double>(delta.x);
                cxy += weight * static_cast<double>(delta.x) * static_cast<double>(delta.y);
                cyy += weight * static_cast<double>(delta.y) * static_cast<double>(delta.y);
            }
            cxx /= sum_w;
            cxy /= sum_w;
            cyy /= sum_w;

            Eigen::Matrix2f covariance;
            covariance << static_cast<float>(cxx), static_cast<float>(cxy), static_cast<float>(cxy),
                static_cast<float>(cyy);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
            if (solver.info() != Eigen::Success) {
                return false;
            }

            const Eigen::Vector2f principal_axis = solver.eigenvectors().col(1);
            result.axis = cv::Point2f(principal_axis.x(), principal_axis.y());
            const float axis_len = static_cast<float>(cv::norm(result.axis));
            if (axis_len <= EPS) {
                return false;
            }
            result.axis /= axis_len;
            return true;
        }

        bool selectBestLocalContour(const LightBarCorrectionInput &input,
                                    std::vector<cv::Point> &selected_contour,
                                    LightBar &selected_light,
                                    std::string &detail) {
            if (input.gray_roi.empty()) {
                detail = "empty_roi";
                return false;
            }

            cv::Mat binary;
            cv::threshold(input.gray_roi, binary, input.gray_threshold, 255, cv::THRESH_BINARY);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (contours.empty()) {
                detail = "no_local_contour";
                return false;
            }

            const cv::Point2f seed_axis = seedAxisTopToBottom(input.seed_light);
            if (cv::norm(seed_axis) <= EPS || input.seed_light.length <= EPS) {
                detail = "invalid_seed_light";
                return false;
            }

            float best_score = std::numeric_limits<float>::max();
            bool found = false;
            for (const auto &local_contour : contours) {
                if (!LightDetector::checkLightGeometry(local_contour, input.geometry)) {
                    continue;
                }

                auto full_contour = toFullImageContour(local_contour, input.roi_offset);
                LightBar light = LightDetector::createLightBar(full_contour);
                light.color = input.seed_light.color;
                light.id = input.seed_light.id;
                if (light.length <= EPS) {
                    continue;
                }

                const float center_distance = static_cast<float>(cv::norm(light.center - input.seed_light.center));
                const float max_center_distance =
                    std::max(static_cast<float>(input.seed_light.length) * 0.5f, input.search_half_width_px * 2.0f);
                if (center_distance > max_center_distance) {
                    continue;
                }

                const cv::Point2f light_axis = seedAxisTopToBottom(light);
                const float angle_diff = angleDiffRad(seed_axis, light_axis);
                if (angle_diff > MAX_LOCAL_ANGLE_DIFF_RAD) {
                    continue;
                }

                const float overlap_ratio = projectionOverlapRatio(full_contour, input.seed_light, seed_axis);
                if (overlap_ratio < MIN_PROJECTION_OVERLAP_RATIO) {
                    continue;
                }

                const float center_score = center_distance / std::max(static_cast<float>(input.seed_light.length), EPS);
                const float score = center_score + angle_diff + (1.0f - overlap_ratio);
                if (score < best_score) {
                    best_score = score;
                    selected_contour = std::move(full_contour);
                    selected_light = light;
                    found = true;
                }
            }

            if (!found) {
                detail = "no_matching_local_contour";
                return false;
            }
            detail = "success";
            return true;
        }
    } // namespace

    LightBarCorrectionResult LightBarCorrector::correct(const LightBarCorrectionInput &input,
                                                        LightBarCorrectionMethod method) const {
        switch (method) {
            case LightBarCorrectionMethod::MIN_AREA_RECT:
                return correctByMinAreaRect(input);
            case LightBarCorrectionMethod::FIT_ELLIPSE:
                return correctByEllipse(input);
            case LightBarCorrectionMethod::PCA_GRADIENT:
                return correctByPCAGradient(input);
            case LightBarCorrectionMethod::NONE: {
                LightBarCorrectionResult result;
                result.raw_light = input.seed_light;
                result.output_light = input.seed_light;
                result.method = "none";
                result.corrected = false;
                result.detail = "none_method";
                return result;
            }
        }
        return correctByEllipse(input);
    }

    LightBarCorrectionResult LightBarCorrector::correctByMinAreaRect(const LightBarCorrectionInput &input) const {
        LightBarCorrectionResult result;
        result.raw_light = input.seed_light;
        result.output_light = input.seed_light;
        result.method = "min_area_rect";

        std::vector<cv::Point> contour;
        LightBar light;
        std::string detail;
        if (!selectBestLocalContour(input, contour, light, detail)) {
            result.detail = detail;
            return result;
        }

        result.output_light = light;
        result.corrected = true;
        result.detail = "success";
        return result;
    }

    LightBarCorrectionResult LightBarCorrector::correctByEllipse(const LightBarCorrectionInput &input) const {
        LightBarCorrectionResult result;
        result.raw_light = input.seed_light;
        result.output_light = input.seed_light;
        result.method = "fit_ellipse";

        std::vector<cv::Point> contour;
        LightBar base_light;
        std::string detail;
        if (!selectBestLocalContour(input, contour, base_light, detail)) {
            result.detail = detail;
            return result;
        }
        if (contour.size() < 5) {
            result.detail = "too_few_points";
            return result;
        }

        cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
        result.output_light = base_light;
        result.output_light.rect = ellipse_rect;
        result.output_light.color = input.seed_light.color;
        result.output_light.id = input.seed_light.id;

        double angle_rad = tools::degToRad(ellipse_rect.angle + 90);
        cv::Point2f dir = cv::Point2f(static_cast<float>(std::cos(angle_rad)), static_cast<float>(std::sin(angle_rad)));

        if (std::abs(dir.y) > 0.8f) {
            if (dir.y > 0) {
                dir = -dir;
            }
        }
        else if (dir.x < 0) {
            dir = -dir;
        }

        double len = cv::norm(dir);
        if (len < EPS) {
            result.detail = "zero_length";
            return result;
        }
        dir = dir / len;

        double half_len = std::max(base_light.width, base_light.length) / 2;
        result.output_light.top = base_light.center + dir * half_len;
        result.output_light.bottom = base_light.center - dir * half_len;
        result.output_light.angle = std::atan2(dir.y, dir.x);
        result.corrected = true;
        result.detail = "success";
        return result;
    }

    LightBarCorrectionResult LightBarCorrector::correctByPCAGradient(const LightBarCorrectionInput &input) const {
        LightBarCorrectionResult result;
        result.raw_light = input.seed_light;
        result.output_light = input.seed_light;
        result.method = "pca_gradient";

        constexpr float MAX_BRIGHTNESS = 25.0f;
        constexpr float PCA_WEIGHT_THRESHOLD = 3.0f;
        // 搜索窗口: 灯条长度 0.4-0.6 区域
        constexpr float SEARCH_START = 0.4f;
        constexpr float SEARCH_END = 0.6f;

        if (input.gray_roi.empty() || input.seed_light.length <= EPS) {
            result.detail = "empty_input";
            return result;
        }

        result.debug_gray_roi = input.gray_roi.clone();

        // ---- 1. 归一化 ROI ----
        const float mean_val = static_cast<float>(cv::mean(input.gray_roi)[0]);
        cv::Mat roi;
        input.gray_roi.convertTo(roi, CV_32F);
        cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

        // ---- 2. 亮度加权 PCA 主轴估计 ----
        WeightedPCAResult weighted_pca;
        if (!computeWeightedPCAAxis(roi, PCA_WEIGHT_THRESHOLD, weighted_pca)) {
            result.detail = weighted_pca.points.size() < 6 ? "too_few_weighted_pca_points" : "weighted_pca_failed";
            return result;
        }
        const cv::Point2f centroid = weighted_pca.centroid;
        const std::vector<cv::Point2f> &points = weighted_pca.points;
        cv::Point2f axis = weighted_pca.axis;

        // 确保主轴方向与 seed light 一致（下 → 上）
        const cv::Point2f seed_top_axis = seedAxisBottomToTop(input.seed_light);
        if (axis.x * seed_top_axis.x + axis.y * seed_top_axis.y < 0.0f) {
            axis = -axis;
        }
        const cv::Point2f normal(-axis.y, axis.x);

        // ---- 4. 沿主轴梯度搜索端点 ----
        const auto inside = [&](const cv::Point2f &pt) {
            return pt.x >= 0 && pt.x < input.gray_roi.cols && pt.y >= 0 && pt.y < input.gray_roi.rows;
        };

        // 找到点对应的亮度值
        const auto sample_u8 = [&](const cv::Point2f &pt) -> uchar {
            const int x = std::clamp(static_cast<int>(std::round(pt.x)), 0, input.gray_roi.cols - 1);
            const int y = std::clamp(static_cast<int>(std::round(pt.y)), 0, input.gray_roi.rows - 1);
            return input.gray_roi.at<uchar>(y, x);
        };

        const auto find_endpoint_projection = [&](float direction, EndpointProjection &endpoint) -> bool {
            const cv::Point2f search_axis = axis * direction;
            const float search_length = static_cast<float>(input.seed_light.length) * (SEARCH_END - SEARCH_START);
            const int half_width = std::max(0, static_cast<int>(input.search_half_width_px));

            // 4a. 多条平行扫描线，每条找到该线上最大梯度位置。
            // 候选点只贡献轴向投影，避免横向噪声重新改写 PCA 方向。
            std::vector<float> axis_offsets;
            std::vector<float> normal_offsets;
            axis_offsets.reserve(static_cast<std::size_t>(half_width * 2 + 1));
            normal_offsets.reserve(static_cast<std::size_t>(half_width * 2 + 1));
            for (int offset = -half_width; offset <= half_width; ++offset) {
                const cv::Point2f start = centroid +
                    search_axis * (static_cast<float>(input.seed_light.length) * SEARCH_START) +
                    normal * static_cast<float>(offset);

                float max_diff = 0.0f;
                cv::Point2f best;
                bool found = false;
                for (float step = 0.0f; step < search_length; step += 1.0f) {
                    const cv::Point2f cur = start + search_axis * step;
                    const cv::Point2f prev = cur - search_axis;
                    if (!inside(cur) || !inside(prev)) {
                        break;
                    }

                    const float prev_val = static_cast<float>(sample_u8(prev));
                    const float cur_val = static_cast<float>(sample_u8(cur));
                    const float diff = prev_val - cur_val;
                    if (diff > max_diff && prev_val > mean_val) {
                        max_diff = diff;
                        best = prev;
                        found = true;
                    }
                }
                if (found) {
                    const cv::Point2f delta = best - centroid;
                    axis_offsets.push_back(delta.dot(axis));
                    normal_offsets.push_back(delta.dot(normal));
                }
            }

            // 4b. 用中位数抑制背景边缘/反光造成的离群扫描线。
            if (axis_offsets.empty()) {
                return false;
            }
            endpoint.axis_offset = medianValue(std::move(axis_offsets));
            endpoint.normal_offset = medianValue(std::move(normal_offsets));
            return true;
        };

        EndpointProjection top_endpoint;
        EndpointProjection bottom_endpoint;
        if (!find_endpoint_projection(1.0f, top_endpoint) || !find_endpoint_projection(-1.0f, bottom_endpoint)) {
            result.detail = "endpoints_not_found";
            return result;
        }

        const float shared_normal_offset = (top_endpoint.normal_offset + bottom_endpoint.normal_offset) * 0.5f;
        const cv::Point2f top_local = centroid + axis * top_endpoint.axis_offset + normal * shared_normal_offset;
        const cv::Point2f bottom_local = centroid + axis * bottom_endpoint.axis_offset + normal * shared_normal_offset;
        if (!inside(top_local) || !inside(bottom_local)) {
            result.detail = "projected_endpoints_outside";
            return result;
        }

        // ---- 5. 局部坐标 → 全图坐标，生成输出 LightBar ----
        cv::Point2f top = top_local + input.roi_offset;
        cv::Point2f bottom = bottom_local + input.roi_offset;
        if (top.y > bottom.y) {
            std::swap(top, bottom);
        }

        result.output_light.top = top;
        result.output_light.bottom = bottom;
        result.output_light.center = (top + bottom) * 0.5f;
        result.output_light.length = cv::norm(top - bottom);
        result.output_light.angle = std::atan2((bottom - top).y, (bottom - top).x);
        result.output_light.color = input.seed_light.color;
        result.output_light.id = input.seed_light.id;

        // ---- 6. 渲染 PCA debug 可视化 ----
        {
            cv::Mat viz_bgr;
            cv::cvtColor(input.gray_roi, viz_bgr, cv::COLOR_GRAY2BGR);
            for (const auto &pt : points)
                cv::circle(viz_bgr, pt, 1, cv::Scalar(0, 255, 0), -1);
            float axis_draw_len = input.seed_light.length * 0.5f;
            cv::Point2f axis_start = centroid - axis * axis_draw_len;
            cv::Point2f axis_end = centroid + axis * axis_draw_len;
            cv::line(viz_bgr, axis_start, axis_end, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            cv::line(viz_bgr, top_local, bottom_local, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
            cv::circle(viz_bgr, top_local, 2, cv::Scalar(0, 255, 255), -1);
            cv::circle(viz_bgr, bottom_local, 2, cv::Scalar(0, 255, 255), -1);
            result.debug_pca_viz = viz_bgr;
        }

        result.corrected = true;
        result.detail = "success";
        return result;
    }

} // namespace armor_detector
