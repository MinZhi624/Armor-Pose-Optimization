#include "armor_detector/detector/LightDetector.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace armor_detector {

    LightDetector::LightDetector(const Params &params) : params_(params) {
    }

    std::vector<LightBar> LightDetector::findLights(const cv::Mat &img_thre, const cv::Mat &img_bgr) {
        light_debug_ = {};

        std::vector<std::vector<cv::Point>> contours;
        std::vector<LightBar> lights;
        cv::findContours(img_thre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        for (const auto &contour : contours) {
            if (!checkLightGeometry(contour, params_)) {
                cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
                debug::RejectedLight rejected;
                rejected.light.rect = min_rect;
                rejected.light.center = min_rect.center;
                rejected.reason = debug::DebugRejectReason::BAD_RATIO;
                rejected.detail = "geometry";
                light_debug_.rejected_lights.push_back(std::move(rejected));
                continue;
            }

            cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
            LightBarColor color = findLightColor(img_bgr, min_rect, contour);

            if (color == LightBarColor::NONE) {
                debug::RejectedLight rejected;
                rejected.light.rect = min_rect;
                rejected.light.center = min_rect.center;
                rejected.reason = debug::DebugRejectReason::BAD_COLOR;
                rejected.detail = "none";
                light_debug_.rejected_lights.push_back(std::move(rejected));
                continue;
            }

            LightBar light = createLightBar(contour);
            light.color = color;
            light.id = static_cast<int>(lights.size());
            lights.push_back(light);
            light_debug_.accepted_lights.push_back(light);
        }

        std::sort(
            lights.begin(), lights.end(), [](const LightBar &a, const LightBar &b) { return a.center.x < b.center.x; });

        return lights;
    }

    bool LightDetector::checkLightGeometry(const std::vector<cv::Point> &contour,
                                           const LightGeometryParams &params) {
        if (contour.size() <= 6) {
            return false;
        }
        int area = cv::contourArea(contour);
        if (area <= params.min_contours_area) {
            return false;
        }

        cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
        double long_length = std::max(min_rect.size.width, min_rect.size.height);
        double short_length = std::min(min_rect.size.width, min_rect.size.height);
        double ratio = short_length / long_length;
        return ratio > params.min_contours_ratio && ratio < params.max_contours_ratio;
    }

    LightBar LightDetector::createLightBar(const std::vector<cv::Point> &contour) {
        cv::RotatedRect min_rect = cv::minAreaRect(contour);
        LightBar light;
        light.rect = min_rect;
        light.center = min_rect.center;
        light.length = std::max(min_rect.size.width, min_rect.size.height);
        light.width = std::min(min_rect.size.width, min_rect.size.height);
        light.area = static_cast<int>(min_rect.size.width * min_rect.size.height);

        cv::Point2f vertices[4];
        min_rect.points(vertices);

        double max_edge_len = 0;
        cv::Point2f p1, p2;
        for (int i = 0; i < 4; ++i) {
            int next = (i + 1) % 4;
            double edge_len = cv::norm(vertices[next] - vertices[i]);
            if (edge_len > max_edge_len) {
                max_edge_len = edge_len;
                p1 = vertices[i];
                p2 = vertices[next];
            }
        }

        cv::Point2f dir = p2 - p1;
        if (p1.y > p2.y) {
            dir = -dir;
        }

        double len = cv::norm(dir);
        if (len < 1e-6) {
            return light;
        }
        dir = dir / len;

        double half_len = light.length / 2;
        light.top = light.center - dir * half_len;
        light.bottom = light.center + dir * half_len;
        light.angle = std::atan2(dir.y, dir.x);
        return light;
    }

    LightBarColor LightDetector::findLightColor(const cv::Mat &img_bgr,
                                                const cv::RotatedRect &rect,
                                                const std::vector<cv::Point> &contour) {
        cv::Rect bbox = rect.boundingRect();
        bbox &= cv::Rect(0, 0, img_bgr.cols, img_bgr.rows);
        if (bbox.empty()) {
            return LightBarColor::NONE;
        }

        cv::Mat mask = cv::Mat::zeros(bbox.size(), CV_8UC1);
        std::vector<cv::Point> local_contour;
        local_contour.reserve(contour.size());
        for (const auto &pt : contour) {
            local_contour.push_back(pt - cv::Point(bbox.x, bbox.y));
        }
        cv::drawContours(mask, std::vector<std::vector<cv::Point>>{local_contour}, -1, 255, -1);
        cv::erode(mask, mask, cv::Mat());

        cv::Mat roi = img_bgr(bbox);
        std::vector<cv::Mat> bgr;
        cv::split(roi, bgr);

        double mean_b = cv::mean(bgr[0], mask)[0];
        double mean_r = cv::mean(bgr[2], mask)[0];

        bool is_red = (mean_r / (mean_b + 1.0)) > 1.1;
        bool is_blue = (mean_b / (mean_r + 1.0)) > 1.1;
        if (is_red) {
            return LightBarColor::RED;
        }
        if (is_blue) {
            return LightBarColor::BLUE;
        }
        return LightBarColor::NONE;
    }

} // namespace armor_detector
