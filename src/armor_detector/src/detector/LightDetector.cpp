#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/tools/angle.hpp"

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
            if (!checkLightGeometry(contour)) {
                // 记录被拒绝的灯条
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
            cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
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

            LightBar light = createLight(ellipse_rect, min_rect, color);
            light.id = static_cast<int>(lights.size());
            lights.push_back(light);
            light_debug_.accepted_lights.push_back(light);
        }

        // 按 x 坐标排序
        std::sort(
            lights.begin(), lights.end(), [](const LightBar &a, const LightBar &b) { return a.center.x < b.center.x; });

        return lights;
    }

    bool LightDetector::checkLightGeometry(const std::vector<cv::Point> &contour) const {
        if (contour.size() <= 6) {
            return false;
        }
        int area = cv::contourArea(contour);
        if (area <= params_.min_contours_area) {
            return false;
        }

        cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
        double long_length = std::max(min_rect.size.width, min_rect.size.height);
        double short_length = std::min(min_rect.size.width, min_rect.size.height);
        double ratio = short_length / long_length;
        return ratio > params_.min_contours_ratio && ratio < params_.max_contours_ratio;
    }

    LightBarColor LightDetector::findLightColor(const cv::Mat &img_bgr,
                                                const cv::RotatedRect &rect,
                                                const std::vector<cv::Point> &contour) const {
        cv::Rect bbox = rect.boundingRect();
        bbox &= cv::Rect(0, 0, img_bgr.cols, img_bgr.rows);
        if (bbox.empty()) {
            return LightBarColor::NONE;
        }

        // 生成轮廓 mask
        cv::Mat mask = cv::Mat::zeros(bbox.size(), CV_8UC1);
        std::vector<cv::Point> local_contour;
        local_contour.reserve(contour.size());
        for (const auto &pt : contour) {
            local_contour.push_back(pt - cv::Point(bbox.x, bbox.y));
        }
        cv::drawContours(mask, std::vector<std::vector<cv::Point>>{local_contour}, -1, 255, -1);
        cv::erode(mask, mask, cv::Mat());

        // 提取 ROI 并分离 B/R 通道
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

    LightBar LightDetector::createLight(const cv::RotatedRect &ellipse_rect,
                                        const cv::RotatedRect &min_rect,
                                        LightBarColor color) const {
        LightBar light;
        light.rect = ellipse_rect;
        light.color = color;

        // 用椭圆角度计算方向，用 minAreaRect 尺寸约束端点
        light.center = min_rect.center;
        double angle_rad = tools::degToRad(ellipse_rect.angle + 90);
        cv::Point2f dir = cv::Point2f(std::cos(angle_rad), std::sin(angle_rad));

        // 确保方向一致性
        if (std::abs(dir.y) > 0.8f) {
            if (dir.y > 0) {
                dir = -dir;
            }
        }
        else if (dir.x < 0) {
            dir = -dir;
        }

        double len = cv::norm(dir);
        if (len < 1e-6) {
            return light;
        }
        dir = dir / len;

        double half_len = std::max(min_rect.size.width, min_rect.size.height) / 2;
        light.top = light.center + dir * half_len;
        light.bottom = light.center - dir * half_len;
        light.angle = std::atan2(dir.y, dir.x);
        light.length = half_len * 2.0;
        light.width = std::min(min_rect.size.width, min_rect.size.height);
        light.area = static_cast<int>(min_rect.size.width * min_rect.size.height);

        return light;
    }

} // namespace armor_detector
