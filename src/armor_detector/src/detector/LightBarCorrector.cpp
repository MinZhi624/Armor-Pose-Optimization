#include "armor_detector/detector/LightBarCorrector.hpp"
#include "armor_detector/tools/angle.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace armor_detector {

    void LightBarCorrector::lightbarPointsCorrect(LightBar &light, const std::vector<cv::Point> &contour) {
        correctByEllipse(light, contour);
    }

    void LightBarCorrector::correctByEllipse(LightBar &light, const std::vector<cv::Point> &contour) {
        cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
        light.rect = ellipse_rect;

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
            return;
        }
        dir = dir / len;

        double half_len = std::max(light.width, light.length) / 2;
        light.top = light.center + dir * half_len;
        light.bottom = light.center - dir * half_len;
        light.angle = std::atan2(dir.y, dir.x);
    }

} // namespace armor_detector
