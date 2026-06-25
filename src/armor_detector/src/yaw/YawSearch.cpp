#include "armor_detector/yaw/YawSearch.hpp"
#include "armor_detector/tools/angle.hpp"

#include <cmath>
#include <limits>

namespace armor_detector::yaw {

    // 搜索参数（后续可参数化）
    constexpr double kSearchRangeRad = tools::degToRad(30.0);
    constexpr double kEnumerateStepRad = tools::degToRad(4.0);
    constexpr double kLocalRangeRad = tools::degToRad(3.0);
    constexpr int kTernaryIterations = 8;

    double runYawSearch(double center_yaw, const YawErrorFunction &calculate_error) {
        using tools::normalizeRadAngle;

        // Phase 1: Coarse enumeration
        const double start_yaw = center_yaw - kSearchRangeRad;
        const int steps = static_cast<int>(2.0 * kSearchRangeRad / kEnumerateStepRad);

        double best_yaw = center_yaw;
        double min_error = std::numeric_limits<double>::infinity();

        for (int i = 0; i <= steps; ++i) {
            const double yaw = normalizeRadAngle(start_yaw + static_cast<double>(i) * kEnumerateStepRad);
            const double error = calculate_error(yaw);
            if (error < min_error) {
                min_error = error;
                best_yaw = yaw;
            }
        }

        // Phase 2: Local ternary search
        double l = best_yaw - kLocalRangeRad;
        double r = best_yaw + kLocalRangeRad;

        for (int iter = 0; iter < kTernaryIterations; ++iter) {
            const double m1 = l + (r - l) / 3.0;
            const double m2 = r - (r - l) / 3.0;
            const double e1 = calculate_error(normalizeRadAngle(m1));
            const double e2 = calculate_error(normalizeRadAngle(m2));

            if (e1 < e2) {
                r = m2;
            }
            else {
                l = m1;
            }
        }

        return normalizeRadAngle((l + r) / 2.0);
    }

} // namespace armor_detector::yaw
