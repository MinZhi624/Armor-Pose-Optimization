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

    double enumerate(double center_yaw, const YawErrorFunction &calculate_error,
                     double search_range_rad, double step_rad) {
        using tools::normalizeRadAngle;

        const double start_yaw = center_yaw - search_range_rad;
        const int steps = static_cast<int>(2.0 * search_range_rad / step_rad);

        double best_yaw = center_yaw;
        double min_error = std::numeric_limits<double>::infinity();

        for (int i = 0; i <= steps; ++i) {
            const double yaw = normalizeRadAngle(start_yaw + static_cast<double>(i) * step_rad);
            const double error = calculate_error(yaw);
            if (error < min_error) {
                min_error = error;
                best_yaw = yaw;
            }
        }

        return best_yaw;
    }

    double ternary(double initial_yaw, const YawErrorFunction &calculate_error,
                   double local_range_rad, int iterations) {
        using tools::normalizeRadAngle;

        double l = initial_yaw - local_range_rad;
        double r = initial_yaw + local_range_rad;

        for (int iter = 0; iter < iterations; ++iter) {
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

    double runYawSearch(double center_yaw, const YawErrorFunction &calculate_error) {
        const double coarse_yaw = enumerate(center_yaw, calculate_error,
                                            kSearchRangeRad, kEnumerateStepRad);
        return ternary(coarse_yaw, calculate_error,
                       kLocalRangeRad, kTernaryIterations);
    }

} // namespace armor_detector::yaw
