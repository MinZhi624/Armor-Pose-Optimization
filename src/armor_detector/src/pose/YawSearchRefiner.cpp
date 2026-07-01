#include "armor_detector/pose/YawSearchRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/angle.hpp"

#include <limits>

namespace armor_detector::pose {

    namespace {
        constexpr double kSearchRangeRad = tools::degToRad(30.0);
        constexpr double kEnumerateStepRad = tools::degToRad(4.0);
        constexpr double kLocalRangeRad = tools::degToRad(3.0);
        constexpr int kTernaryIterations = 8;

        template <typename ErrorFunction>
        double enumerateYaw(double center_yaw,
                            const ErrorFunction &calculate_error,
                            double search_range_rad,
                            double step_rad) {
            const double start_yaw = center_yaw - search_range_rad;
            const int steps = static_cast<int>(2.0 * search_range_rad / step_rad);

            double best_yaw = center_yaw;
            double min_error = std::numeric_limits<double>::infinity();
            for (int i = 0; i <= steps; ++i) {
                const double yaw = tools::normalizeRadAngle(start_yaw + static_cast<double>(i) * step_rad);
                const double error = calculate_error(yaw);
                if (error < min_error) {
                    min_error = error;
                    best_yaw = yaw;
                }
            }
            return best_yaw;
        }

        template <typename ErrorFunction>
        double
        ternaryYaw(double initial_yaw, const ErrorFunction &calculate_error, double local_range_rad, int iterations) {
            double left = initial_yaw - local_range_rad;
            double right = initial_yaw + local_range_rad;
            for (int iter = 0; iter < iterations; ++iter) {
                const double m1 = left + (right - left) / 3.0;
                const double m2 = right - (right - left) / 3.0;
                const double e1 = calculate_error(tools::normalizeRadAngle(m1));
                const double e2 = calculate_error(tools::normalizeRadAngle(m2));
                if (e1 < e2) {
                    right = m2;
                }
                else {
                    left = m1;
                }
            }
            return tools::normalizeRadAngle((left + right) / 2.0);
        }
    } // namespace

    PoseRefineOutput YawSearchRefiner::refine(const PoseRefineInput &input) const {
        PoseRefineOutput output;
        output.rvec = input.initial_rvec;
        output.tvec = input.initial_tvec;
        output.success = false;

        const ArmorPose initial_pose =
            calculateArmorPose(input.initial_rvec, input.initial_tvec, input.image_corners, input.camera_matrix);
        const Eigen::Vector3d xyz_gimbal = initial_pose.xyz_gimbal;
        const double initial_yaw_rad = initial_pose.ypr_gimbal.x();

        auto yaw_error_func = [&](double yaw_rad) -> double {
            cv::Vec3d rvec;
            cv::Vec3d tvec;
            rvecTvecFromGimbalXyzYaw(xyz_gimbal, yaw_rad, rvec, tvec);
            return calculateReprojectionError(
                input.armor_type, input.image_corners, rvec, tvec, input.camera_matrix, input.distortion_coefficients);
        };

        const double coarse_yaw_rad = enumerateYaw(initial_yaw_rad, yaw_error_func, kSearchRangeRad, kEnumerateStepRad);
        const double refined_yaw_rad = ternaryYaw(coarse_yaw_rad, yaw_error_func, kLocalRangeRad, kTernaryIterations);
        rvecTvecFromGimbalXyzYaw(xyz_gimbal, refined_yaw_rad, output.rvec, output.tvec);
        output.reprojection_error_px = calculateReprojectionError(input.armor_type,
                                                                  input.image_corners,
                                                                  output.rvec,
                                                                  output.tvec,
                                                                  input.camera_matrix,
                                                                  input.distortion_coefficients);
        output.success = true;
        return output;
    }

} // namespace armor_detector::pose
