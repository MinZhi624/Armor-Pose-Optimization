#include "armor_detector/pose/PoseOnlyBa6DofRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

namespace armor_detector::pose {

    namespace {
        constexpr double kAcceptEpsilonPx = 1e-6;
        constexpr double kHuberLossScalePx = 3.0;

        struct ReprojectionErrorNorm {
            ReprojectionErrorNorm(const cv::Point3d &point_3d, const cv::Point2d &observed_norm, double fx, double fy) :
                point_3d_(point_3d), observed_norm_(observed_norm), fx_(fx), fy_(fy) {
            }

            template <typename T>
            bool operator()(const T *const pose, T *residual) const {
                // pose[0..2]: angle-axis rvec, pose[3..5]: tvec in camera frame.
                const T p_armor[3] = {T(point_3d_.x), T(point_3d_.y), T(point_3d_.z)};
                T p_camera[3];
                ceres::AngleAxisRotatePoint(pose, p_armor, p_camera);
                p_camera[0] += pose[3];
                p_camera[1] += pose[4];
                p_camera[2] += pose[5];

                const T x_pred = p_camera[0] / p_camera[2];
                const T y_pred = p_camera[1] / p_camera[2];

                residual[0] = T(fx_) * (x_pred - T(observed_norm_.x));
                residual[1] = T(fy_) * (y_pred - T(observed_norm_.y));
                return true;
            }

            cv::Point3d point_3d_;
            cv::Point2d observed_norm_;
            double fx_ = 0.0;
            double fy_ = 0.0;
        };

        bool isFinitePose6D(const double *pose) {
            for (int i = 0; i < 6; ++i) {
                if (!std::isfinite(pose[i])) {
                    return false;
                }
            }
            return true;
        }

        bool isUsableOutputPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
                    return false;
                }
            }
            return tvec[2] > 0.0;
        }
    } // namespace

    PoseRefineOutput PoseOnlyBa6DofRefiner::refine(const PoseRefineInput &input) const {
        PoseRefineOutput output;
        output.rvec = input.initial_rvec;
        output.tvec = input.initial_tvec;
        output.success = false;
        output.reprojection_error_px = calculateReprojectionError(input.armor_type,
                                                                  input.image_corners,
                                                                  output.rvec,
                                                                  output.tvec,
                                                                  input.camera_matrix,
                                                                  input.distortion_coefficients);

        if (input.armor_type == ArmorType::NONE || !std::isfinite(output.reprojection_error_px) ||
            !isUsableOutputPose(input.initial_rvec, input.initial_tvec)) {
            return output;
        }

        const auto &object_points_f = objectPointsForArmor(input.armor_type);
        std::vector<cv::Point3d> object_points(object_points_f.begin(), object_points_f.end());

        const std::vector<cv::Point2d> image_corners(input.image_corners.begin(), input.image_corners.end());
        std::vector<cv::Point2d> norm_points;
        cv::undistortPoints(
            image_corners, norm_points, cv::Mat(input.camera_matrix), cv::Mat(input.distortion_coefficients));

        if (object_points.empty() || norm_points.size() != object_points.size()) {
            return output;
        }

        double pose[6] = {input.initial_rvec[0],
                          input.initial_rvec[1],
                          input.initial_rvec[2],
                          input.initial_tvec[0],
                          input.initial_tvec[1],
                          input.initial_tvec[2]};
        if (!isFinitePose6D(pose)) {
            return output;
        }

        ceres::Problem problem;
        const double fx = input.camera_matrix(0, 0);
        const double fy = input.camera_matrix(1, 1);
        for (std::size_t i = 0; i < object_points.size(); ++i) {
            auto *cost_function = new ceres::AutoDiffCostFunction<ReprojectionErrorNorm, 2, 6>(
                new ReprojectionErrorNorm(object_points[i], norm_points[i], fx, fy));
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(kHuberLossScalePx), pose);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 20;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = 1;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (!summary.IsSolutionUsable() || !isFinitePose6D(pose)) {
            return output;
        }

        const cv::Vec3d refined_rvec(pose[0], pose[1], pose[2]);
        const cv::Vec3d refined_tvec(pose[3], pose[4], pose[5]);

        const double refined_error_px = calculateReprojectionError(input.armor_type,
                                                                   input.image_corners,
                                                                   refined_rvec,
                                                                   refined_tvec,
                                                                   input.camera_matrix,
                                                                   input.distortion_coefficients);
        if (isUsableOutputPose(refined_rvec, refined_tvec) && std::isfinite(refined_error_px) &&
            refined_error_px <= output.reprojection_error_px + kAcceptEpsilonPx) {
            output.rvec = refined_rvec;
            output.tvec = refined_tvec;
            output.reprojection_error_px = refined_error_px;
            output.success = true;
        }

        return output;
    }

} // namespace armor_detector::pose
