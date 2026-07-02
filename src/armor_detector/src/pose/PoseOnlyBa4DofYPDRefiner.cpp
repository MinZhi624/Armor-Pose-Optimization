#include "armor_detector/pose/PoseOnlyBa4DofYPDRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/armor_geometry.hpp"

#include <Eigen/Core>
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

namespace armor_detector::pose {

    namespace {
        constexpr double kHuberLossScalePx = 3.0;

        struct ReprojectionErrorNorm {
            ReprojectionErrorNorm(const cv::Point3d &point_3d, const cv::Point2d &observed_norm, double fx, double fy) :
                point_3d_(point_3d), observed_norm_(observed_norm), fx_(fx), fy_(fy) {
            }

            template <typename T>
            bool operator()(const T *const pose, T *residual) const {

                const Eigen::Matrix<T, 3, 1> p_armor(T(point_3d_.x), T(point_3d_.y), T(point_3d_.z));
                // 旋转
                const T yaw = pose[3];
                const T cy = ceres::cos(yaw);
                const T sy = ceres::sin(yaw);

                const T pitch = T(tools::ARMOR_PITCH_RAD);
                const T cp = ceres::cos(pitch);
                const T sp = ceres::sin(pitch);


                Eigen::Matrix<T, 3, 3> Rz;
                Rz << cy, -sy, T(0), sy, cy, T(0), T(0), T(0), T(1);

                Eigen::Matrix<T, 3, 3> Ry;
                Ry << cp, T(0), sp, T(0), T(1), T(0), -sp, T(0), cp;

                const Eigen::Matrix<T, 3, 1> ypd_gimbal(pose[0], pose[1], pose[2]);
                // ypd球面坐标系 -> xyz直角坐标系坐标系
                const T dir_yaw = pose[0];
                const T dir_pitch = pose[1];
                const T dir_distance = pose[2];

                const T cy_dir = ceres::cos(dir_yaw);
                const T sy_dir = ceres::sin(dir_yaw);
                const T cp_dir = ceres::cos(dir_pitch);
                const T sp_dir = ceres::sin(dir_pitch);

                const Eigen::Matrix<T, 3, 1> xyz_gimbal(
                    dir_distance * cp_dir * cy_dir, dir_distance * cp_dir * sy_dir, dir_distance * sp_dir);

                const Eigen::Matrix<T, 3, 1> p_gimbal = Rz * Ry * p_armor + xyz_gimbal;

                // xyz_gimbal -> xyz_camera
                const Eigen::Matrix<T, 3, 1> p_camera(-p_gimbal.y(), -p_gimbal.z(), p_gimbal.x());

                // 归一化
                const T x_pred = p_camera.x() / p_camera.z();
                const T y_pred = p_camera.y() / p_camera.z();

                residual[0] = T(fx_) * (x_pred - T(observed_norm_.x));
                residual[1] = T(fy_) * (y_pred - T(observed_norm_.y));
                return true;
            }

            cv::Point3d point_3d_;
            cv::Point2d observed_norm_;
            double fx_ = 0.0;
            double fy_ = 0.0;
        };

        bool isFinitePose4D(const double *pose) {
            for (int i = 0; i < 4; ++i) {
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

    PoseRefineOutput PoseOnlyBa4DofYPDRefiner::refine(const PoseRefineInput &input) const {
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

        const ArmorPose initial_pose =
            calculateArmorPose(input.initial_rvec, input.initial_tvec, input.image_corners, input.camera_matrix);
        // Pose 优化变量: dir_yaw_gimbal, dir_pitch_gimbal, dir_distance_gimbal, pose_yaw.
        double pose[4] = {initial_pose.ypd_gimbal.x(), // dir_yaw_gimbal
                          initial_pose.ypd_gimbal.y(), // dir_pitch_gimbal
                          initial_pose.ypd_gimbal.z(), // dir_distance_gimbal
                          initial_pose.ypr_gimbal.x()}; // pose_yaw
        if (!isFinitePose4D(pose)) {
            return output;
        }

        ceres::Problem problem;
        const double fx = input.camera_matrix(0, 0);
        const double fy = input.camera_matrix(1, 1);
        for (std::size_t i = 0; i < object_points.size(); ++i) {
            auto *cost_function = new ceres::AutoDiffCostFunction<ReprojectionErrorNorm, 2, 4>(
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

        if (!summary.IsSolutionUsable() || !isFinitePose4D(pose)) {
            return output;
        }

        cv::Vec3d refined_rvec;
        cv::Vec3d refined_tvec;
        rvecTvecFromGimbalYpdYaw(Eigen::Vector3d(pose[0], pose[1], pose[2]), pose[3], refined_rvec, refined_tvec);

        const double refined_error_px = calculateReprojectionError(input.armor_type,
                                                                   input.image_corners,
                                                                   refined_rvec,
                                                                   refined_tvec,
                                                                   input.camera_matrix,
                                                                   input.distortion_coefficients);
        if (isUsableOutputPose(refined_rvec, refined_tvec) && std::isfinite(refined_error_px)) {
            output.rvec = refined_rvec;
            output.tvec = refined_tvec;
            output.reprojection_error_px = refined_error_px;
            output.success = true;
        }

        return output;
    }

} // namespace armor_detector::pose
