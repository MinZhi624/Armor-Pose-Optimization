#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/loss_function.h>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <vector>

#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/armor_geometry.hpp"
#include "armor_detector/tools/geometry.hpp"
#include "armor_detector/tools/transform.hpp"

namespace armor_detector::pose {

    namespace {
        bool isFinite(const cv::Point2f &point) {
            return std::isfinite(point.x) && std::isfinite(point.y);
        }

        bool isFinite(const cv::Point2d &point) {
            return std::isfinite(point.x) && std::isfinite(point.y);
        }

        bool isFinite(const Eigen::Vector3d &value) {
            return value.allFinite();
        }

        template <typename T>
        Eigen::Matrix<T, 3, 1> xyzFromYpd(const T *const pose) {
            const T dir_yaw = pose[0];
            const T dir_pitch = pose[1];
            const T distance = pose[2];
            const T cos_pitch = ceres::cos(dir_pitch);
            return {distance * cos_pitch * ceres::cos(dir_yaw),
                    distance * cos_pitch * ceres::sin(dir_yaw),
                    distance * ceres::sin(dir_pitch)};
        }

        template <typename T>
        Eigen::Matrix<T, 3, 1> projectArmorPointToCamera(const cv::Point3d &point_3d,
                                                         const Eigen::Matrix<T, 3, 1> &xyz_gimbal,
                                                         const T &pose_yaw_rad) {
            const T cy = ceres::cos(pose_yaw_rad);
            const T sy = ceres::sin(pose_yaw_rad);
            const T pitch = T(tools::ARMOR_PITCH_RAD);
            const T cp = ceres::cos(pitch);
            const T sp = ceres::sin(pitch);

            Eigen::Matrix<T, 3, 3> Rz;
            Rz << cy, -sy, T(0), sy, cy, T(0), T(0), T(0), T(1);

            Eigen::Matrix<T, 3, 3> Ry;
            Ry << cp, T(0), sp, T(0), T(1), T(0), -sp, T(0), cp;

            const Eigen::Matrix<T, 3, 1> p_armor(T(point_3d.x), T(point_3d.y), T(point_3d.z));
            const Eigen::Matrix<T, 3, 1> p_gimbal = Rz * Ry * p_armor + xyz_gimbal;
            return tools::R_CAMERA_GIMBAL.cast<T>() * p_gimbal;
        }

        struct Pose4DofReprojectionResidual {
            Pose4DofReprojectionResidual(const cv::Point3d &point_3d,
                                         const cv::Point2d &observed_normalized,
                                         double fx,
                                         double fy,
                                         Pose4DofParameterization parameterization) :
                point_3d_(point_3d), observed_normalized_(observed_normalized), fx_(fx), fy_(fy),
                parameterization_(parameterization) {
            }

            template <typename T>
            bool operator()(const T *const pose, T *residual) const {
                const Eigen::Matrix<T, 3, 1> xyz_gimbal = (parameterization_ == Pose4DofParameterization::YPD)
                    ? xyzFromYpd(pose)
                    : Eigen::Matrix<T, 3, 1>(pose[0], pose[1], pose[2]);
                const Eigen::Matrix<T, 3, 1> p_camera = projectArmorPointToCamera(point_3d_, xyz_gimbal, pose[3]);

                const T x_pred = p_camera.x() / p_camera.z();
                const T y_pred = p_camera.y() / p_camera.z();
                residual[0] = T(fx_) * (x_pred - T(observed_normalized_.x));
                residual[1] = T(fy_) * (y_pred - T(observed_normalized_.y));
                return true;
            }

            cv::Point3d point_3d_;
            cv::Point2d observed_normalized_;
            double fx_ = 0.0;
            double fy_ = 0.0;
            Pose4DofParameterization parameterization_ = Pose4DofParameterization::XYZ;
        };

        Pose4DofCostEvaluation invalidEvaluation(const std::string &status) {
            Pose4DofCostEvaluation evaluation;
            evaluation.status = status;
            return evaluation;
        }
    } // namespace

    Pose4DofObservation createPose4DofObservation(const PoseRefineInput &input) {
        Pose4DofObservation observation;
        observation.armor_type = input.armor_type;
        if (input.armor_type == ArmorType::NONE) {
            observation.status = "armor_type_none";
            return observation;
        }

        for (const auto &corner : input.image_corners) {
            if (!isFinite(corner)) {
                observation.status = "non_finite_image_corner";
                return observation;
            }
        }
        for (double value : input.camera_matrix.val) {
            if (!std::isfinite(value)) {
                observation.status = "non_finite_camera_matrix";
                return observation;
            }
        }
        for (int i = 0; i < input.distortion_coefficients.rows; ++i) {
            if (!std::isfinite(input.distortion_coefficients[i])) {
                observation.status = "non_finite_distortion_coefficients";
                return observation;
            }
        }

        observation.fx = input.camera_matrix(0, 0);
        observation.fy = input.camera_matrix(1, 1);
        if (std::abs(observation.fx) <= std::numeric_limits<double>::epsilon() ||
            std::abs(observation.fy) <= std::numeric_limits<double>::epsilon()) {
            observation.status = "invalid_focal_length";
            return observation;
        }

        const auto &object_points = objectPointsForArmor(input.armor_type);
        if (object_points.size() != observation.object_points.size()) {
            observation.status = "unexpected_object_point_count";
            return observation;
        }
        for (std::size_t i = 0; i < object_points.size(); ++i) {
            observation.object_points[i] = object_points[i];
        }

        const std::vector<cv::Point2d> image_corners(input.image_corners.begin(), input.image_corners.end());
        std::vector<cv::Point2d> normalized;
        try {
            cv::undistortPoints(
                image_corners, normalized, cv::Mat(input.camera_matrix), cv::Mat(input.distortion_coefficients));
        }
        catch (const cv::Exception &) {
            observation.status = "undistort_failed";
            return observation;
        }

        if (normalized.size() != observation.observed_normalized.size()) {
            observation.status = "unexpected_normalized_point_count";
            return observation;
        }
        for (std::size_t i = 0; i < normalized.size(); ++i) {
            if (!isFinite(normalized[i])) {
                observation.status = "non_finite_normalized_corner";
                return observation;
            }
            observation.observed_normalized[i] = normalized[i];
        }

        observation.valid = true;
        observation.status = "ok";
        return observation;
    }

    Pose4DofCostEvaluation evaluatePose4DofCost(const Pose4DofObservation &observation,
                                                const Eigen::Vector3d &xyz_gimbal,
                                                double pose_yaw_rad) {
        if (!observation.valid) {
            return invalidEvaluation(observation.status);
        }
        if (!isFinite(xyz_gimbal) || !std::isfinite(pose_yaw_rad)) {
            return invalidEvaluation("non_finite_pose");
        }

        Pose4DofCostEvaluation evaluation;
        ceres::HuberLoss huber_loss(kPose4DofHuberLossScalePx);
        double total_cost = 0.0;
        double total_residual = 0.0;
        for (std::size_t i = 0; i < observation.object_points.size(); ++i) {
            const Eigen::Vector3d p_camera =
                projectArmorPointToCamera<double>(observation.object_points[i], xyz_gimbal, pose_yaw_rad);
            if (!p_camera.allFinite() || p_camera.z() <= 0.0) {
                return invalidEvaluation("invalid_projection_depth");
            }

            const double x_pred = p_camera.x() / p_camera.z();
            const double y_pred = p_camera.y() / p_camera.z();
            const double residual_u = observation.fx * (x_pred - observation.observed_normalized[i].x);
            const double residual_v = observation.fy * (y_pred - observation.observed_normalized[i].y);
            const double squared_residual = residual_u * residual_u + residual_v * residual_v;
            if (!std::isfinite(squared_residual)) {
                return invalidEvaluation("non_finite_residual");
            }

            double rho[3] = {};
            huber_loss.Evaluate(squared_residual, rho);
            if (!std::isfinite(rho[0])) {
                return invalidEvaluation("non_finite_robust_loss");
            }
            total_cost += 0.5 * rho[0];
            evaluation.corner_residual_px[i] = std::sqrt(squared_residual);
            total_residual += evaluation.corner_residual_px[i];
        }

        evaluation.valid = std::isfinite(total_cost) && std::isfinite(total_residual);
        evaluation.status = evaluation.valid ? "ok" : "non_finite_total";
        if (evaluation.valid) {
            evaluation.cost = total_cost;
            evaluation.mean_residual_px = total_residual / static_cast<double>(observation.object_points.size());
        }
        return evaluation;
    }

    Pose4DofCostEvaluation evaluatePose4DofYpdCost(const Pose4DofObservation &observation,
                                                   const Eigen::Vector3d &ypd_gimbal,
                                                   double pose_yaw_rad) {
        if (!isFinite(ypd_gimbal) || ypd_gimbal.z() <= 0.0) {
            return invalidEvaluation("invalid_ypd_distance");
        }
        return evaluatePose4DofCost(observation, tools::calculateXYZ(ypd_gimbal), pose_yaw_rad);
    }

    ceres::CostFunction *createPose4DofReprojectionCostFunction(const Pose4DofObservation &observation,
                                                                std::size_t corner_index,
                                                                Pose4DofParameterization parameterization) {
        if (!observation.valid || corner_index >= observation.object_points.size()) {
            return nullptr;
        }
        return new ceres::AutoDiffCostFunction<Pose4DofReprojectionResidual, 2, 4>(
            new Pose4DofReprojectionResidual(observation.object_points[corner_index],
                                             observation.observed_normalized[corner_index],
                                             observation.fx,
                                             observation.fy,
                                             parameterization));
    }

} // namespace armor_detector::pose
