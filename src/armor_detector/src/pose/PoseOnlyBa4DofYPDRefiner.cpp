#include "armor_detector/pose/PoseOnlyBa4DofYPDRefiner.hpp"
#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

#include <Eigen/Core>
#include <ceres/ceres.h>

#include <cmath>

namespace armor_detector::pose {

    namespace {
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

    PoseRefineOutput PoseOnlyBa4DofYPDRefiner::refineOne(const PoseRefineInput &input) const {
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

        const Pose4DofObservation observation = createPose4DofObservation(input);
        if (!observation.valid) {
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
        for (std::size_t i = 0; i < observation.object_points.size(); ++i) {
            auto *cost_function = createPose4DofReprojectionCostFunction(observation, i, Pose4DofParameterization::YPD);
            if (cost_function == nullptr) {
                return output;
            }
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(kPose4DofHuberLossScalePx), pose);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 20;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = 1;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        output.solver_summary.available = true;
        output.solver_summary.initial_cost = summary.initial_cost;
        output.solver_summary.final_cost = summary.final_cost;
        output.solver_summary.num_iterations =
            summary.iterations.empty() ? 0 : static_cast<int>(summary.iterations.size() - 1);
        output.solver_summary.termination_type = ceres::TerminationTypeToString(summary.termination_type);

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
