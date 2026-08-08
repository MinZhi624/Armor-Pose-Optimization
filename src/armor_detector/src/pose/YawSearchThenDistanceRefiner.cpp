#include "armor_detector/pose/YawSearchThenDistanceRefiner.hpp"

#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/pose/YawSearchRefiner.hpp"

#include <ceres/ceres.h>

#include <cmath>
#include <limits>

namespace armor_detector::pose {

    namespace {
        constexpr double kMinimumDistanceM = std::numeric_limits<double>::epsilon();

        bool isUsableOutputPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
                    return false;
                }
            }
            return tvec[2] > 0.0;
        }

        bool isUsableRefineOutput(const PoseRefineOutput &output) {
            return output.success && std::isfinite(output.reprojection_error_px) &&
                isUsableOutputPose(output.rvec, output.tvec);
        }

        void copySolverSummary(const ceres::Solver::Summary &summary, PoseRefineSolverSummary &output_summary) {
            output_summary.available = true;
            output_summary.initial_cost = summary.initial_cost;
            output_summary.final_cost = summary.final_cost;
            output_summary.num_iterations =
                summary.iterations.empty() ? 0 : static_cast<int>(summary.iterations.size() - 1);
            output_summary.termination_type = ceres::TerminationTypeToString(summary.termination_type);
        }
    } // namespace

    PoseRefineOutput YawSearchThenDistanceRefiner::refineOne(const PoseRefineInput &input) const {
        PoseRefineOutput initial_output;
        initial_output.rvec = input.initial_rvec;
        initial_output.tvec = input.initial_tvec;
        initial_output.reprojection_error_px = calculateReprojectionError(input.armor_type,
                                                                          input.image_corners,
                                                                          input.initial_rvec,
                                                                          input.initial_tvec,
                                                                          input.camera_matrix,
                                                                          input.distortion_coefficients);

        if (input.armor_type == ArmorType::NONE || !isUsableOutputPose(input.initial_rvec, input.initial_tvec) ||
            !std::isfinite(initial_output.reprojection_error_px)) {
            return initial_output;
        }

        const PoseRefineOutput yaw_output = YawSearchRefiner{}.refine({input}).items.front();
        if (!isUsableRefineOutput(yaw_output)) {
            return initial_output;
        }

        const ArmorPose yaw_pose =
            calculateArmorPose(yaw_output.rvec, yaw_output.tvec, input.image_corners, input.camera_matrix);
        const Eigen::Vector3d ypd_gimbal = yaw_pose.ypd_gimbal;
        const double pose_yaw_rad = yaw_pose.ypr_gimbal.x();
        if (!ypd_gimbal.allFinite() || ypd_gimbal.z() <= kMinimumDistanceM || !std::isfinite(pose_yaw_rad)) {
            return yaw_output;
        }

        const Pose4DofObservation observation = createPose4DofObservation(input);
        if (!observation.valid) {
            return yaw_output;
        }

        double distance_m = ypd_gimbal.z();
        ceres::Problem problem;
        for (std::size_t i = 0; i < observation.object_points.size(); ++i) {
            auto *cost_function = createPose4DofDistanceReprojectionCostFunction(
                observation, i, ypd_gimbal.x(), ypd_gimbal.y(), pose_yaw_rad);
            if (cost_function == nullptr) {
                return yaw_output;
            }
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(kPose4DofHuberLossScalePx), &distance_m);
        }
        problem.SetParameterLowerBound(&distance_m, 0, kMinimumDistanceM);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 20;
        options.minimizer_progress_to_stdout = false;
        options.num_threads = 1;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (!summary.IsSolutionUsable() || !std::isfinite(distance_m) || distance_m <= kMinimumDistanceM) {
            return yaw_output;
        }

        Eigen::Vector3d refined_ypd_gimbal = ypd_gimbal;
        refined_ypd_gimbal.z() = distance_m;
        cv::Vec3d refined_rvec;
        cv::Vec3d refined_tvec;
        rvecTvecFromGimbalYpdYaw(refined_ypd_gimbal, pose_yaw_rad, refined_rvec, refined_tvec);
        const double refined_error_px = calculateReprojectionError(input.armor_type,
                                                                   input.image_corners,
                                                                   refined_rvec,
                                                                   refined_tvec,
                                                                   input.camera_matrix,
                                                                   input.distortion_coefficients);
        if (!isUsableOutputPose(refined_rvec, refined_tvec) || !std::isfinite(refined_error_px)) {
            return yaw_output;
        }

        PoseRefineOutput output;
        output.rvec = refined_rvec;
        output.tvec = refined_tvec;
        output.success = true;
        output.reprojection_error_px = refined_error_px;
        copySolverSummary(summary, output.solver_summary);
        return output;
    }

} // namespace armor_detector::pose
