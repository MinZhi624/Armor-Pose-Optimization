#include "armor_detector/pose/DualArmorBa7DofXYZRefiner.hpp"

#include "DualArmorJointRefineCommon.hpp"
#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/angle.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace armor_detector::pose {
    namespace {
        constexpr double kQuarterTurnRad = M_PI / 2.0;
        constexpr double kCostTieRelativeTolerance = 1e-12;

        struct Candidate {
            bool usable = false;
            double yaw_offset_rad = 0.0;
            double shared_yaw_rad = 0.0;
            Eigen::Vector3d xyz_a = Eigen::Vector3d::Zero();
            Eigen::Vector3d xyz_b = Eigen::Vector3d::Zero();
            cv::Vec3d rvec_a;
            cv::Vec3d tvec_a;
            cv::Vec3d rvec_b;
            cv::Vec3d tvec_b;
            Pose4DofCostEvaluation evaluation_a;
            Pose4DofCostEvaluation evaluation_b;
            double reprojection_error_a = std::numeric_limits<double>::quiet_NaN();
            double reprojection_error_b = std::numeric_limits<double>::quiet_NaN();
            PoseRefineSolverSummary solver_summary;
        };

        PoseRefineSolverSummary solverSummary(const ceres::Solver::Summary &summary) {
            return {true,
                    summary.initial_cost,
                    summary.final_cost,
                    summary.iterations.empty() ? 0 : static_cast<int>(summary.iterations.size() - 1),
                    ceres::TerminationTypeToString(summary.termination_type)};
        }

        bool isUsablePose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
                    return false;
                }
            }
            return tvec[2] > 0.0;
        }

        Candidate solveCandidate(const PoseRefineInput &input_a,
                                 const PoseRefineInput &input_b,
                                 const Pose4DofObservation &observation_a,
                                 const Pose4DofObservation &observation_b,
                                 const ArmorPose &pose_a,
                                 const ArmorPose &pose_b,
                                 double yaw_offset_rad) {
            Candidate result;
            result.yaw_offset_rad = yaw_offset_rad;
            result.xyz_a = pose_a.xyz_gimbal;
            result.xyz_b = pose_b.xyz_gimbal;
            if (!observation_a.valid || !observation_b.valid || !result.xyz_a.allFinite() ||
                !result.xyz_b.allFinite() ||
                !dual_armor_detail::initializeSharedYaw(pose_a, pose_b, yaw_offset_rad, result.shared_yaw_rad)) {
                return result;
            }
            ceres::Problem problem;
            for (std::size_t corner = 0; corner < 4; ++corner) {
                auto *cost_a = createPose4DofSharedYawXyzReprojectionCostFunction(observation_a, corner, 0.0);
                auto *cost_b =
                    createPose4DofSharedYawXyzReprojectionCostFunction(observation_b, corner, yaw_offset_rad);
                if (cost_a == nullptr || cost_b == nullptr) {
                    delete cost_a;
                    delete cost_b;
                    return result;
                }
                problem.AddResidualBlock(cost_a,
                                         new ceres::HuberLoss(kPose4DofHuberLossScalePx),
                                         &result.shared_yaw_rad,
                                         result.xyz_a.data());
                problem.AddResidualBlock(cost_b,
                                         new ceres::HuberLoss(kPose4DofHuberLossScalePx),
                                         &result.shared_yaw_rad,
                                         result.xyz_b.data());
            }
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 20;
            options.num_threads = 1;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            result.solver_summary = solverSummary(summary);
            if (!summary.IsSolutionUsable() || !std::isfinite(summary.final_cost) ||
                !std::isfinite(result.shared_yaw_rad) || !result.xyz_a.allFinite() || !result.xyz_b.allFinite() ||
                result.xyz_a.norm() <= 0.0 || result.xyz_b.norm() <= 0.0) {
                return result;
            }
            result.shared_yaw_rad = tools::normalizeRadAngle(result.shared_yaw_rad);
            rvecTvecFromGimbalXyzYaw(result.xyz_a, result.shared_yaw_rad, result.rvec_a, result.tvec_a);
            rvecTvecFromGimbalXyzYaw(result.xyz_b,
                                     tools::normalizeRadAngle(result.shared_yaw_rad + yaw_offset_rad),
                                     result.rvec_b,
                                     result.tvec_b);
            result.evaluation_a = evaluatePose4DofCost(observation_a, result.xyz_a, result.shared_yaw_rad);
            result.evaluation_b = evaluatePose4DofCost(
                observation_b, result.xyz_b, tools::normalizeRadAngle(result.shared_yaw_rad + yaw_offset_rad));
            result.reprojection_error_a = calculateReprojectionError(input_a.armor_type,
                                                                     input_a.image_corners,
                                                                     result.rvec_a,
                                                                     result.tvec_a,
                                                                     input_a.camera_matrix,
                                                                     input_a.distortion_coefficients);
            result.reprojection_error_b = calculateReprojectionError(input_b.armor_type,
                                                                     input_b.image_corners,
                                                                     result.rvec_b,
                                                                     result.tvec_b,
                                                                     input_b.camera_matrix,
                                                                     input_b.distortion_coefficients);
            result.usable = isUsablePose(result.rvec_a, result.tvec_a) && isUsablePose(result.rvec_b, result.tvec_b) &&
                result.evaluation_a.valid && result.evaluation_b.valid && std::isfinite(result.reprojection_error_a) &&
                std::isfinite(result.reprojection_error_b);
            return result;
        }

        const Candidate *selectCandidate(const Candidate &plus, const Candidate &minus) {
            if (!plus.usable) {
                return minus.usable ? &minus : nullptr;
            }
            if (!minus.usable) {
                return &plus;
            }
            const double tolerance = kCostTieRelativeTolerance *
                std::max({1.0, std::abs(plus.solver_summary.final_cost), std::abs(minus.solver_summary.final_cost)});
            return minus.solver_summary.final_cost + tolerance < plus.solver_summary.final_cost ? &minus : &plus;
        }
    } // namespace

    void DualArmorBa7DofXYZRefiner::setFallbackRefiner(const IPoseRefiner &fallback_refiner) {
        fallback_refiner_ = &fallback_refiner;
    }

    PoseRefineBatchOutput DualArmorBa7DofXYZRefiner::refine(const std::vector<PoseRefineInput> &inputs) const {
        PoseRefineBatchOutput output;
        output.items.resize(inputs.size());
        dual_armor_detail::PairIndices pair;
        if (!dual_armor_detail::findUniquePair(inputs, pair)) {
            if (fallback_refiner_ != nullptr) {
                return fallback_refiner_->refine(inputs);
            }
            for (std::size_t i = 0; i < inputs.size(); ++i) {
                output.items[i] = dual_armor_detail::pnpOutput(inputs[i]);
            }
            return output;
        }
        const auto observation_a = createPose4DofObservation(inputs[pair.armor_a_index]);
        const auto observation_b = createPose4DofObservation(inputs[pair.armor_b_index]);
        const ArmorPose pose_a = calculateArmorPose(inputs[pair.armor_a_index].initial_rvec,
                                                    inputs[pair.armor_a_index].initial_tvec,
                                                    inputs[pair.armor_a_index].image_corners,
                                                    inputs[pair.armor_a_index].camera_matrix);
        const ArmorPose pose_b = calculateArmorPose(inputs[pair.armor_b_index].initial_rvec,
                                                    inputs[pair.armor_b_index].initial_tvec,
                                                    inputs[pair.armor_b_index].image_corners,
                                                    inputs[pair.armor_b_index].camera_matrix);
        const Candidate plus = solveCandidate(inputs[pair.armor_a_index],
                                              inputs[pair.armor_b_index],
                                              observation_a,
                                              observation_b,
                                              pose_a,
                                              pose_b,
                                              kQuarterTurnRad);
        const Candidate minus = solveCandidate(inputs[pair.armor_a_index],
                                               inputs[pair.armor_b_index],
                                               observation_a,
                                               observation_b,
                                               pose_a,
                                               pose_b,
                                               -kQuarterTurnRad);
        const Candidate *winner = selectCandidate(plus, minus);
        if (winner == nullptr) {
            dual_armor_detail::setPairPnpFallback(inputs, pair, output);
        }
        else {
            output.items[pair.armor_a_index] = {
                winner->rvec_a, winner->tvec_a, true, winner->reprojection_error_a, winner->solver_summary};
            output.items[pair.armor_b_index] = {
                winner->rvec_b, winner->tvec_b, true, winner->reprojection_error_b, winner->solver_summary};
            output.dual_summary = DualArmorPoseRefineSummary{
                pair.armor_a_index,
                pair.armor_b_index,
                winner->shared_yaw_rad,
                winner->xyz_a.norm(),
                winner->xyz_b.norm(),
                winner->evaluation_a.mean_residual_px,
                winner->evaluation_b.mean_residual_px,
                (winner->evaluation_a.mean_residual_px + winner->evaluation_b.mean_residual_px) / 2.0,
                winner->solver_summary};
        }
        dual_armor_detail::copyFallbackItems(inputs, pair, fallback_refiner_, output);
        return output;
    }
} // namespace armor_detector::pose
