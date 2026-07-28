#include "armor_detector/pose/DualArmorBa3DofYPDRefiner.hpp"

#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/angle.hpp"

#include <Eigen/Core>
#include <ceres/ceres.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace armor_detector::pose {

    namespace {
        constexpr double kQuarterTurnRad = M_PI / 2.0;
        constexpr double kNearZeroDirection = 1e-12;

        bool isFinitePose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
                    return false;
                }
            }
            return true;
        }

        bool isUsablePose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
            return isFinitePose(rvec, tvec) && tvec[2] > 0.0;
        }

        bool isQualifiedInitialPose(const PoseRefineInput &input) {
            return isUsablePose(input.initial_rvec, input.initial_tvec);
        }

        double imageCenterX(const PoseRefineInput &input) {
            double center_x = 0.0;
            for (const auto &corner : input.image_corners) {
                center_x += static_cast<double>(corner.x);
            }
            return center_x / static_cast<double>(input.image_corners.size());
        }

        PoseRefineOutput pnpOutput(const PoseRefineInput &input) {
            PoseRefineOutput output;
            output.rvec = input.initial_rvec;
            output.tvec = input.initial_tvec;
            output.success = isUsablePose(output.rvec, output.tvec);
            try {
                output.reprojection_error_px = calculateReprojectionError(input.armor_type,
                                                                          input.image_corners,
                                                                          output.rvec,
                                                                          output.tvec,
                                                                          input.camera_matrix,
                                                                          input.distortion_coefficients);
            }
            catch (const cv::Exception &) {
                output.reprojection_error_px = std::numeric_limits<double>::quiet_NaN();
            }
            return output;
        }

        void setPairPnpFallback(const std::vector<PoseRefineInput> &inputs,
                                std::size_t index_a,
                                std::size_t index_b,
                                PoseRefineBatchOutput &output,
                                const PoseRefineSolverSummary *solver_summary = nullptr) {
            output.items[index_a] = pnpOutput(inputs[index_a]);
            output.items[index_b] = pnpOutput(inputs[index_b]);
            if (solver_summary != nullptr) {
                output.items[index_a].solver_summary = *solver_summary;
                output.items[index_b].solver_summary = *solver_summary;
            }
            output.dual_summary.reset();
        }

        void copyFallbackItems(const std::vector<PoseRefineInput> &inputs,
                               const std::vector<std::size_t> &indices,
                               const IPoseRefiner *fallback_refiner,
                               PoseRefineBatchOutput &output) {
            if (indices.empty()) {
                return;
            }

            if (fallback_refiner == nullptr) {
                for (const std::size_t index : indices) {
                    output.items[index] = pnpOutput(inputs[index]);
                }
                return;
            }

            std::vector<PoseRefineInput> fallback_inputs;
            fallback_inputs.reserve(indices.size());
            for (const std::size_t index : indices) {
                fallback_inputs.push_back(inputs[index]);
            }
            const PoseRefineBatchOutput fallback_output = fallback_refiner->refine(fallback_inputs);
            for (std::size_t i = 0; i < indices.size(); ++i) {
                if (i < fallback_output.items.size()) {
                    output.items[indices[i]] = fallback_output.items[i];
                }
                else {
                    output.items[indices[i]] = pnpOutput(inputs[indices[i]]);
                }
            }
        }

        bool findUniquePair(const std::vector<PoseRefineInput> &inputs, std::size_t &index_a, std::size_t &index_b) {
            std::vector<std::array<std::size_t, 2>> pairs;

            for (std::size_t first = 0; first < inputs.size(); ++first) {
                if (inputs[first].armor_name == ArmorName::NONE) {
                    continue;
                }
                bool seen_name = false;
                for (std::size_t previous = 0; previous < first; ++previous) {
                    if (inputs[previous].armor_name == inputs[first].armor_name) {
                        seen_name = true;
                        break;
                    }
                }
                if (seen_name) {
                    continue;
                }

                std::vector<std::size_t> group;
                for (std::size_t index = 0; index < inputs.size(); ++index) {
                    if (inputs[index].armor_name == inputs[first].armor_name) {
                        group.push_back(index);
                    }
                }
                if (group.size() == 2 && inputs[group[0]].armor_type == inputs[group[1]].armor_type &&
                    inputs[group[0]].armor_type != ArmorType::NONE && isQualifiedInitialPose(inputs[group[0]]) &&
                    isQualifiedInitialPose(inputs[group[1]])) {
                    pairs.push_back({group[0], group[1]});
                }
            }

            if (pairs.size() != 1) {
                return false;
            }

            const auto &pair = pairs.front();
            const double center_a = imageCenterX(inputs[pair[0]]);
            const double center_b = imageCenterX(inputs[pair[1]]);
            if (center_b < center_a) {
                index_a = pair[1];
                index_b = pair[0];
            }
            else {
                index_a = pair[0];
                index_b = pair[1];
            }
            return true;
        }

        bool isFiniteYpd(const Eigen::Vector3d &ypd) {
            return ypd.allFinite();
        }

        bool initializeSharedYaw(const ArmorPose &pose_a, const ArmorPose &pose_b, double &shared_yaw_rad) {
            const double sin_sum = std::sin(pose_a.ypr_gimbal.x()) + std::sin(pose_b.ypr_gimbal.x() - kQuarterTurnRad);
            const double cos_sum = std::cos(pose_a.ypr_gimbal.x()) + std::cos(pose_b.ypr_gimbal.x() - kQuarterTurnRad);
            const double norm = std::hypot(sin_sum, cos_sum);
            if (!std::isfinite(norm) || norm <= kNearZeroDirection) {
                return false;
            }
            shared_yaw_rad = tools::normalizeRadAngle(std::atan2(sin_sum, cos_sum));
            return std::isfinite(shared_yaw_rad);
        }

        PoseRefineSolverSummary solverSummary(const ceres::Solver::Summary &summary) {
            PoseRefineSolverSummary result;
            result.available = true;
            result.initial_cost = summary.initial_cost;
            result.final_cost = summary.final_cost;
            result.num_iterations = summary.iterations.empty() ? 0 : static_cast<int>(summary.iterations.size() - 1);
            result.termination_type = ceres::TerminationTypeToString(summary.termination_type);
            return result;
        }
    } // namespace

    void DualArmorBa3DofYPDRefiner::setFallbackRefiner(const IPoseRefiner &fallback_refiner) {
        fallback_refiner_ = &fallback_refiner;
    }

    PoseRefineBatchOutput DualArmorBa3DofYPDRefiner::refine(const std::vector<PoseRefineInput> &inputs) const {
        PoseRefineBatchOutput output;
        output.items.resize(inputs.size());
        output.dual_summary.reset();

        std::size_t index_a = 0;
        std::size_t index_b = 0;
        if (!findUniquePair(inputs, index_a, index_b)) {
            if (fallback_refiner_ != nullptr) {
                output = fallback_refiner_->refine(inputs);
                output.dual_summary.reset();
                return output;
            }
            for (std::size_t i = 0; i < inputs.size(); ++i) {
                output.items[i] = pnpOutput(inputs[i]);
            }
            return output;
        }

        const Pose4DofObservation observation_a = createPose4DofObservation(inputs[index_a]);
        const Pose4DofObservation observation_b = createPose4DofObservation(inputs[index_b]);
        const ArmorPose initial_pose_a = calculateArmorPose(inputs[index_a].initial_rvec,
                                                            inputs[index_a].initial_tvec,
                                                            inputs[index_a].image_corners,
                                                            inputs[index_a].camera_matrix);
        const ArmorPose initial_pose_b = calculateArmorPose(inputs[index_b].initial_rvec,
                                                            inputs[index_b].initial_tvec,
                                                            inputs[index_b].image_corners,
                                                            inputs[index_b].camera_matrix);

        const Eigen::Vector3d fixed_ypd_a(tools::normalizeRadAngle(initial_pose_a.ypd_gimbal.x()),
                                          tools::normalizeRadAngle(initial_pose_a.ypd_gimbal.y()),
                                          initial_pose_a.ypd_gimbal.z());
        const Eigen::Vector3d fixed_ypd_b(tools::normalizeRadAngle(initial_pose_b.ypd_gimbal.x()),
                                          tools::normalizeRadAngle(initial_pose_b.ypd_gimbal.y()),
                                          initial_pose_b.ypd_gimbal.z());
        double shared_yaw_rad = 0.0;
        if (!observation_a.valid || !observation_b.valid || !isFiniteYpd(fixed_ypd_a) || !isFiniteYpd(fixed_ypd_b) ||
            !initializeSharedYaw(initial_pose_a, initial_pose_b, shared_yaw_rad)) {
            setPairPnpFallback(inputs, index_a, index_b, output);
        }
        else {
            double distance_a_m = fixed_ypd_a.z();
            double distance_b_m = fixed_ypd_b.z();
            ceres::Problem problem;
            bool costs_valid = true;
            for (std::size_t corner = 0; corner < 4; ++corner) {
                auto *cost_a = createPose4DofSharedYawDistanceReprojectionCostFunction(
                    observation_a, corner, fixed_ypd_a.x(), fixed_ypd_a.y(), 0.0);
                auto *cost_b = createPose4DofSharedYawDistanceReprojectionCostFunction(
                    observation_b, corner, fixed_ypd_b.x(), fixed_ypd_b.y(), kQuarterTurnRad);
                if (cost_a == nullptr || cost_b == nullptr) {
                    delete cost_a;
                    delete cost_b;
                    costs_valid = false;
                    break;
                }
                problem.AddResidualBlock(
                    cost_a, new ceres::HuberLoss(kPose4DofHuberLossScalePx), &shared_yaw_rad, &distance_a_m);
                problem.AddResidualBlock(
                    cost_b, new ceres::HuberLoss(kPose4DofHuberLossScalePx), &shared_yaw_rad, &distance_b_m);
            }

            if (!costs_valid) {
                setPairPnpFallback(inputs, index_a, index_b, output);
            }
            else {
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 20;
                options.minimizer_progress_to_stdout = false;
                options.num_threads = 1;

                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                const PoseRefineSolverSummary pose_summary = solverSummary(summary);
                const bool finite_solution = summary.IsSolutionUsable() && std::isfinite(shared_yaw_rad) &&
                    std::isfinite(distance_a_m) && std::isfinite(distance_b_m) && distance_a_m > 0.0 &&
                    distance_b_m > 0.0;

                if (!finite_solution) {
                    setPairPnpFallback(inputs, index_a, index_b, output, &pose_summary);
                }
                else {
                    const double yaw_a_rad = tools::normalizeRadAngle(shared_yaw_rad);
                    const double yaw_b_rad = tools::normalizeRadAngle(shared_yaw_rad + kQuarterTurnRad);
                    const Eigen::Vector3d refined_ypd_a(fixed_ypd_a.x(), fixed_ypd_a.y(), distance_a_m);
                    const Eigen::Vector3d refined_ypd_b(fixed_ypd_b.x(), fixed_ypd_b.y(), distance_b_m);
                    cv::Vec3d refined_rvec_a;
                    cv::Vec3d refined_tvec_a;
                    cv::Vec3d refined_rvec_b;
                    cv::Vec3d refined_tvec_b;
                    rvecTvecFromGimbalYpdYaw(refined_ypd_a, yaw_a_rad, refined_rvec_a, refined_tvec_a);
                    rvecTvecFromGimbalYpdYaw(refined_ypd_b, yaw_b_rad, refined_rvec_b, refined_tvec_b);
                    const Pose4DofCostEvaluation evaluation_a =
                        evaluatePose4DofYpdCost(observation_a, refined_ypd_a, yaw_a_rad);
                    const Pose4DofCostEvaluation evaluation_b =
                        evaluatePose4DofYpdCost(observation_b, refined_ypd_b, yaw_b_rad);
                    const double reprojection_error_a =
                        calculateReprojectionError(inputs[index_a].armor_type,
                                                   inputs[index_a].image_corners,
                                                   refined_rvec_a,
                                                   refined_tvec_a,
                                                   inputs[index_a].camera_matrix,
                                                   inputs[index_a].distortion_coefficients);
                    const double reprojection_error_b =
                        calculateReprojectionError(inputs[index_b].armor_type,
                                                   inputs[index_b].image_corners,
                                                   refined_rvec_b,
                                                   refined_tvec_b,
                                                   inputs[index_b].camera_matrix,
                                                   inputs[index_b].distortion_coefficients);
                    const bool valid_pair = isUsablePose(refined_rvec_a, refined_tvec_a) &&
                        isUsablePose(refined_rvec_b, refined_tvec_b) && evaluation_a.valid && evaluation_b.valid &&
                        std::isfinite(reprojection_error_a) && std::isfinite(reprojection_error_b);

                    if (!valid_pair) {
                        setPairPnpFallback(inputs, index_a, index_b, output, &pose_summary);
                    }
                    else {
                        output.items[index_a].rvec = refined_rvec_a;
                        output.items[index_a].tvec = refined_tvec_a;
                        output.items[index_a].success = true;
                        output.items[index_a].reprojection_error_px = reprojection_error_a;
                        output.items[index_a].solver_summary = pose_summary;
                        output.items[index_b].rvec = refined_rvec_b;
                        output.items[index_b].tvec = refined_tvec_b;
                        output.items[index_b].success = true;
                        output.items[index_b].reprojection_error_px = reprojection_error_b;
                        output.items[index_b].solver_summary = pose_summary;

                        DualArmorPoseRefineSummary dual_summary;
                        dual_summary.armor_a_index = index_a;
                        dual_summary.armor_b_index = index_b;
                        dual_summary.shared_pose_yaw_rad = yaw_a_rad;
                        dual_summary.distance_a_m = distance_a_m;
                        dual_summary.distance_b_m = distance_b_m;
                        dual_summary.armor_a_mean_reprojection_error_px = evaluation_a.mean_residual_px;
                        dual_summary.armor_b_mean_reprojection_error_px = evaluation_b.mean_residual_px;
                        dual_summary.mean_reprojection_error_px =
                            (evaluation_a.mean_residual_px + evaluation_b.mean_residual_px) / 2.0;
                        dual_summary.solver_summary = pose_summary;
                        output.dual_summary = dual_summary;
                    }
                }
            }
        }

        std::vector<std::size_t> fallback_indices;
        fallback_indices.reserve(inputs.size() - 2);
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            if (i != index_a && i != index_b) {
                fallback_indices.push_back(i);
            }
        }
        copyFallbackItems(inputs, fallback_indices, fallback_refiner_, output);
        return output;
    }

} // namespace armor_detector::pose
