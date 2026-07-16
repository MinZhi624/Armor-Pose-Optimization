#include "armor_detector/pose/PoseLandscapeAnalyzer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofYPDRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/pose/YawSearchRefiner.hpp"
#include "armor_detector/tools/angle.hpp"
#include "armor_detector/tools/geometry.hpp"

namespace armor_detector::pose {

    namespace {
        constexpr double kGridComparisonEpsilon = 1e-10;

        bool isFinite(const cv::Vec3d &value) {
            return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
        }

        bool isFinite(const Eigen::Vector3d &value) {
            return value.allFinite();
        }

        bool isFinite(double value) {
            return std::isfinite(value);
        }

        debug::PoseLandscapeMetric toMetric(const Pose4DofCostEvaluation &evaluation) {
            debug::PoseLandscapeMetric metric;
            metric.valid = evaluation.valid;
            metric.status = evaluation.status;
            metric.cost = evaluation.cost;
            metric.mean_residual_px = evaluation.mean_residual_px;
            return metric;
        }

        debug::PoseLandscapeMarker unavailableMarker(const std::string &name, const std::string &status) {
            debug::PoseLandscapeMarker marker;
            marker.name = name;
            marker.status = status;
            return marker;
        }

        bool isInsideGrid(const debug::PoseLandscapeSample &sample, double distance_m, double pose_yaw_rad) {
            if (sample.distance_count == 0 || sample.pose_yaw_count == 0 || !isFinite(distance_m) ||
                !isFinite(pose_yaw_rad)) {
                return false;
            }
            const double distance_epsilon = std::max(std::abs(sample.distance_step_m) * kGridComparisonEpsilon, 1e-12);
            const double yaw_epsilon = std::max(std::abs(sample.pose_yaw_step_rad) * kGridComparisonEpsilon, 1e-12);
            return distance_m >= sample.actual_distance_min_m - distance_epsilon &&
                distance_m <= sample.actual_distance_max_m + distance_epsilon &&
                pose_yaw_rad >= sample.actual_pose_yaw_min_rad - yaw_epsilon &&
                pose_yaw_rad <= sample.actual_pose_yaw_max_rad + yaw_epsilon;
        }

        void setMarkerMetrics(debug::PoseLandscapeMarker &marker,
                              const Pose4DofCostEvaluation &actual,
                              const Pose4DofCostEvaluation &slice) {
            marker.actual_metric = toMetric(actual);
            marker.slice_metric = toMetric(slice);
            if (!marker.available) {
                return;
            }
            if (!actual.valid) {
                marker.status = "actual_" + actual.status;
            }
            else if (!slice.valid) {
                marker.status = "slice_" + slice.status;
            }
            else {
                marker.status = "ok";
            }
        }

        void setMarkerCoordinate(debug::PoseLandscapeSample &sample,
                                 debug::PoseLandscapeMarker &marker,
                                 double distance_m,
                                 double pose_yaw_rad) {
            marker.distance_m = distance_m;
            marker.pose_yaw_rad = pose_yaw_rad;
            marker.available = isFinite(distance_m) && isFinite(pose_yaw_rad);
            marker.inside_grid = marker.available && isInsideGrid(sample, distance_m, pose_yaw_rad);
            if (!marker.available) {
                marker.status = "non_finite_marker_coordinate";
            }
        }

        bool calculateGridCount(double lower, double upper, double step, std::size_t &count, double &actual_upper) {
            if (!isFinite(lower) || !isFinite(upper) || !isFinite(step) || step <= 0.0 || upper < lower) {
                return false;
            }
            const double step_count_double = (upper - lower) / step;
            if (!isFinite(step_count_double) ||
                step_count_double > static_cast<double>(std::numeric_limits<std::size_t>::max() - 1)) {
                return false;
            }
            const auto step_count = static_cast<std::size_t>(std::floor(step_count_double + kGridComparisonEpsilon));
            count = step_count + 1;
            actual_upper = lower + static_cast<double>(step_count) * step;
            return isFinite(actual_upper);
        }

        void setElapsed(debug::PoseLandscapeSample &sample, std::chrono::steady_clock::time_point start) {
            sample.scan_elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }

        bool hasValidParams(const PoseLandscapeParams &params) {
            return isFinite(params.physical_min_distance_m) && isFinite(params.physical_max_distance_m) &&
                isFinite(params.half_window_m) && isFinite(params.distance_step_m) &&
                isFinite(params.pose_yaw_min_deg) && isFinite(params.pose_yaw_max_deg) &&
                isFinite(params.pose_yaw_step_deg) && params.physical_min_distance_m > 0.0 &&
                params.physical_max_distance_m >= params.physical_min_distance_m && params.half_window_m >= 0.0 &&
                params.distance_step_m > 0.0 && params.pose_yaw_max_deg >= params.pose_yaw_min_deg &&
                params.pose_yaw_step_deg > 0.0;
        }
    } // namespace

    PoseLandscapeAnalyzer::PoseLandscapeAnalyzer(PoseLandscapeParams params) : params_(params) {
    }

    const PoseLandscapeParams &PoseLandscapeAnalyzer::params() const {
        return params_;
    }

    debug::PoseLandscapeSample PoseLandscapeAnalyzer::analyze(const PoseRefineInput &input,
                                                              const PoseLandscapeSampleInfo &sample_info) const {
        const auto scan_start = std::chrono::steady_clock::now();
        debug::PoseLandscapeSample sample;
        sample.armor_index = sample_info.armor_index;
        sample.armor_name = sample_info.armor_name;
        sample.armor_type = sample_info.armor_type;
        sample.confidence = sample_info.confidence;
        sample.image_corners = input.image_corners;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                sample.camera_matrix[static_cast<std::size_t>(row * 3 + column)] = input.camera_matrix(row, column);
            }
        }
        for (int i = 0; i < input.distortion_coefficients.rows; ++i) {
            sample.distortion_coefficients[static_cast<std::size_t>(i)] = input.distortion_coefficients[i];
        }
        sample.physical_min_distance_m = params_.physical_min_distance_m;
        sample.physical_max_distance_m = params_.physical_max_distance_m;
        sample.distance_step_m = params_.distance_step_m;
        sample.requested_pose_yaw_min_rad = tools::degToRad(params_.pose_yaw_min_deg);
        sample.requested_pose_yaw_max_rad = tools::degToRad(params_.pose_yaw_max_deg);
        sample.pose_yaw_step_rad = tools::degToRad(params_.pose_yaw_step_deg);
        sample.huber_loss_scale_px = kPose4DofHuberLossScalePx;
        sample.markers.push_back(unavailableMarker("pnp", "not_evaluated"));
        sample.markers.push_back(unavailableMarker("yaw_search", "not_evaluated"));
        sample.markers.push_back(unavailableMarker("ba_4dof_ypd", "not_evaluated"));
        sample.markers.push_back(unavailableMarker("grid_min", "not_evaluated"));

        const Pose4DofObservation observation = createPose4DofObservation(input);
        if (!observation.valid) {
            sample.status = "invalid_observation_" + observation.status;
            for (auto &marker : sample.markers) {
                marker.status = sample.status;
            }
            setElapsed(sample, scan_start);
            return sample;
        }
        if (!isFinite(input.initial_rvec) || !isFinite(input.initial_tvec)) {
            sample.status = "non_finite_pnp_pose";
            for (auto &marker : sample.markers) {
                marker.status = sample.status;
            }
            setElapsed(sample, scan_start);
            return sample;
        }

        ArmorPose initial_pose;
        try {
            initial_pose =
                calculateArmorPose(input.initial_rvec, input.initial_tvec, input.image_corners, input.camera_matrix);
        }
        catch (const cv::Exception &) {
            sample.status = "invalid_pnp_pose";
            for (auto &marker : sample.markers) {
                marker.status = sample.status;
            }
            setElapsed(sample, scan_start);
            return sample;
        }
        if (!isFinite(initial_pose.ypd_gimbal) || !isFinite(initial_pose.ypr_gimbal) ||
            initial_pose.ypd_gimbal.z() <= 0.0) {
            sample.status = "invalid_pnp_ypd";
            for (auto &marker : sample.markers) {
                marker.status = sample.status;
            }
            setElapsed(sample, scan_start);
            return sample;
        }

        const Eigen::Vector3d fixed_ypd = initial_pose.ypd_gimbal;
        const double theta0 = initial_pose.ypr_gimbal.x();
        sample.fixed_dir_yaw_rad = fixed_ypd.x();
        sample.fixed_dir_pitch_rad = fixed_ypd.y();
        sample.pnp_distance_m = fixed_ypd.z();
        sample.pnp_pose_yaw_rad = theta0;

        if (!hasValidParams(params_)) {
            sample.status = "invalid_landscape_parameters";
        }
        else {
            sample.requested_distance_min_m =
                std::max(params_.physical_min_distance_m, fixed_ypd.z() - params_.half_window_m);
            sample.requested_distance_max_m =
                std::min(params_.physical_max_distance_m, fixed_ypd.z() + params_.half_window_m);
            double actual_distance_max_m = std::numeric_limits<double>::quiet_NaN();
            double actual_pose_yaw_max_rad = std::numeric_limits<double>::quiet_NaN();
            if (!calculateGridCount(sample.requested_distance_min_m,
                                    sample.requested_distance_max_m,
                                    params_.distance_step_m,
                                    sample.distance_count,
                                    actual_distance_max_m)) {
                sample.status = "empty_distance_grid";
            }
            else if (!calculateGridCount(sample.requested_pose_yaw_min_rad,
                                         sample.requested_pose_yaw_max_rad,
                                         sample.pose_yaw_step_rad,
                                         sample.pose_yaw_count,
                                         actual_pose_yaw_max_rad)) {
                sample.status = "empty_pose_yaw_grid";
            }
            else {
                sample.actual_distance_min_m = sample.requested_distance_min_m;
                sample.actual_distance_max_m = actual_distance_max_m;
                sample.actual_pose_yaw_min_rad = sample.requested_pose_yaw_min_rad;
                sample.actual_pose_yaw_max_rad = actual_pose_yaw_max_rad;
                sample.status = "grid_ready";
            }
        }

        auto &pnp_marker = sample.markers[0];
        setMarkerCoordinate(sample, pnp_marker, fixed_ypd.z(), theta0);
        const Pose4DofCostEvaluation pnp_evaluation = evaluatePose4DofYpdCost(observation, fixed_ypd, theta0);
        setMarkerMetrics(pnp_marker, pnp_evaluation, pnp_evaluation);
        pnp_marker.direction_delta_yaw_rad = 0.0;
        pnp_marker.direction_delta_pitch_rad = 0.0;

        auto &yaw_marker = sample.markers[1];
        const auto yaw_start = std::chrono::steady_clock::now();
        const YawSearchRefiner yaw_search_refiner;
        const PoseRefineOutput yaw_output = yaw_search_refiner.refine(input);
        sample.yaw_search_elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - yaw_start).count();
        sample.yaw_search_success = yaw_output.success;
        if (!yaw_output.success || !isFinite(yaw_output.rvec) || !isFinite(yaw_output.tvec)) {
            sample.yaw_search_status = "yaw_search_failed";
            yaw_marker.status = sample.yaw_search_status;
        }
        else {
            try {
                const ArmorPose yaw_pose =
                    calculateArmorPose(yaw_output.rvec, yaw_output.tvec, input.image_corners, input.camera_matrix);
                setMarkerCoordinate(sample, yaw_marker, fixed_ypd.z(), yaw_pose.ypr_gimbal.x());
                const Pose4DofCostEvaluation yaw_actual =
                    evaluatePose4DofYpdCost(observation, yaw_pose.ypd_gimbal, yaw_pose.ypr_gimbal.x());
                const Pose4DofCostEvaluation yaw_slice =
                    evaluatePose4DofYpdCost(observation, fixed_ypd, yaw_pose.ypr_gimbal.x());
                setMarkerMetrics(yaw_marker, yaw_actual, yaw_slice);
                yaw_marker.direction_delta_yaw_rad = tools::normalizeRadAngle(yaw_pose.ypd_gimbal.x() - fixed_ypd.x());
                yaw_marker.direction_delta_pitch_rad = yaw_pose.ypd_gimbal.y() - fixed_ypd.y();
                sample.yaw_search_status = yaw_marker.status;
            }
            catch (const cv::Exception &) {
                sample.yaw_search_success = false;
                sample.yaw_search_status = "invalid_yaw_search_pose";
                yaw_marker.status = sample.yaw_search_status;
            }
        }

        auto &ba_marker = sample.markers[2];
        const auto ba_start = std::chrono::steady_clock::now();
        const PoseOnlyBa4DofYPDRefiner ba_refiner;
        const PoseRefineOutput ba_output = ba_refiner.refine(input);
        sample.ba_elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ba_start).count();
        sample.ba_success = ba_output.success;
        sample.ba_solver_summary_available = ba_output.solver_summary.available;
        if (sample.ba_solver_summary_available) {
            sample.ba_initial_cost = ba_output.solver_summary.initial_cost;
            sample.ba_final_cost = ba_output.solver_summary.final_cost;
            sample.ba_num_iterations = ba_output.solver_summary.num_iterations;
            sample.ba_termination_type = ba_output.solver_summary.termination_type;
        }
        if (!ba_output.success || !isFinite(ba_output.rvec) || !isFinite(ba_output.tvec)) {
            sample.ba_status = "ba_4dof_ypd_failed";
            if (sample.ba_solver_summary_available) {
                sample.ba_status += "_" + sample.ba_termination_type;
            }
            ba_marker.status = sample.ba_status;
        }
        else {
            try {
                const ArmorPose ba_pose =
                    calculateArmorPose(ba_output.rvec, ba_output.tvec, input.image_corners, input.camera_matrix);
                setMarkerCoordinate(sample, ba_marker, ba_pose.ypd_gimbal.z(), ba_pose.ypr_gimbal.x());
                const Pose4DofCostEvaluation ba_actual =
                    evaluatePose4DofYpdCost(observation, ba_pose.ypd_gimbal, ba_pose.ypr_gimbal.x());
                const Eigen::Vector3d ba_slice_ypd(fixed_ypd.x(), fixed_ypd.y(), ba_pose.ypd_gimbal.z());
                const Pose4DofCostEvaluation ba_slice =
                    evaluatePose4DofYpdCost(observation, ba_slice_ypd, ba_pose.ypr_gimbal.x());
                setMarkerMetrics(ba_marker, ba_actual, ba_slice);
                ba_marker.direction_delta_yaw_rad = tools::normalizeRadAngle(ba_pose.ypd_gimbal.x() - fixed_ypd.x());
                ba_marker.direction_delta_pitch_rad = ba_pose.ypd_gimbal.y() - fixed_ypd.y();
                sample.ba_status = ba_marker.status;
            }
            catch (const cv::Exception &) {
                sample.ba_success = false;
                sample.ba_status = "invalid_ba_4dof_ypd_pose";
                ba_marker.status = sample.ba_status;
            }
        }

        if (sample.status == "grid_ready") {
            const auto grid_start = std::chrono::steady_clock::now();
            sample.grid.reserve(sample.distance_count * sample.pose_yaw_count);
            bool has_grid_minimum = false;
            debug::PoseLandscapeGridPoint grid_minimum;
            for (std::size_t distance_index = 0; distance_index < sample.distance_count; ++distance_index) {
                const double distance_m =
                    sample.actual_distance_min_m + static_cast<double>(distance_index) * sample.distance_step_m;
                const Eigen::Vector3d ypd(fixed_ypd.x(), fixed_ypd.y(), distance_m);
                for (std::size_t yaw_index = 0; yaw_index < sample.pose_yaw_count; ++yaw_index) {
                    const double pose_yaw_rad =
                        sample.actual_pose_yaw_min_rad + static_cast<double>(yaw_index) * sample.pose_yaw_step_rad;
                    const Pose4DofCostEvaluation evaluation = evaluatePose4DofYpdCost(observation, ypd, pose_yaw_rad);
                    debug::PoseLandscapeGridPoint point;
                    point.distance_index = distance_index;
                    point.pose_yaw_index = yaw_index;
                    point.distance_m = distance_m;
                    point.pose_yaw_rad = pose_yaw_rad;
                    point.valid = evaluation.valid;
                    point.status = evaluation.status;
                    point.cost = evaluation.cost;
                    point.mean_residual_px = evaluation.mean_residual_px;
                    sample.grid.push_back(point);
                    if (point.valid && (!has_grid_minimum || point.cost < grid_minimum.cost)) {
                        grid_minimum = point;
                        has_grid_minimum = true;
                    }
                }
            }
            sample.grid_elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - grid_start).count();

            auto &grid_marker = sample.markers[3];
            if (has_grid_minimum) {
                setMarkerCoordinate(sample, grid_marker, grid_minimum.distance_m, grid_minimum.pose_yaw_rad);
                const Pose4DofCostEvaluation grid_evaluation =
                    evaluatePose4DofYpdCost(observation,
                                            Eigen::Vector3d(fixed_ypd.x(), fixed_ypd.y(), grid_minimum.distance_m),
                                            grid_minimum.pose_yaw_rad);
                setMarkerMetrics(grid_marker, grid_evaluation, grid_evaluation);
                grid_marker.direction_delta_yaw_rad = 0.0;
                grid_marker.direction_delta_pitch_rad = 0.0;
                sample.valid = true;
                sample.status = "ok";
            }
            else {
                grid_marker.status = "no_valid_grid_point";
                sample.status = "no_valid_grid_point";
            }
        }

        setElapsed(sample, scan_start);
        return sample;
    }

} // namespace armor_detector::pose
