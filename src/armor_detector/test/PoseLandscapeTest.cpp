#include <cmath>
#include <gtest/gtest.h>

#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseLandscapeAnalyzer.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/pose/PoseRefineData.hpp"
#include "armor_detector/pose/PoseRefineRunner.hpp"

namespace armor_detector::pose {

    namespace {
        constexpr double kTolerance = 1e-8;
        constexpr double kProjectionTolerance = 1e-3;

        PoseRefineInput makeSyntheticInput(double distance_m, double pose_yaw_rad, double pixel_offset) {
            PoseRefineInput input;
            input.armor_type = ArmorType::SMALL;
            input.camera_matrix = cv::Matx33d(800.0, 0.0, 640.0, 0.0, 810.0, 512.0, 0.0, 0.0, 1.0);
            input.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};

            const Eigen::Vector3d ypd_gimbal(0.15, -0.08, distance_m);
            rvecTvecFromGimbalYpdYaw(ypd_gimbal, pose_yaw_rad, input.initial_rvec, input.initial_tvec);
            const auto projected = projectArmor(input.armor_type,
                                                input.initial_rvec,
                                                input.initial_tvec,
                                                input.camera_matrix,
                                                input.distortion_coefficients);
            if (projected.size() != input.image_corners.size()) {
                ADD_FAILURE() << "Synthetic projection did not produce four corners";
                return input;
            }
            for (std::size_t i = 0; i < input.image_corners.size(); ++i) {
                input.image_corners[i] = {projected[i].x + static_cast<float>(pixel_offset), projected[i].y};
            }
            return input;
        }

        PoseRefineInput makeDistanceRefineInput(double initial_distance_m,
                                                double target_distance_m,
                                                double initial_pose_yaw_rad,
                                                double target_pose_yaw_rad) {
            PoseRefineInput input;
            input.armor_type = ArmorType::SMALL;
            input.camera_matrix = cv::Matx33d(800.0, 0.0, 640.0, 0.0, 810.0, 512.0, 0.0, 0.0, 1.0);
            input.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};

            const Eigen::Vector3d target_ypd_gimbal(0.15, -0.08, target_distance_m);
            cv::Vec3d target_rvec;
            cv::Vec3d target_tvec;
            rvecTvecFromGimbalYpdYaw(target_ypd_gimbal, target_pose_yaw_rad, target_rvec, target_tvec);
            const auto projected = projectArmor(
                input.armor_type, target_rvec, target_tvec, input.camera_matrix, input.distortion_coefficients);
            if (projected.size() != input.image_corners.size()) {
                ADD_FAILURE() << "Synthetic projection did not produce four corners";
                return input;
            }
            for (std::size_t i = 0; i < input.image_corners.size(); ++i) {
                input.image_corners[i] = projected[i];
            }

            const Eigen::Vector3d initial_ypd_gimbal(0.15, -0.08, initial_distance_m);
            rvecTvecFromGimbalYpdYaw(initial_ypd_gimbal, initial_pose_yaw_rad, input.initial_rvec, input.initial_tvec);
            return input;
        }

        double expectedHuberCost(double residual_px) {
            const double squared_residual = residual_px * residual_px;
            const double scale = kPose4DofHuberLossScalePx;
            const double rho = (squared_residual <= scale * scale)
                ? squared_residual
                : 2.0 * scale * std::sqrt(squared_residual) - scale * scale;
            return 4.0 * 0.5 * rho;
        }

        const debug::PoseLandscapeMarker *findMarker(const debug::PoseLandscapeSample &sample, const char *name) {
            for (const auto &marker : sample.markers) {
                if (marker.name == name) {
                    return &marker;
                }
            }
            return nullptr;
        }

        void expectSamePoseOutput(const PoseRefineOutput &expected, const PoseRefineOutput &actual) {
            for (int i = 0; i < 3; ++i) {
                EXPECT_DOUBLE_EQ(actual.rvec[i], expected.rvec[i]);
                EXPECT_DOUBLE_EQ(actual.tvec[i], expected.tvec[i]);
            }
            EXPECT_EQ(actual.success, expected.success);
            EXPECT_DOUBLE_EQ(actual.reprojection_error_px, expected.reprojection_error_px);
            EXPECT_EQ(actual.solver_summary.available, expected.solver_summary.available);
        }
    } // namespace

    TEST(PoseRefineData, ParsesYawSearchThenDistanceMethod) {
        EXPECT_EQ(singlePoseRefineMethodFromString("yaw_search_then_distance"),
                  SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE);
        EXPECT_EQ(toString(SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE), "yaw_search_then_distance");
    }

    TEST(YawSearchThenDistanceRefiner, LocksYawAndDirectionsWhileOptimizingDistance) {
        constexpr double kTargetDistanceM = 5.0;
        const PoseRefineInput input = makeDistanceRefineInput(4.0, kTargetDistanceM, 0.0, 0.0);

        PoseRefineRunner yaw_runner;
        yaw_runner.setSingleMethod(SinglePoseRefineMethod::YAW_SEARCH);
        yaw_runner.setDualMethod(DualPoseRefineMethod::NONE);
        const auto yaw_batch = yaw_runner.refine({input});
        ASSERT_EQ(yaw_batch.items.size(), 1U);
        const PoseRefineOutput &yaw_output = yaw_batch.items[0];
        ASSERT_TRUE(yaw_output.success);

        PoseRefineRunner runner;
        runner.setSingleMethod(SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE);
        runner.setDualMethod(DualPoseRefineMethod::NONE);
        const auto batch = runner.refine({input});
        ASSERT_EQ(batch.items.size(), 1U);
        const PoseRefineOutput &output = batch.items[0];
        ASSERT_TRUE(output.success);
        ASSERT_TRUE(output.solver_summary.available);

        const ArmorPose yaw_pose =
            calculateArmorPose(yaw_output.rvec, yaw_output.tvec, input.image_corners, input.camera_matrix);
        const ArmorPose refined_pose =
            calculateArmorPose(output.rvec, output.tvec, input.image_corners, input.camera_matrix);
        EXPECT_NEAR(refined_pose.ypr_gimbal.x(), yaw_pose.ypr_gimbal.x(), kTolerance);
        EXPECT_NEAR(refined_pose.ypd_gimbal.x(), yaw_pose.ypd_gimbal.x(), kTolerance);
        EXPECT_NEAR(refined_pose.ypd_gimbal.y(), yaw_pose.ypd_gimbal.y(), kTolerance);
        EXPECT_LT(std::abs(refined_pose.ypd_gimbal.z() - kTargetDistanceM),
                  std::abs(yaw_pose.ypd_gimbal.z() - kTargetDistanceM));
        EXPECT_LT(output.reprojection_error_px, yaw_output.reprojection_error_px);
    }

    TEST(YawSearchThenDistanceRefiner, FallsBackToYawSearchWhenDistanceSetupFails) {
        PoseRefineInput input = makeDistanceRefineInput(4.0, 5.0, 0.0, 0.0);
        input.camera_matrix(0, 0) = 0.0;

        PoseRefineRunner yaw_runner;
        yaw_runner.setSingleMethod(SinglePoseRefineMethod::YAW_SEARCH);
        yaw_runner.setDualMethod(DualPoseRefineMethod::NONE);
        const auto yaw_batch = yaw_runner.refine({input});
        ASSERT_EQ(yaw_batch.items.size(), 1U);
        const PoseRefineOutput &yaw_output = yaw_batch.items[0];
        ASSERT_TRUE(yaw_output.success);
        ASSERT_TRUE(std::isfinite(yaw_output.reprojection_error_px));

        PoseRefineRunner runner;
        runner.setSingleMethod(SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE);
        runner.setDualMethod(DualPoseRefineMethod::NONE);
        const auto batch = runner.refine({input});
        ASSERT_EQ(batch.items.size(), 1U);
        const PoseRefineOutput &output = batch.items[0];
        ASSERT_TRUE(output.success);
        expectSamePoseOutput(yaw_output, output);
        EXPECT_FALSE(output.solver_summary.available);
    }

    TEST(Pose4DofCostEvaluator, MatchesCeresInitialCostOnBothHuberSides) {
        for (const double pixel_offset : {2.0, 5.0}) {
            const PoseRefineInput input = makeSyntheticInput(5.0, 0.0, pixel_offset);
            const Pose4DofObservation observation = createPose4DofObservation(input);
            ASSERT_TRUE(observation.valid) << observation.status;

            const ArmorPose initial_pose =
                calculateArmorPose(input.initial_rvec, input.initial_tvec, input.image_corners, input.camera_matrix);
            const Pose4DofCostEvaluation ypd_evaluation =
                evaluatePose4DofYpdCost(observation, initial_pose.ypd_gimbal, initial_pose.ypr_gimbal.x());
            const Pose4DofCostEvaluation xyz_evaluation =
                evaluatePose4DofCost(observation, initial_pose.xyz_gimbal, initial_pose.ypr_gimbal.x());
            ASSERT_TRUE(ypd_evaluation.valid) << ypd_evaluation.status;
            ASSERT_TRUE(xyz_evaluation.valid) << xyz_evaluation.status;
            EXPECT_NEAR(ypd_evaluation.cost, expectedHuberCost(pixel_offset), kProjectionTolerance);
            EXPECT_NEAR(xyz_evaluation.cost, expectedHuberCost(pixel_offset), kProjectionTolerance);

            PoseRefineRunner ypd_runner;
            ypd_runner.setSingleMethod(SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_YPD);
            ypd_runner.setDualMethod(DualPoseRefineMethod::NONE);
            const auto ypd_batch = ypd_runner.refine({input});
            ASSERT_EQ(ypd_batch.items.size(), 1U);
            const PoseRefineOutput &ypd_output = ypd_batch.items[0];
            ASSERT_TRUE(ypd_output.solver_summary.available);
            EXPECT_NEAR(ypd_evaluation.cost, ypd_output.solver_summary.initial_cost, kTolerance);

            PoseRefineRunner xyz_runner;
            xyz_runner.setSingleMethod(SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ);
            xyz_runner.setDualMethod(DualPoseRefineMethod::NONE);
            const auto xyz_batch = xyz_runner.refine({input});
            ASSERT_EQ(xyz_batch.items.size(), 1U);
            const PoseRefineOutput &xyz_output = xyz_batch.items[0];
            ASSERT_TRUE(xyz_output.solver_summary.available);
            EXPECT_NEAR(xyz_evaluation.cost, xyz_output.solver_summary.initial_cost, kTolerance);
        }
    }

    TEST(PoseLandscapeAnalyzer, FindsKnownMinimumAndUsesConfirmedFullWindow) {
        const PoseRefineInput input = makeSyntheticInput(5.0, 0.0, 0.0);
        PoseLandscapeParams params;
        params.enabled = true;
        PoseLandscapeAnalyzer analyzer(params);
        PoseLandscapeSampleInfo info;
        info.armor_index = 2;
        info.armor_name = 3;
        info.armor_type = "small";
        info.confidence = 0.9;

        const debug::PoseLandscapeSample sample = analyzer.analyze(input, info);
        ASSERT_TRUE(sample.valid) << sample.status;
        EXPECT_EQ(sample.distance_count, 121U);
        EXPECT_EQ(sample.pose_yaw_count, 71U);
        EXPECT_EQ(sample.grid.size(), 121U * 71U);

        const auto *grid_minimum = findMarker(sample, "grid_min");
        ASSERT_NE(grid_minimum, nullptr);
        ASSERT_TRUE(grid_minimum->available) << grid_minimum->status;
        ASSERT_TRUE(grid_minimum->actual_metric.valid) << grid_minimum->actual_metric.status;
        EXPECT_NEAR(grid_minimum->distance_m, 5.0, kTolerance);
        EXPECT_NEAR(grid_minimum->pose_yaw_rad, 0.0, kTolerance);
        EXPECT_NEAR(grid_minimum->actual_metric.cost, 0.0, kTolerance);
    }

    TEST(PoseLandscapeAnalyzer, ClipsDistanceWindowAtPhysicalMinimum) {
        const PoseRefineInput input = makeSyntheticInput(1.2, 0.0, 0.0);
        PoseLandscapeParams params;
        params.enabled = true;
        PoseLandscapeAnalyzer analyzer(params);
        PoseLandscapeSampleInfo info;
        info.armor_type = "small";

        const debug::PoseLandscapeSample sample = analyzer.analyze(input, info);
        ASSERT_TRUE(sample.valid) << sample.status;
        EXPECT_NEAR(sample.requested_distance_min_m, 1.0, kTolerance);
        EXPECT_NEAR(sample.actual_distance_min_m, 1.0, kTolerance);
        EXPECT_NEAR(sample.actual_distance_max_m, 4.2, kTolerance);
        EXPECT_EQ(sample.distance_count, 65U);
        EXPECT_EQ(sample.pose_yaw_count, 71U);
        EXPECT_EQ(sample.grid.size(), 65U * 71U);
    }

} // namespace armor_detector::pose
