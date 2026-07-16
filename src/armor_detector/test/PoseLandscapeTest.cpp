#include <cmath>
#include <gtest/gtest.h>

#include "armor_detector/pose/Pose4DofCostEvaluator.hpp"
#include "armor_detector/pose/PoseLandscapeAnalyzer.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofXYZRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofYPDRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

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
    } // namespace

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

            const PoseOnlyBa4DofYPDRefiner ypd_refiner;
            const PoseRefineOutput ypd_output = ypd_refiner.refine(input);
            ASSERT_TRUE(ypd_output.solver_summary.available);
            EXPECT_NEAR(ypd_evaluation.cost, ypd_output.solver_summary.initial_cost, kTolerance);

            const PoseOnlyBa4DofXYZRefiner xyz_refiner;
            const PoseRefineOutput xyz_output = xyz_refiner.refine(input);
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
