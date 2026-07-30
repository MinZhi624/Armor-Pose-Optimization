#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/pose/PoseRefineRunner.hpp"
#include "armor_detector/tools/angle.hpp"

namespace armor_detector::pose {
    namespace {
        constexpr double kHardConstraintToleranceRad = 1e-9;
        constexpr double kPoseToleranceRad = 1e-4;
        constexpr double kDistanceToleranceM = 1e-4;
        constexpr double kMeanErrorTolerancePx = 1e-3;

        PoseRefineInput makeSyntheticArmor(ArmorName armor_name,
                                           double dir_yaw_rad,
                                           double dir_pitch_rad,
                                           double initial_distance_m,
                                           double target_distance_m,
                                           double initial_pose_yaw_rad,
                                           double target_pose_yaw_rad) {
            PoseRefineInput input;
            input.armor_name = armor_name;
            input.armor_type = ArmorType::SMALL;
            input.camera_matrix = cv::Matx33d(800.0, 0.0, 640.0, 0.0, 810.0, 512.0, 0.0, 0.0, 1.0);
            input.distortion_coefficients = {0.0, 0.0, 0.0, 0.0, 0.0};

            const Eigen::Vector3d target_ypd_gimbal(dir_yaw_rad, dir_pitch_rad, target_distance_m);
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

            const Eigen::Vector3d initial_ypd_gimbal(dir_yaw_rad, dir_pitch_rad, initial_distance_m);
            rvecTvecFromGimbalYpdYaw(initial_ypd_gimbal, initial_pose_yaw_rad, input.initial_rvec, input.initial_tvec);
            return input;
        }

        PoseRefineRunner makeRunner(DualPoseRefineMethod dual_method) {
            PoseRefineRunner runner;
            runner.setSingleMethod(SinglePoseRefineMethod::NONE);
            runner.setDualMethod(dual_method);
            return runner;
        }

        void expectSamePnpPose(const PoseRefineInput &input, const PoseRefineOutput &output) {
            for (int i = 0; i < 3; ++i) {
                EXPECT_DOUBLE_EQ(output.rvec[i], input.initial_rvec[i]);
                EXPECT_DOUBLE_EQ(output.tvec[i], input.initial_tvec[i]);
            }
            EXPECT_TRUE(output.success);
        }

        struct SyntheticPair {
            PoseRefineInput armor_a;
            PoseRefineInput armor_b;
            double target_pose_yaw_rad = 0.0;
            double target_distance_a_m = 0.0;
            double target_distance_b_m = 0.0;
        };

        SyntheticPair makeSyntheticPair(double target_pose_yaw_rad = 0.35) {
            SyntheticPair pair;
            pair.target_pose_yaw_rad = target_pose_yaw_rad;
            pair.target_distance_a_m = 4.8;
            pair.target_distance_b_m = 5.2;
            pair.armor_a = makeSyntheticArmor(ArmorName::ONE,
                                              0.15,
                                              -0.05,
                                              4.2,
                                              pair.target_distance_a_m,
                                              target_pose_yaw_rad + 0.12,
                                              target_pose_yaw_rad);
            pair.armor_b = makeSyntheticArmor(ArmorName::ONE,
                                              -0.15,
                                              -0.05,
                                              5.8,
                                              pair.target_distance_b_m,
                                              target_pose_yaw_rad + M_PI_2 - 0.12,
                                              target_pose_yaw_rad + M_PI_2);
            return pair;
        }
    } // namespace

    TEST(DualArmorBa3DofYPD, RecoversSharedYawAndDistancesThroughBatchRunner) {
        const SyntheticPair pair = makeSyntheticPair();
        PoseRefineRunner runner = makeRunner(DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD);

        const PoseRefineBatchOutput output = runner.refine({pair.armor_a, pair.armor_b});

        ASSERT_EQ(output.items.size(), 2U);
        ASSERT_TRUE(output.dual_summary.has_value());
        const auto &summary = *output.dual_summary;
        EXPECT_EQ(summary.armor_a_index, 0U);
        EXPECT_EQ(summary.armor_b_index, 1U);
        EXPECT_NEAR(tools::shortestAngularDistance(pair.target_pose_yaw_rad, summary.shared_pose_yaw_rad),
                    0.0,
                    kPoseToleranceRad);
        EXPECT_NEAR(summary.distance_a_m, pair.target_distance_a_m, kDistanceToleranceM);
        EXPECT_NEAR(summary.distance_b_m, pair.target_distance_b_m, kDistanceToleranceM);
        EXPECT_LT(summary.mean_reprojection_error_px, kMeanErrorTolerancePx);
        ASSERT_TRUE(summary.solver_summary.available);

        const ArmorPose initial_pose_a = calculateArmorPose(pair.armor_a.initial_rvec,
                                                            pair.armor_a.initial_tvec,
                                                            pair.armor_a.image_corners,
                                                            pair.armor_a.camera_matrix);
        const ArmorPose initial_pose_b = calculateArmorPose(pair.armor_b.initial_rvec,
                                                            pair.armor_b.initial_tvec,
                                                            pair.armor_b.image_corners,
                                                            pair.armor_b.camera_matrix);
        const ArmorPose final_pose_a = calculateArmorPose(
            output.items[0].rvec, output.items[0].tvec, pair.armor_a.image_corners, pair.armor_a.camera_matrix);
        const ArmorPose final_pose_b = calculateArmorPose(
            output.items[1].rvec, output.items[1].tvec, pair.armor_b.image_corners, pair.armor_b.camera_matrix);
        EXPECT_NEAR(tools::shortestAngularDistance(final_pose_a.ypr_gimbal.x(), final_pose_b.ypr_gimbal.x()),
                    M_PI_2,
                    kHardConstraintToleranceRad);
        EXPECT_NEAR(final_pose_a.ypd_gimbal.x(), initial_pose_a.ypd_gimbal.x(), kHardConstraintToleranceRad);
        EXPECT_NEAR(final_pose_a.ypd_gimbal.y(), initial_pose_a.ypd_gimbal.y(), kHardConstraintToleranceRad);
        EXPECT_NEAR(final_pose_b.ypd_gimbal.x(), initial_pose_b.ypd_gimbal.x(), kHardConstraintToleranceRad);
        EXPECT_NEAR(final_pose_b.ypd_gimbal.y(), initial_pose_b.ypd_gimbal.y(), kHardConstraintToleranceRad);
    }

    TEST(DualArmorBa3DofYPD, UsesImagePositionInsteadOfInputListOrder) {
        const SyntheticPair pair = makeSyntheticPair();
        PoseRefineRunner runner = makeRunner(DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD);

        const PoseRefineBatchOutput output = runner.refine({pair.armor_b, pair.armor_a});

        ASSERT_EQ(output.items.size(), 2U);
        ASSERT_TRUE(output.dual_summary.has_value());
        const auto &summary = *output.dual_summary;
        EXPECT_EQ(summary.armor_a_index, 1U);
        EXPECT_EQ(summary.armor_b_index, 0U);
        EXPECT_NEAR(tools::shortestAngularDistance(pair.target_pose_yaw_rad, summary.shared_pose_yaw_rad),
                    0.0,
                    kPoseToleranceRad);
        EXPECT_NEAR(summary.distance_a_m, pair.target_distance_a_m, kDistanceToleranceM);
        EXPECT_NEAR(summary.distance_b_m, pair.target_distance_b_m, kDistanceToleranceM);
    }

    TEST(DualArmorBa3DofYPD, UsesCircularMeanAcrossAngleBoundary) {
        const SyntheticPair pair = makeSyntheticPair(3.10);
        PoseRefineRunner runner = makeRunner(DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD);

        const PoseRefineBatchOutput output = runner.refine({pair.armor_a, pair.armor_b});

        ASSERT_TRUE(output.dual_summary.has_value());
        EXPECT_NEAR(tools::shortestAngularDistance(pair.target_pose_yaw_rad, output.dual_summary->shared_pose_yaw_rad),
                    0.0,
                    kPoseToleranceRad);
    }

    TEST(DualArmorBa3DofYPD, SelectsNegativeQuarterTurn) {
        constexpr double target_yaw_rad = -0.4;
        const PoseRefineInput armor_a =
            makeSyntheticArmor(ArmorName::ONE, 0.15, -0.05, 4.2, 4.8, target_yaw_rad + 0.12, target_yaw_rad);
        const PoseRefineInput armor_b = makeSyntheticArmor(
            ArmorName::ONE, -0.15, -0.05, 5.8, 5.2, target_yaw_rad - M_PI_2 - 0.12, target_yaw_rad - M_PI_2);
        PoseRefineRunner runner = makeRunner(DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD);

        const PoseRefineBatchOutput output = runner.refine({armor_a, armor_b});

        ASSERT_TRUE(output.dual_summary.has_value());
        const ArmorPose final_pose_a = calculateArmorPose(
            output.items[0].rvec, output.items[0].tvec, armor_a.image_corners, armor_a.camera_matrix);
        const ArmorPose final_pose_b = calculateArmorPose(
            output.items[1].rvec, output.items[1].tvec, armor_b.image_corners, armor_b.camera_matrix);
        EXPECT_NEAR(tools::shortestAngularDistance(target_yaw_rad, output.dual_summary->shared_pose_yaw_rad),
                    0.0,
                    kPoseToleranceRad);
        EXPECT_NEAR(tools::shortestAngularDistance(final_pose_a.ypr_gimbal.x(), final_pose_b.ypr_gimbal.x()),
                    -M_PI_2,
                    kHardConstraintToleranceRad);
    }

    TEST(DualArmorBa3DofYPD, DelegatesWhenDualIsDisabledOrPairIsIneligible) {
        SyntheticPair pair = makeSyntheticPair();
        pair.armor_b.armor_name = ArmorName::TWO;
        PoseRefineRunner runner = makeRunner(DualPoseRefineMethod::NONE);

        const PoseRefineBatchOutput disabled_output = runner.refine({pair.armor_a, pair.armor_b});
        ASSERT_EQ(disabled_output.items.size(), 2U);
        EXPECT_FALSE(disabled_output.dual_summary.has_value());
        expectSamePnpPose(pair.armor_a, disabled_output.items[0]);
        expectSamePnpPose(pair.armor_b, disabled_output.items[1]);

        runner.setDualMethod(DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD);
        const PoseRefineBatchOutput ineligible_output = runner.refine({pair.armor_a, pair.armor_b});
        ASSERT_EQ(ineligible_output.items.size(), 2U);
        EXPECT_FALSE(ineligible_output.dual_summary.has_value());
        expectSamePnpPose(pair.armor_a, ineligible_output.items[0]);
        expectSamePnpPose(pair.armor_b, ineligible_output.items[1]);
    }

    TEST(DualArmorBa3DofYPD, AtomicallyFallsBackToPnpWhenPairCannotBeProjected) {
        SyntheticPair pair = makeSyntheticPair();
        pair.armor_a.camera_matrix(0, 0) = 0.0;
        PoseRefineRunner runner = makeRunner(DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD);

        const PoseRefineBatchOutput output = runner.refine({pair.armor_a, pair.armor_b});

        ASSERT_EQ(output.items.size(), 2U);
        EXPECT_FALSE(output.dual_summary.has_value());
        expectSamePnpPose(pair.armor_a, output.items[0]);
        expectSamePnpPose(pair.armor_b, output.items[1]);
    }

    TEST(DualArmorBa3DofYPD, UnknownDualMethodSafelyDisablesJointOptimization) {
        EXPECT_EQ(dualPoseRefineMethodFromString("not-a-method"), DualPoseRefineMethod::NONE);
        EXPECT_EQ(toString(dualPoseRefineMethodFromString("not-a-method")), "none");
    }

} // namespace armor_detector::pose
