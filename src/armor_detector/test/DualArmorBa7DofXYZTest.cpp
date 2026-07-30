#include <cmath>

#include <gtest/gtest.h>

#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/pose/PoseRefineRunner.hpp"
#include "armor_detector/tools/angle.hpp"

namespace armor_detector::pose {
    namespace {
        constexpr double kPoseToleranceRad = 1e-4;
        constexpr double kXyzToleranceM = 1e-4;

        PoseRefineInput makeArmor(double target_yaw_rad,
                                  double initial_yaw_rad,
                                  const Eigen::Vector3d &target_xyz,
                                  const Eigen::Vector3d &initial_xyz,
                                  ArmorName name = ArmorName::ONE) {
            PoseRefineInput input;
            input.armor_name = name;
            input.armor_type = ArmorType::SMALL;
            input.camera_matrix = cv::Matx33d(800.0, 0.0, 640.0, 0.0, 810.0, 512.0, 0.0, 0.0, 1.0);
            cv::Vec3d target_rvec;
            cv::Vec3d target_tvec;
            rvecTvecFromGimbalXyzYaw(target_xyz, target_yaw_rad, target_rvec, target_tvec);
            const auto projected = projectArmor(
                input.armor_type, target_rvec, target_tvec, input.camera_matrix, input.distortion_coefficients);
            for (std::size_t i = 0; i < input.image_corners.size(); ++i) {
                input.image_corners[i] = projected[i];
            }
            rvecTvecFromGimbalXyzYaw(initial_xyz, initial_yaw_rad, input.initial_rvec, input.initial_tvec);
            return input;
        }

        PoseRefineRunner runner() {
            PoseRefineRunner value;
            value.setSingleMethod(SinglePoseRefineMethod::NONE);
            value.setDualMethod(DualPoseRefineMethod::DUAL_ARMOR_BA_7DOF_XYZ);
            return value;
        }

        void expectRecovered(const PoseRefineOutput &output, const Eigen::Vector3d &expected_xyz) {
            const ArmorPose pose = calculateArmorPose(output.rvec, output.tvec, {}, cv::Matx33d::eye());
            EXPECT_NEAR(pose.xyz_gimbal.x(), expected_xyz.x(), kXyzToleranceM);
            EXPECT_NEAR(pose.xyz_gimbal.y(), expected_xyz.y(), kXyzToleranceM);
            EXPECT_NEAR(pose.xyz_gimbal.z(), expected_xyz.z(), kXyzToleranceM);
        }
    } // namespace

    TEST(DualArmorBa7DofXYZ, RecoversIndependentGimbalTranslationsAndPositiveQuarterTurn) {
        constexpr double yaw = 0.35;
        const Eigen::Vector3d xyz_a(4.7, 0.6, -0.2);
        const Eigen::Vector3d xyz_b(5.1, -0.8, 0.3);
        const auto armor_a = makeArmor(yaw, yaw + 0.15, xyz_a, xyz_a + Eigen::Vector3d(0.3, -0.2, 0.2));
        const auto armor_b =
            makeArmor(yaw + M_PI_2, yaw + M_PI_2 - 0.12, xyz_b, xyz_b + Eigen::Vector3d(-0.25, 0.2, -0.15));
        const auto output = runner().refine({armor_a, armor_b});

        ASSERT_TRUE(output.dual_summary.has_value());
        ASSERT_TRUE(output.items[0].success);
        ASSERT_TRUE(output.items[1].success);
        expectRecovered(output.items[0], xyz_a);
        expectRecovered(output.items[1], xyz_b);
        const ArmorPose final_a = calculateArmorPose(
            output.items[0].rvec, output.items[0].tvec, armor_a.image_corners, armor_a.camera_matrix);
        const ArmorPose final_b = calculateArmorPose(
            output.items[1].rvec, output.items[1].tvec, armor_b.image_corners, armor_b.camera_matrix);
        EXPECT_NEAR(
            tools::shortestAngularDistance(yaw, output.dual_summary->shared_pose_yaw_rad), 0.0, kPoseToleranceRad);
        EXPECT_NEAR(tools::shortestAngularDistance(final_a.ypr_gimbal.x(), final_b.ypr_gimbal.x()), M_PI_2, 1e-9);
        EXPECT_LT(output.dual_summary->mean_reprojection_error_px, 1e-3);
    }

    TEST(DualArmorBa7DofXYZ, SelectsNegativeQuarterTurn) {
        constexpr double yaw = -0.4;
        const auto armor_a = makeArmor(yaw, yaw + 0.1, {4.8, 0.4, 0.1}, {5.1, 0.2, 0.2});
        const auto armor_b = makeArmor(yaw - M_PI_2, yaw - M_PI_2 - 0.1, {5.0, -0.5, -0.1}, {4.8, -0.3, 0.0});
        const auto output = runner().refine({armor_a, armor_b});

        ASSERT_TRUE(output.dual_summary.has_value());
        const ArmorPose final_a = calculateArmorPose(
            output.items[0].rvec, output.items[0].tvec, armor_a.image_corners, armor_a.camera_matrix);
        const ArmorPose final_b = calculateArmorPose(
            output.items[1].rvec, output.items[1].tvec, armor_b.image_corners, armor_b.camera_matrix);
        EXPECT_NEAR(
            tools::shortestAngularDistance(yaw, output.dual_summary->shared_pose_yaw_rad), 0.0, kPoseToleranceRad);
        EXPECT_NEAR(tools::shortestAngularDistance(final_a.ypr_gimbal.x(), final_b.ypr_gimbal.x()), -M_PI_2, 1e-9);
    }

    TEST(DualArmorBa7DofXYZ, RegistersMethodString) {
        EXPECT_EQ(dualPoseRefineMethodFromString("dual_armor_ba_7dof_xyz"),
                  DualPoseRefineMethod::DUAL_ARMOR_BA_7DOF_XYZ);
        EXPECT_EQ(toString(DualPoseRefineMethod::DUAL_ARMOR_BA_7DOF_XYZ), "dual_armor_ba_7dof_xyz");
    }
} // namespace armor_detector::pose
