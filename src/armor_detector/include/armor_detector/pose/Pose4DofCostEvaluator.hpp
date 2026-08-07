#pragma once

#include <Eigen/Core>
#include <array>
#include <ceres/cost_function.h>
#include <cstddef>
#include <limits>
#include <string>

#include "armor_detector/pose/PoseRefineData.hpp"

namespace armor_detector::pose {

    inline constexpr double kPose4DofHuberLossScalePx = 3.0;

    enum class Pose4DofParameterization { XYZ, YPD };

    /**
     * @brief 4DoF 模型共享的、去畸变后的单帧观测。
     *
     * observed_normalized 必须由与 BA 相同的 K 和畸变参数生成；它可同时供
     * Ceres residual 和无迭代 objective evaluator 使用。
     */
    struct Pose4DofObservation {
        ArmorType armor_type = ArmorType::NONE;
        std::array<cv::Point3d, 4> object_points{};
        std::array<cv::Point2d, 4> observed_normalized{};
        double fx = 0.0;
        double fy = 0.0;
        bool valid = false;
        std::string status = "not_initialized";
    };

    struct Pose4DofCostEvaluation {
        bool valid = false;
        std::string status = "not_evaluated";
        double cost = std::numeric_limits<double>::quiet_NaN();
        double mean_residual_px = std::numeric_limits<double>::quiet_NaN();
        std::array<double, 4> corner_residual_px{std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN(),
                                                 std::numeric_limits<double>::quiet_NaN()};
    };

    Pose4DofObservation createPose4DofObservation(const PoseRefineInput &input);

    Pose4DofCostEvaluation evaluatePose4DofCost(const Pose4DofObservation &observation,
                                                const Eigen::Vector3d &xyz_gimbal,
                                                double pose_yaw_rad);

    Pose4DofCostEvaluation evaluatePose4DofYpdCost(const Pose4DofObservation &observation,
                                                   const Eigen::Vector3d &ypd_gimbal,
                                                   double pose_yaw_rad);

    // Creates a Ceres residual with the same projection formula as evaluatePose4Dof*.
    // ceres::Problem owns the returned cost; callers create the loss for each block.
    ceres::CostFunction *createPose4DofReprojectionCostFunction(const Pose4DofObservation &observation,
                                                                std::size_t corner_index,
                                                                Pose4DofParameterization parameterization);

    /**
     * @brief 创建固定角度的一维 distance Ceres residual。
     *
     * 唯一参数块为 distance（m）。
     */
    ceres::CostFunction *createPose4DofDistanceReprojectionCostFunction(const Pose4DofObservation &observation,
                                                                        std::size_t corner_index,
                                                                        double dir_yaw_rad,
                                                                        double dir_pitch_rad,
                                                                        double pose_yaw_rad);

    /**
     * @brief 创建共享 yaw、独立 distance 的双装甲板 reprojection residual。
     *
     * 参数块依次为 shared yaw 和 distance；方向 yaw、方向 pitch 以及 yaw offset 均为固定值。
     */
    ceres::CostFunction *createPose4DofSharedYawDistanceReprojectionCostFunction(const Pose4DofObservation &observation,
                                                                                 std::size_t corner_index,
                                                                                 double dir_yaw_rad,
                                                                                 double dir_pitch_rad,
                                                                                 double yaw_offset_rad);

    ceres::CostFunction *createPose4DofSharedYawXyzReprojectionCostFunction(const Pose4DofObservation &observation,
                                                                            std::size_t corner_index,
                                                                            double yaw_offset_rad);

} // namespace armor_detector::pose
