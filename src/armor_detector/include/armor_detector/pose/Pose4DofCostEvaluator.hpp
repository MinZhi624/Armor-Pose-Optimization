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

    /**
     * @brief 创建与 evaluatePose4Dof* 使用同一投影公式的 Ceres residual。
     *
     * 返回值所有权交给 ceres::Problem；调用方仍负责为每个 block 创建 loss。
     */
    ceres::CostFunction *createPose4DofReprojectionCostFunction(const Pose4DofObservation &observation,
                                                                std::size_t corner_index,
                                                                Pose4DofParameterization parameterization);

} // namespace armor_detector::pose
