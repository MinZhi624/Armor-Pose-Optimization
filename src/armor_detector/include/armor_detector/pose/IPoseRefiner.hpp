#pragma once

#include "armor_detector/pose/PoseRefineData.hpp"

#include <Eigen/Core>

#include <functional>

namespace armor_detector::pose {

    using PoseErrorFunction = std::function<double(const Eigen::Vector3d &xyz_gimbal, double yaw_rad)>;

    class IPoseRefiner {
    public:
        virtual ~IPoseRefiner() = default;
        virtual PoseRefineOutput refine(const PoseRefineInput &input,
                                        const PoseErrorFunction &calculate_error) const = 0;
    };

} // namespace armor_detector::pose
