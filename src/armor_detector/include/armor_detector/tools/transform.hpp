#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace armor_detector::tools {

    // Camera 坐标系: X右 Y下 Z前
    // Gimbal 坐标系:  X前 Y左 Z上
    inline const Eigen::Matrix3d R_GIMBAL_CAMERA = (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();

    inline const Eigen::Matrix3d R_CAMERA_GIMBAL = R_GIMBAL_CAMERA.transpose();

    // 从 gimbal yaw/pitch 计算 world→gimbal 旋转
    inline Eigen::Matrix3d calculateRWorldGimbal(double yaw, double pitch) {
        const Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
        return R_yaw * R_pitch;
    }

} // namespace armor_detector::tools
