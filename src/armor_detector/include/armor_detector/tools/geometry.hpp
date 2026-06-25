#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace armor_detector::tools {

    // 旋转矩阵/四元数 → (yaw, pitch, roll) 弧度
    inline Eigen::Vector3d calculateYPR(const Eigen::Quaterniond &q) {
        const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        const double yaw = std::atan2(siny_cosp, cosy_cosp);

        const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
        const double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

        const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
        const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        const double roll = std::atan2(sinr_cosp, cosr_cosp);

        return {yaw, pitch, roll};
    }

    inline Eigen::Vector3d calculateYPR(const Eigen::Matrix3d &R) {
        return calculateYPR(Eigen::Quaterniond(R));
    }

    // gimbal 坐标 → (yaw, pitch, distance)
    inline Eigen::Vector3d calculateYPD(const Eigen::Vector3d &xyz) {
        const double yaw = std::atan2(xyz.y(), xyz.x());
        const double pitch = std::atan2(xyz.z(), std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y()));
        const double distance = xyz.norm();
        return {yaw, pitch, distance};
    }

    // calculateYPD 的逆变换
    inline Eigen::Vector3d calculateXYZ(const Eigen::Vector3d &ypd) {
        const double yaw = ypd.x();
        const double pitch = ypd.y();
        const double dist = ypd.z();
        const double cos_p = std::cos(pitch);
        return {dist * cos_p * std::cos(yaw), dist * cos_p * std::sin(yaw), dist * std::sin(pitch)};
    }

} // namespace armor_detector::tools
