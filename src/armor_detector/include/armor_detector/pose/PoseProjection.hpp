#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <array>
#include <vector>

namespace armor_detector::pose {

    cv::Vec3d matToVec3d(const cv::Mat &mat);

    const std::vector<cv::Point3f> &objectPointsForArmor(ArmorType type);

    Eigen::Matrix3d rotationMatrixFromRvec(const cv::Vec3d &rvec);

    void rvecTvecFromGimbalXyzYaw(const Eigen::Vector3d &xyz_gimbal, double yaw_rad, cv::Vec3d &rvec, cv::Vec3d &tvec);

    std::vector<cv::Point2f> projectArmor(const std::vector<cv::Point3f> &object_points,
                                          const cv::Vec3d &rvec,
                                          const cv::Vec3d &tvec,
                                          const cv::Matx33d &camera_matrix,
                                          const cv::Vec<double, 5> &distortion_coefficients);

    std::vector<cv::Point2f> projectArmor(ArmorType type,
                                          const cv::Vec3d &rvec,
                                          const cv::Vec3d &tvec,
                                          const cv::Matx33d &camera_matrix,
                                          const cv::Vec<double, 5> &distortion_coefficients);

    double calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                      const std::vector<cv::Point2f> &image_points,
                                      const cv::Vec3d &rvec,
                                      const cv::Vec3d &tvec,
                                      const cv::Matx33d &camera_matrix,
                                      const cv::Vec<double, 5> &distortion_coefficients);

    double calculateReprojectionError(ArmorType type,
                                      const std::array<cv::Point2f, 4> &image_corners,
                                      const cv::Vec3d &rvec,
                                      const cv::Vec3d &tvec,
                                      const cv::Matx33d &camera_matrix,
                                      const cv::Vec<double, 5> &distortion_coefficients);

    ArmorPose calculateArmorPose(const cv::Vec3d &rvec,
                                 const cv::Vec3d &tvec,
                                 const std::array<cv::Point2f, 4> &image_corners,
                                 const cv::Matx33d &camera_matrix);

} // namespace armor_detector::pose
