#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/armor_geometry.hpp"
#include "armor_detector/tools/geometry.hpp"
#include "armor_detector/tools/transform.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <stdexcept>

namespace armor_detector::pose {

    namespace {
        cv::Mat distortionMat(const cv::Vec<double, 5> &distortion_coefficients) {
            cv::Mat distortion_mat(1, 5, CV_64F);
            for (int i = 0; i < 5; ++i) {
                distortion_mat.at<double>(0, i) = distortion_coefficients[i];
            }
            return distortion_mat;
        }
    } // namespace

    cv::Vec3d matToVec3d(const cv::Mat &mat) {
        if (mat.total() < 3) {
            throw std::invalid_argument("rvec/tvec mat must contain at least 3 elements");
        }

        cv::Mat mat64;
        mat.reshape(1, 1).convertTo(mat64, CV_64F);
        return {mat64.at<double>(0, 0), mat64.at<double>(0, 1), mat64.at<double>(0, 2)};
    }

    const std::vector<cv::Point3f> &objectPointsForArmor(ArmorType type) {
        return (type == ArmorType::LARGE) ? tools::LARGE_ARMOR_POINTS : tools::SMALL_ARMOR_POINTS;
    }

    Eigen::Matrix3d rotationMatrixFromRvec(const cv::Vec3d &rvec) {
        cv::Mat rmat_cv;
        cv::Rodrigues(rvec, rmat_cv);
        Eigen::Matrix3d rotation;
        cv::cv2eigen(rmat_cv, rotation);
        return rotation;
    }

    void rvecTvecFromGimbalXyzYaw(const Eigen::Vector3d &xyz_gimbal, double yaw_rad, cv::Vec3d &rvec, cv::Vec3d &tvec) {
        const auto R_pitch = Eigen::AngleAxisd(tools::ARMOR_PITCH_RAD, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const auto R_yaw = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d R_gimbal_armor = R_yaw * R_pitch;
        const Eigen::Matrix3d R_camera_armor = tools::R_CAMERA_GIMBAL * R_gimbal_armor;
        const Eigen::Vector3d xyz_camera = tools::R_CAMERA_GIMBAL * xyz_gimbal;

        cv::Mat R_cv;
        cv::eigen2cv(R_camera_armor, R_cv);
        cv::Rodrigues(R_cv, rvec);
        cv::eigen2cv(xyz_camera, tvec);
    }

    std::vector<cv::Point2f> projectArmor(const std::vector<cv::Point3f> &object_points,
                                          const cv::Vec3d &rvec,
                                          const cv::Vec3d &tvec,
                                          const cv::Matx33d &camera_matrix,
                                          const cv::Vec<double, 5> &distortion_coefficients) {
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(
            object_points, rvec, tvec, cv::Mat(camera_matrix), distortionMat(distortion_coefficients), image_points);
        return image_points;
    }

    std::vector<cv::Point2f> projectArmor(ArmorType type,
                                          const cv::Vec3d &rvec,
                                          const cv::Vec3d &tvec,
                                          const cv::Matx33d &camera_matrix,
                                          const cv::Vec<double, 5> &distortion_coefficients) {
        return projectArmor(objectPointsForArmor(type), rvec, tvec, camera_matrix, distortion_coefficients);
    }

    double calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                      const std::vector<cv::Point2f> &image_points,
                                      const cv::Vec3d &rvec,
                                      const cv::Vec3d &tvec,
                                      const cv::Matx33d &camera_matrix,
                                      const cv::Vec<double, 5> &distortion_coefficients) {
        const auto projected = projectArmor(object_points, rvec, tvec, camera_matrix, distortion_coefficients);
        double error = 0.0;
        const std::size_t count = std::min(projected.size(), image_points.size());
        for (std::size_t i = 0; i < count; ++i) {
            error += cv::norm(projected[i] - image_points[i]);
        }
        return error;
    }

    double calculateReprojectionError(ArmorType type,
                                      const std::array<cv::Point2f, 4> &image_corners,
                                      const cv::Vec3d &rvec,
                                      const cv::Vec3d &tvec,
                                      const cv::Matx33d &camera_matrix,
                                      const cv::Vec<double, 5> &distortion_coefficients) {
        const std::vector<cv::Point2f> image_points(image_corners.begin(), image_corners.end());
        return calculateReprojectionError(
            objectPointsForArmor(type), image_points, rvec, tvec, camera_matrix, distortion_coefficients);
    }

    ArmorPose calculateArmorPose(const cv::Vec3d &rvec,
                                 const cv::Vec3d &tvec,
                                 const std::array<cv::Point2f, 4> &image_corners,
                                 const cv::Matx33d &camera_matrix) {
        const Eigen::Matrix3d R_camera_armor = rotationMatrixFromRvec(rvec);
        const Eigen::Vector3d xyz_camera(tvec[0], tvec[1], tvec[2]);

        ArmorPose pose;
        pose.xyz_camera = xyz_camera;
        pose.xyz_gimbal = tools::R_GIMBAL_CAMERA * xyz_camera;
        pose.ypr_camera = tools::calculateYPR(R_camera_armor);

        const Eigen::Matrix3d R_gimbal_armor = tools::R_GIMBAL_CAMERA * R_camera_armor;
        pose.ypr_gimbal = tools::calculateYPR(R_gimbal_armor);
        pose.ypd_gimbal = tools::calculateYPD(pose.xyz_gimbal);

        const double yaw = std::atan2(xyz_camera.x(), xyz_camera.z());
        const double pitch = std::atan2(-xyz_camera.y(), std::hypot(xyz_camera.x(), xyz_camera.z()));
        const double distance = xyz_camera.norm();
        pose.ypd_camera = {yaw, pitch, distance};

        const cv::Point2f target_center =
            (image_corners[0] + image_corners[1] + image_corners[2] + image_corners[3]) / 4.0f;
        const cv::Point2f image_center(static_cast<float>(camera_matrix(0, 2)),
                                       static_cast<float>(camera_matrix(1, 2)));
        pose.image_distance_to_center = static_cast<float>(cv::norm(image_center - target_center));
        return pose;
    }

} // namespace armor_detector::pose
