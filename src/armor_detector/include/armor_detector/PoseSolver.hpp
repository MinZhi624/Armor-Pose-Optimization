#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/types/ArmorData.hpp"
#include "armor_detector/types/CameraInfo.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <unordered_map>
#include <vector>

namespace armor_detector {

    class PoseSolver {
    public:
        PoseSolver() = default;
        PoseSolver(const CameraInfo &camera_info);

        void init(const CameraInfo &camera_info);

        std::vector<SolvedArmor> solve(const std::vector<ClassifiedArmor> &armors);

        const debug::PoseDebugData &getPoseDebugData() const {
            return pose_debug_;
        }

    private:
        struct LastArmorYawRecord {
            double yaw = 0.0;
            cv::Point2f center;
        };

        struct PnPCandidate {
            cv::Mat rvec;
            cv::Mat tvec;
            double yaw = 0.0;
            double world_pitch = 0.0;
            double reprojection_error = 0.0;
        };

        std::vector<PnPCandidate> createPnPCandidates(const std::vector<cv::Point3f> &object_points,
                                                      const std::vector<cv::Point2f> &image_points) const;

        static std::size_t selectByGeometry(const std::vector<PnPCandidate> &candidates);
        static std::size_t selectByYawContinuity(const std::vector<PnPCandidate> &candidates, double nearest_yaw);
        std::size_t selectBestCandidate(const std::vector<PnPCandidate> &candidates,
                                        int armor_name_key,
                                        const cv::Point2f &target_center) const;

        double calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                          const std::vector<cv::Point2f> &image_points,
                                          const cv::Mat &rvec,
                                          const cv::Mat &tvec) const;

        static double calculateWorldPitchFromRvec(const cv::Mat &rvec);

        void optimizeYaw(SolvedArmor &armor, const cv::Point2f &target_center);

        std::vector<cv::Point2f> reprojectArmor(const Eigen::Vector3d &xyz_gimbal, double yaw, ArmorType type) const;

        cv::Mat camera_matrix_;
        cv::Mat distortion_coefficients_;

        Eigen::Matrix3d R_gimbal_world_ = Eigen::Matrix3d::Identity();

        std::unordered_map<int, std::vector<LastArmorYawRecord>> record_;

        debug::PoseDebugData pose_debug_;
    };

} // namespace armor_detector
