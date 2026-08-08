#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/pose/PoseLandscapeAnalyzer.hpp"
#include "armor_detector/pose/PoseRefineRunner.hpp"
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
        struct PosePerturbationParams {
            bool enabled = false;
            double dir_yaw_delta_rad = 0.0;
            double dir_pitch_delta_rad = 0.0;
            double distance_delta_m = 0.0;
            double pose_yaw_delta_rad = 0.0;
        };

        PoseSolver() = default;
        PoseSolver(const CameraInfo &camera_info);

        void init(const CameraInfo &camera_info);
        void setPosePerturbationParams(const PosePerturbationParams &params);
        void setPoseLandscapeParams(const pose::PoseLandscapeParams &params);

        std::vector<SolvedArmor> solve(const std::vector<DetectedArmor> &armors,
                                       const pose::PoseRefineRunner &pose_refiner);
        const debug::PoseDebugData &getPoseDebugData() const {
            return pose_debug_;
        }

    private:
        struct PnPCandidate {
            cv::Vec3d rvec = {0.0, 0.0, 0.0};
            cv::Vec3d tvec = {0.0, 0.0, 0.0};
            double yaw = 0.0;
            double world_pitch = 0.0;
            double reprojection_error = 0.0;
        };
        struct LastArmorYawRecord {
            double yaw = 0.0;
            cv::Point2f center;
        };


        SolvedArmor
        createSolvedArmorFromRvecTvec(const DetectedArmor &armor, const cv::Vec3d &rvec, const cv::Vec3d &tvec) const;
        std::vector<PnPCandidate> createPnPCandidates(const std::vector<cv::Point3f> &object_points,
                                                      const std::vector<cv::Point2f> &image_points) const;

        static std::size_t selectByGeometry(const std::vector<PnPCandidate> &candidates);
        static std::size_t selectByYawContinuity(const std::vector<PnPCandidate> &candidates, double nearest_yaw);
        std::size_t selectBestCandidate(const std::vector<PnPCandidate> &candidates,
                                        int armor_name_key,
                                        const cv::Point2f &target_center) const;

        double calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                          const std::vector<cv::Point2f> &image_points,
                                          const cv::Vec3d &rvec,
                                          const cv::Vec3d &tvec) const;

        static double calculateWorldPitchFromRvec(const cv::Vec3d &rvec);

        cv::Matx33d camera_matrix_ = cv::Matx33d::eye();
        cv::Vec<double, 5> distortion_coefficients_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        Eigen::Matrix3d R_gimbal_world_ = Eigen::Matrix3d::Identity();
        std::unordered_map<int, std::vector<LastArmorYawRecord>> record_;
        PosePerturbationParams pose_perturbation_params_;
        pose::PoseLandscapeAnalyzer pose_landscape_analyzer_;
        debug::PoseDebugData pose_debug_;
    };

} // namespace armor_detector
