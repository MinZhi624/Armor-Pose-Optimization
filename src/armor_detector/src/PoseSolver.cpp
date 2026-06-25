#include "armor_detector/PoseSolver.hpp"
#include "armor_detector/tools/angle.hpp"
#include "armor_detector/tools/armor_geometry.hpp"
#include "armor_detector/tools/geometry.hpp"
#include "armor_detector/tools/transform.hpp"
#include "armor_detector/yaw/YawSearch.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace armor_detector {

    using tools::ARMOR_PITCH_RAD;
    using tools::LARGE_ARMOR_POINTS;
    using tools::normalizeRadAngle;
    using tools::R_CAMERA_GIMBAL;
    using tools::R_GIMBAL_CAMERA;
    using tools::SMALL_ARMOR_POINTS;

    static constexpr double SAME_ARMOR_CENTER_THRESH = 30.0;
    static constexpr double YAW_MUTATION_THRESH = M_PI_2;
    static constexpr double REPROJECTION_ERROR_MARGIN = 3.0;
    static constexpr double MIN_VALID_ARMOR_PITCH_WORLD = -0.05;

    // ===================== 构造 / 初始化 =====================

    PoseSolver::PoseSolver(const CameraInfo &camera_info) {
        init(camera_info);
    }

    void PoseSolver::init(const CameraInfo &camera_info) {
        camera_matrix_ = cv::Mat(cv::Matx33d(camera_info.camera_matrix));
        distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
        for (int i = 0; i < 5; ++i) {
            distortion_coefficients_.at<double>(0, i) = camera_info.distortion_coefficients[i];
        }
    }

    // ===================== 主流程 =====================

    std::vector<SolvedArmor> PoseSolver::solve(const std::vector<ClassifiedArmor> &armors) {
        std::unordered_map<int, std::vector<LastArmorYawRecord>> new_record;
        std::vector<SolvedArmor> solved_armors;
        solved_armors.reserve(armors.size());

        for (std::size_t i = 0; i < armors.size(); ++i) {
            const auto &classified = armors[i];

            if (classified.classification.name == ArmorName::NONE || classified.geometry.type == ArmorType::NONE) {
                continue;
            }

            const auto &object_points =
                (classified.geometry.type == ArmorType::LARGE) ? LARGE_ARMOR_POINTS : SMALL_ARMOR_POINTS;

            const std::vector<cv::Point2f> image_points(classified.geometry.corners.begin(),
                                                        classified.geometry.corners.end());

            cv::Point2f target_center = (image_points[0] + image_points[1] + image_points[2] + image_points[3]) / 4.0f;

            auto candidates = createPnPCandidates(object_points, image_points);
            if (candidates.empty()) {
                continue;
            }

            const int armor_name_key = static_cast<int>(classified.classification.name);
            std::size_t best_id = selectBestCandidate(candidates, armor_name_key, target_center);

            cv::Mat rmat;
            cv::Rodrigues(candidates[best_id].rvec, rmat);

            Eigen::Matrix3d R_camera_armor;
            Eigen::Vector3d xyz_camera;
            cv::cv2eigen(rmat, R_camera_armor);
            cv::cv2eigen(candidates[best_id].tvec, xyz_camera);

            SolvedArmor solved;
            solved.geometry = classified.geometry;
            solved.classification = classified.classification;

            solved.pose.xyz_camera = xyz_camera;
            solved.pose.xyz_gimbal = R_GIMBAL_CAMERA * xyz_camera;
            solved.pose.ypr_camera = tools::calculateYPR(R_camera_armor);

            Eigen::Matrix3d R_gimbal_armor = R_GIMBAL_CAMERA * R_camera_armor;
            solved.pose.ypr_gimbal = tools::calculateYPR(R_gimbal_armor);

            solved.pose.ypd_gimbal = tools::calculateYPD(solved.pose.xyz_gimbal);

            {
                double yaw = std::atan2(xyz_camera.x(), xyz_camera.z());
                double pitch = std::atan2(-xyz_camera.y(), std::hypot(xyz_camera.x(), xyz_camera.z()));
                double distance = xyz_camera.norm();
                solved.pose.ypd_camera = {yaw, pitch, distance};
            }

            {
                double cx = camera_matrix_.at<double>(0, 2);
                double cy = camera_matrix_.at<double>(1, 2);
                solved.pose.image_distance_to_center =
                    static_cast<float>(cv::norm(cv::Point2f(cx, cy) - target_center));
            }

            optimizeYaw(solved, target_center);

            new_record[armor_name_key].push_back({solved.pose.ypr_gimbal.x(), target_center});

            solved_armors.push_back(std::move(solved));
        }

        record_ = std::move(new_record);
        pose_debug_.solved_armors = solved_armors;
        return solved_armors;
    }

    // ===================== PnP 候选 =====================

    std::vector<PoseSolver::PnPCandidate>
    PoseSolver::createPnPCandidates(const std::vector<cv::Point3f> &object_points,
                                    const std::vector<cv::Point2f> &image_points) const {

        std::vector<cv::Mat> ippe_rvecs, ippe_tvecs;
        cv::solvePnPGeneric(object_points,
                            image_points,
                            camera_matrix_,
                            distortion_coefficients_,
                            ippe_rvecs,
                            ippe_tvecs,
                            false,
                            cv::SOLVEPNP_IPPE);

        std::vector<PnPCandidate> candidates;
        candidates.reserve(ippe_rvecs.size());
        for (std::size_t j = 0; j < ippe_rvecs.size(); ++j) {
            PnPCandidate c;
            c.rvec = ippe_rvecs[j];
            c.tvec = ippe_tvecs[j];

            cv::Mat rmat;
            cv::Rodrigues(c.rvec, rmat);
            Eigen::Matrix3d R;
            cv::cv2eigen(rmat, R);
            c.yaw = tools::calculateYPR(R).x();
            c.world_pitch = calculateWorldPitchFromRvec(c.rvec);
            c.reprojection_error = calculateReprojectionError(object_points, image_points, c.rvec, c.tvec);
            candidates.push_back(c);
        }

        if (candidates.empty()) {
            PnPCandidate c;
            cv::solvePnP(object_points,
                         image_points,
                         camera_matrix_,
                         distortion_coefficients_,
                         c.rvec,
                         c.tvec,
                         false,
                         cv::SOLVEPNP_ITERATIVE);
            cv::Mat rmat;
            cv::Rodrigues(c.rvec, rmat);
            Eigen::Matrix3d R;
            cv::cv2eigen(rmat, R);
            c.yaw = tools::calculateYPR(R).x();
            c.world_pitch = calculateWorldPitchFromRvec(c.rvec);
            c.reprojection_error = calculateReprojectionError(object_points, image_points, c.rvec, c.tvec);
            candidates.push_back(c);
        }

        return candidates;
    }

    // ===================== 候选选择 =====================

    std::size_t PoseSolver::selectByGeometry(const std::vector<PnPCandidate> &candidates) {
        std::size_t best_id = 0;
        for (std::size_t i = 1; i < candidates.size(); ++i) {
            const bool cand_valid = candidates[i].world_pitch >= MIN_VALID_ARMOR_PITCH_WORLD;
            const bool best_valid = candidates[best_id].world_pitch >= MIN_VALID_ARMOR_PITCH_WORLD;

            if (cand_valid && !best_valid) {
                best_id = i;
            }
            else if (cand_valid == best_valid) {
                if (candidates[i].reprojection_error < candidates[best_id].reprojection_error) {
                    best_id = i;
                }
            }
        }
        return best_id;
    }

    std::size_t PoseSolver::selectByYawContinuity(const std::vector<PnPCandidate> &candidates, double nearest_yaw) {

        const bool has_valid = std::any_of(candidates.begin(), candidates.end(), [](const PnPCandidate &c) {
            return c.world_pitch >= MIN_VALID_ARMOR_PITCH_WORLD;
        });

        std::size_t continuous_id = 0;
        double min_delta = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            if (has_valid && candidates[i].world_pitch < MIN_VALID_ARMOR_PITCH_WORLD) {
                continue;
            }
            const double delta = std::abs(normalizeRadAngle(candidates[i].yaw - nearest_yaw));
            if (delta < min_delta) {
                min_delta = delta;
                continuous_id = i;
            }
        }
        return continuous_id;
    }

    std::size_t PoseSolver::selectBestCandidate(const std::vector<PnPCandidate> &candidates,
                                                int armor_name_key,
                                                const cv::Point2f &target_center) const {

        std::size_t best_id = selectByGeometry(candidates);

        auto group_it = record_.find(armor_name_key);
        if (group_it == record_.end() || candidates.size() < 2) {
            return best_id;
        }

        std::size_t nearest_idx = group_it->second.size();
        double min_dist = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < group_it->second.size(); ++i) {
            double dist = cv::norm(target_center - group_it->second[i].center);
            if (dist < SAME_ARMOR_CENTER_THRESH && dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        if (nearest_idx >= group_it->second.size()) {
            return best_id;
        }

        const auto &nearest_record = group_it->second[nearest_idx];
        std::size_t continuous_id = selectByYawContinuity(candidates, nearest_record.yaw);

        const double best_yaw_delta = std::abs(normalizeRadAngle(candidates[best_id].yaw - nearest_record.yaw));
        const double error_margin =
            candidates[continuous_id].reprojection_error - candidates[best_id].reprojection_error;

        if (best_yaw_delta > YAW_MUTATION_THRESH && error_margin < REPROJECTION_ERROR_MARGIN) {
            best_id = continuous_id;
        }

        return best_id;
    }

    // ===================== 工具方法 =====================

    double PoseSolver::calculateWorldPitchFromRvec(const cv::Mat &rvec) {
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_camera_armor;
        cv::cv2eigen(rmat, R_camera_armor);
        Eigen::Matrix3d R_world_armor = R_GIMBAL_CAMERA * R_camera_armor;
        return tools::calculateYPR(R_world_armor).y();
    }

    double PoseSolver::calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                                  const std::vector<cv::Point2f> &image_points,
                                                  const cv::Mat &rvec,
                                                  const cv::Mat &tvec) const {

        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distortion_coefficients_, projected);
        double error = 0.0;
        for (std::size_t j = 0; j < image_points.size(); ++j) {
            error += cv::norm(image_points[j] - projected[j]);
        }
        return error;
    }

    // ===================== 重投影 =====================

    std::vector<cv::Point2f>
    PoseSolver::reprojectArmor(const Eigen::Vector3d &xyz_gimbal, double yaw, ArmorType type) const {

        const auto R_pitch = Eigen::AngleAxisd(ARMOR_PITCH_RAD, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const auto R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d R_gimbal_armor = R_yaw * R_pitch;

        Eigen::Matrix3d R_camera_armor = R_CAMERA_GIMBAL * R_gimbal_armor;
        Eigen::Vector3d t_camera_armor = R_CAMERA_GIMBAL * xyz_gimbal;

        cv::Vec3d rvec_cv;
        cv::Vec3d tvec_cv;
        cv::Mat R_cv;
        cv::eigen2cv(R_camera_armor, R_cv);
        cv::Rodrigues(R_cv, rvec_cv);
        cv::eigen2cv(t_camera_armor, tvec_cv);

        const auto &object_points = (type == ArmorType::LARGE) ? LARGE_ARMOR_POINTS : SMALL_ARMOR_POINTS;
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(object_points, rvec_cv, tvec_cv, camera_matrix_, distortion_coefficients_, image_points);
        return image_points;
    }

    // ===================== Yaw 优化 =====================

    void PoseSolver::optimizeYaw(SolvedArmor &armor, const cv::Point2f &target_center) {
        const double init_yaw = armor.pose.ypr_gimbal.x();
        const Eigen::Vector3d &xyz_gimbal = armor.pose.xyz_gimbal;
        const ArmorType type = armor.geometry.type;

        auto error_func = [&](double yaw) -> double {
            auto pts = reprojectArmor(xyz_gimbal, yaw, type);
            double error = 0.0;
            for (int j = 0; j < 4; ++j) {
                error += cv::norm(pts[j] - armor.geometry.corners[j]);
            }
            return error;
        };

        armor.pose.ypr_gimbal.x() = yaw::runYawSearch(init_yaw, error_func);

        armor.pose.ypd_gimbal.x() = armor.pose.ypr_gimbal.x();
    }

} // namespace armor_detector
