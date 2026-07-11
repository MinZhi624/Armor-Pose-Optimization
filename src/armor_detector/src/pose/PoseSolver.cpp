#include "armor_detector/pose/PoseSolver.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/angle.hpp"
#include "armor_detector/tools/armor_geometry.hpp"
#include "armor_detector/tools/geometry.hpp"
#include "armor_detector/tools/transform.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace armor_detector {

    using tools::LARGE_ARMOR_POINTS;
    using tools::normalizeRadAngle;
    using tools::R_GIMBAL_CAMERA;
    using tools::SMALL_ARMOR_POINTS;

    namespace {
        constexpr double SAME_ARMOR_CENTER_THRESH = 30.0;
        constexpr double YAW_MUTATION_THRESH = M_PI_2;
        constexpr double REPROJECTION_ERROR_MARGIN = 3.0;
        constexpr double MIN_VALID_ARMOR_PITCH_WORLD = -0.05;

        cv::Mat distortionMat(const cv::Vec<double, 5> &distortion_coefficients) {
            cv::Mat distortion_mat(1, 5, CV_64F);
            for (int i = 0; i < 5; ++i) {
                distortion_mat.at<double>(0, i) = distortion_coefficients[i];
            }
            return distortion_mat;
        }
    } // namespace

    // ===================== 构造 / 初始化 =====================

    PoseSolver::PoseSolver(const CameraInfo &camera_info) {
        init(camera_info);
    }

    void PoseSolver::init(const CameraInfo &camera_info) {
        camera_matrix_ = camera_info.camera_matrix;
        distortion_coefficients_ = camera_info.distortion_coefficients;
    }

    // ===================== 主流程 =====================

    std::vector<SolvedArmor> PoseSolver::solve(const std::vector<DetectedArmor> &armors,
                                               const pose::PoseRefineRunner &pose_refiner) {
        pose_debug_.refine_records.clear();
        pose_debug_.solved_armors.clear();
        pose_debug_.timings.clear();

        double pnp_elapsed_ms = 0.0;
        double refine_elapsed_ms = 0.0;

        std::unordered_map<int, std::vector<LastArmorYawRecord>> new_record;
        std::vector<SolvedArmor> solved_armors;
        solved_armors.reserve(armors.size());
        for (std::size_t i = 0; i < armors.size(); ++i) {
            auto pnp_start = std::chrono::steady_clock::now();
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

            const auto &best_candidate = candidates[best_id];
            SolvedArmor solved = createSolvedArmorFromRvecTvec(classified, best_candidate.rvec, best_candidate.tvec);
            const ArmorPose initial_pose = solved.pose;

            auto pnp_end = std::chrono::steady_clock::now();
            pnp_elapsed_ms += std::chrono::duration<double, std::milli>(pnp_end - pnp_start).count();

            // Refine pose
            auto refine_start = std::chrono::steady_clock::now();
            pose::PoseRefineInput refine_input;
            refine_input.image_corners = solved.geometry.corners;
            refine_input.initial_rvec = best_candidate.rvec;
            refine_input.initial_tvec = best_candidate.tvec;
            refine_input.armor_type = classified.geometry.type;
            refine_input.camera_matrix = camera_matrix_;
            refine_input.distortion_coefficients = distortion_coefficients_;

            const double initial_error = pose_refiner.calculateInitialError(refine_input);
            const auto refine_output = pose_refiner.refine(refine_input);
            if (refine_output.success) {
                solved = createSolvedArmorFromRvecTvec(classified, refine_output.rvec, refine_output.tvec);
            }
            const ArmorPose &final_pose = solved.pose;

            // Fill debug record with full fields
            debug::PoseRefineDebugRecord rec;
            rec.armor_index = solved_armors.size();
            rec.armor_name = static_cast<int>(classified.classification.name);
            rec.armor_type = (classified.geometry.type == ArmorType::LARGE) ? "large"
                : (classified.geometry.type == ArmorType::SMALL)            ? "small"
                                                                            : "none";
            rec.confidence = classified.classification.confidence;
            rec.center_x_px = target_center.x;
            rec.center_y_px = target_center.y;
            rec.method = pose_refiner.methodName();
            rec.success = refine_output.success;
            rec.initial_xyz_gimbal = initial_pose.xyz_gimbal;
            rec.final_xyz_gimbal = final_pose.xyz_gimbal;
            rec.delta_xyz_gimbal = final_pose.xyz_gimbal - initial_pose.xyz_gimbal;
            rec.initial_yaw_rad = initial_pose.ypr_gimbal.x();
            rec.final_yaw_rad = final_pose.ypr_gimbal.x();
            rec.delta_yaw_rad = normalizeRadAngle(rec.final_yaw_rad - rec.initial_yaw_rad);
            rec.initial_reprojection_error_px = initial_error;
            rec.final_reprojection_error_px = refine_output.reprojection_error_px;
            rec.delta_reprojection_error_px = initial_error - refine_output.reprojection_error_px;
            if (refine_output.success) {
                const auto projected_corners = pose::projectArmor(classified.geometry.type,
                                                                  refine_output.rvec,
                                                                  refine_output.tvec,
                                                                  camera_matrix_,
                                                                  distortion_coefficients_);
                std::copy_n(projected_corners.begin(), rec.projected_corners.size(), rec.projected_corners.begin());
                rec.has_projected_corners = true;
            }
            pose_debug_.refine_records.push_back(rec);

            auto refine_end = std::chrono::steady_clock::now();
            refine_elapsed_ms += std::chrono::duration<double, std::milli>(refine_end - refine_start).count();
            new_record[armor_name_key].push_back({solved.pose.ypr_gimbal.x(), target_center});
            solved_armors.push_back(std::move(solved));
        }
        // 记录
        record_ = std::move(new_record);
        pose_debug_.timings.push_back({"pnp", pnp_elapsed_ms});
        pose_debug_.timings.push_back({"refine", refine_elapsed_ms});

        pose_debug_.solved_armors = solved_armors;
        return solved_armors;
    }

    // ===================== PnP 候选 =====================

    std::vector<PoseSolver::PnPCandidate>
    PoseSolver::createPnPCandidates(const std::vector<cv::Point3f> &object_points,
                                    const std::vector<cv::Point2f> &image_points) const {

        const cv::Mat camera_mat(camera_matrix_);
        const cv::Mat distortion_mat = distortionMat(distortion_coefficients_);

        std::vector<cv::Mat> ippe_rvecs, ippe_tvecs;
        cv::solvePnPGeneric(
            object_points, image_points, camera_mat, distortion_mat, ippe_rvecs, ippe_tvecs, false, cv::SOLVEPNP_IPPE);

        std::vector<PnPCandidate> candidates;
        candidates.reserve(ippe_rvecs.size());
        for (std::size_t j = 0; j < ippe_rvecs.size(); ++j) {
            PnPCandidate c;
            c.rvec = pose::matToVec3d(ippe_rvecs[j]);
            c.tvec = pose::matToVec3d(ippe_tvecs[j]);

            const Eigen::Matrix3d R_gimbal_armor = R_GIMBAL_CAMERA * pose::rotationMatrixFromRvec(c.rvec);
            c.yaw = tools::calculateYPR(R_gimbal_armor).x();
            c.world_pitch = calculateWorldPitchFromRvec(c.rvec);
            c.reprojection_error = calculateReprojectionError(object_points, image_points, c.rvec, c.tvec);
            candidates.push_back(c);
        }

        if (candidates.empty()) {
            cv::Mat rvec;
            cv::Mat tvec;
            const bool solved = cv::solvePnP(
                object_points, image_points, camera_mat, distortion_mat, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            if (solved) {
                PnPCandidate c;
                c.rvec = pose::matToVec3d(rvec);
                c.tvec = pose::matToVec3d(tvec);
                const Eigen::Matrix3d R_gimbal_armor = R_GIMBAL_CAMERA * pose::rotationMatrixFromRvec(c.rvec);
                c.yaw = tools::calculateYPR(R_gimbal_armor).x();
                c.world_pitch = calculateWorldPitchFromRvec(c.rvec);
                c.reprojection_error = calculateReprojectionError(object_points, image_points, c.rvec, c.tvec);
                candidates.push_back(c);
            }
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

    double PoseSolver::calculateWorldPitchFromRvec(const cv::Vec3d &rvec) {
        const Eigen::Matrix3d R_world_armor = R_GIMBAL_CAMERA * pose::rotationMatrixFromRvec(rvec);
        return tools::calculateYPR(R_world_armor).y();
    }

    double PoseSolver::calculateReprojectionError(const std::vector<cv::Point3f> &object_points,
                                                  const std::vector<cv::Point2f> &image_points,
                                                  const cv::Vec3d &rvec,
                                                  const cv::Vec3d &tvec) const {
        return pose::calculateReprojectionError(
            object_points, image_points, rvec, tvec, camera_matrix_, distortion_coefficients_);
    }

    SolvedArmor PoseSolver::createSolvedArmorFromRvecTvec(const DetectedArmor &armor,
                                                          const cv::Vec3d &rvec,
                                                          const cv::Vec3d &tvec) const {
        SolvedArmor solved;
        solved.geometry = armor.geometry;
        solved.classification = armor.classification;
        solved.pose = pose::calculateArmorPose(rvec, tvec, armor.geometry.corners, camera_matrix_);
        return solved;
    }

} // namespace armor_detector
