#include "armor_detector/pose/PoseSolver.hpp"
#include "armor_detector/pose/PoseProjection.hpp"
#include "armor_detector/tools/angle.hpp"
#include "armor_detector/tools/armor_geometry.hpp"
#include "armor_detector/tools/geometry.hpp"
#include "armor_detector/tools/transform.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

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

        bool copyProjectedCorners(const std::vector<cv::Point2f> &projected, std::array<cv::Point2f, 4> &corners) {
            if (projected.size() != corners.size()) {
                return false;
            }
            std::copy(projected.begin(), projected.end(), corners.begin());
            return true;
        }

        double calculate4DofModelMeanError(ArmorType armor_type,
                                           const std::array<cv::Point2f, 4> &image_corners,
                                           const Eigen::Vector3d &xyz_gimbal,
                                           double yaw_rad,
                                           const cv::Matx33d &camera_matrix,
                                           const cv::Vec<double, 5> &distortion_coefficients) {
            if (armor_type == ArmorType::NONE) {
                return 0.0;
            }

            const auto &object_points = (armor_type == ArmorType::LARGE) ? LARGE_ARMOR_POINTS : SMALL_ARMOR_POINTS;

            const std::vector<cv::Point2d> image_corners_vec(image_corners.begin(), image_corners.end());
            std::vector<cv::Point2d> norm_points;
            cv::undistortPoints(
                image_corners_vec, norm_points, cv::Mat(camera_matrix), distortionMat(distortion_coefficients));

            const double fx = camera_matrix(0, 0);
            const double fy = camera_matrix(1, 1);
            const double pitch = tools::ARMOR_PITCH_RAD;

            const auto R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
            const auto R_yaw = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Matrix3d R_gimbal_armor = R_yaw * R_pitch;

            double total_error = 0.0;
            for (std::size_t i = 0; i < 4; ++i) {
                const Eigen::Vector3d p_armor(object_points[i].x, object_points[i].y, object_points[i].z);
                const Eigen::Vector3d p_gimbal = R_gimbal_armor * p_armor + xyz_gimbal;

                const Eigen::Vector3d p_camera(-p_gimbal.y(), -p_gimbal.z(), p_gimbal.x());

                const double x_pred = p_camera.x() / p_camera.z();
                const double y_pred = p_camera.y() / p_camera.z();

                const double dx = fx * (x_pred - norm_points[i].x);
                const double dy = fy * (y_pred - norm_points[i].y);

                total_error += std::sqrt(dx * dx + dy * dy);
            }

            return total_error / 4.0;
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

    void PoseSolver::setPosePerturbationParams(const PosePerturbationParams &params) {
        pose_perturbation_params_ = params;
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
            const double reprojection_point_count = static_cast<double>(refine_input.image_corners.size());
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
            rec.initial_dir_yaw_rad = initial_pose.ypd_gimbal.x();
            rec.final_dir_yaw_rad = final_pose.ypd_gimbal.x();
            rec.delta_dir_yaw_rad = normalizeRadAngle(rec.final_dir_yaw_rad - rec.initial_dir_yaw_rad);
            rec.initial_dir_pitch_rad = initial_pose.ypd_gimbal.y();
            rec.final_dir_pitch_rad = final_pose.ypd_gimbal.y();
            rec.delta_dir_pitch_rad = rec.final_dir_pitch_rad - rec.initial_dir_pitch_rad;
            rec.initial_distance_m = initial_pose.ypd_gimbal.z();
            rec.final_distance_m = final_pose.ypd_gimbal.z();
            rec.delta_distance_m = rec.final_distance_m - rec.initial_distance_m;
            rec.initial_yaw_rad = initial_pose.ypr_gimbal.x();
            rec.final_yaw_rad = final_pose.ypr_gimbal.x();
            rec.delta_yaw_rad = normalizeRadAngle(rec.final_yaw_rad - rec.initial_yaw_rad);
            rec.initial_reprojection_error_px = initial_error;
            rec.final_reprojection_error_px = refine_output.reprojection_error_px;
            rec.delta_reprojection_error_px = initial_error - refine_output.reprojection_error_px;
            rec.initial_reproj_sum_px = initial_error;
            rec.final_reproj_sum_px = refine_output.reprojection_error_px;
            rec.delta_reproj_sum_px = rec.final_reproj_sum_px - rec.initial_reproj_sum_px;
            rec.initial_reproj_mean_px = rec.initial_reproj_sum_px / reprojection_point_count;
            rec.final_reproj_mean_px = rec.final_reproj_sum_px / reprojection_point_count;
            rec.delta_reproj_mean_px = rec.final_reproj_mean_px - rec.initial_reproj_mean_px;
            rec.ba_model_initial_reproj_mean_px = calculate4DofModelMeanError(classified.geometry.type,
                                                                              refine_input.image_corners,
                                                                              initial_pose.xyz_gimbal,
                                                                              initial_pose.ypr_gimbal.x(),
                                                                              camera_matrix_,
                                                                              distortion_coefficients_);
            rec.ba_model_final_reproj_mean_px = calculate4DofModelMeanError(classified.geometry.type,
                                                                            refine_input.image_corners,
                                                                            final_pose.xyz_gimbal,
                                                                            final_pose.ypr_gimbal.x(),
                                                                            camera_matrix_,
                                                                            distortion_coefficients_);
            rec.has_solver_summary = refine_output.solver_summary.available;
            rec.initial_cost = refine_output.solver_summary.initial_cost;
            rec.final_cost = refine_output.solver_summary.final_cost;
            rec.delta_cost = rec.final_cost - rec.initial_cost;
            rec.num_iterations = refine_output.solver_summary.num_iterations;
            rec.termination_type = refine_output.solver_summary.termination_type;
            if (refine_output.success) {
                const auto projected_corners = pose::projectArmor(classified.geometry.type,
                                                                  refine_output.rvec,
                                                                  refine_output.tvec,
                                                                  camera_matrix_,
                                                                  distortion_coefficients_);
                rec.has_projected_corners = copyProjectedCorners(projected_corners, rec.projected_corners);

                if (rec.has_projected_corners && pose_perturbation_params_.enabled) {
                    const auto projectDirectionPerturbation = [&](const Eigen::Vector3d &ypd_gimbal,
                                                                  std::array<cv::Point2f, 4> &corners) {
                        const Eigen::Vector3d xyz_camera = tools::R_CAMERA_GIMBAL * tools::calculateXYZ(ypd_gimbal);
                        const cv::Vec3d tvec_camera(xyz_camera.x(), xyz_camera.y(), xyz_camera.z());
                        return copyProjectedCorners(pose::projectArmor(classified.geometry.type,
                                                                       refine_output.rvec,
                                                                       tvec_camera,
                                                                       camera_matrix_,
                                                                       distortion_coefficients_),
                                                    corners);
                    };

                    const Eigen::Vector3d final_ypd_gimbal = final_pose.ypd_gimbal;
                    auto dir_yaw_ypd = final_ypd_gimbal;
                    dir_yaw_ypd.x() += pose_perturbation_params_.dir_yaw_delta_rad;
                    auto dir_pitch_ypd = final_ypd_gimbal;
                    dir_pitch_ypd.y() += pose_perturbation_params_.dir_pitch_delta_rad;
                    auto distance_ypd = final_ypd_gimbal;
                    distance_ypd.z() += pose_perturbation_params_.distance_delta_m;

                    auto &perturbed = rec.perturbation_projected_corners;
                    const bool direction_corners_available =
                        projectDirectionPerturbation(dir_yaw_ypd, perturbed.dir_yaw_corners) &&
                        projectDirectionPerturbation(dir_pitch_ypd, perturbed.dir_pitch_corners) &&
                        projectDirectionPerturbation(distance_ypd, perturbed.distance_corners);

                    const Eigen::Matrix3d r_gimbal_armor =
                        tools::R_GIMBAL_CAMERA * pose::rotationMatrixFromRvec(refine_output.rvec);
                    const Eigen::Matrix3d r_pose_yaw =
                        Eigen::AngleAxisd(pose_perturbation_params_.pose_yaw_delta_rad, Eigen::Vector3d::UnitZ())
                            .toRotationMatrix();
                    const Eigen::Matrix3d r_camera_armor = tools::R_CAMERA_GIMBAL * r_pose_yaw * r_gimbal_armor;
                    cv::Mat r_camera_armor_cv;
                    cv::eigen2cv(r_camera_armor, r_camera_armor_cv);
                    cv::Vec3d pose_yaw_rvec;
                    cv::Rodrigues(r_camera_armor_cv, pose_yaw_rvec);
                    const bool pose_yaw_corners_available =
                        copyProjectedCorners(pose::projectArmor(classified.geometry.type,
                                                                pose_yaw_rvec,
                                                                refine_output.tvec,
                                                                camera_matrix_,
                                                                distortion_coefficients_),
                                             perturbed.pose_yaw_corners);
                    perturbed.available = direction_corners_available && pose_yaw_corners_available;
                }
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
