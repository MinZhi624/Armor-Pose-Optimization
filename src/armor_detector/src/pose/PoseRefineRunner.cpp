#include "armor_detector/pose/PoseRefineRunner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

namespace armor_detector::pose {

    void PoseRefineRunner::setSingleMethod(SinglePoseRefineMethod method) {
        single_method_ = method;
        dual_armor_ba_3dof_ypd_refiner_.setFallbackRefiner(singleRefiner());
        dual_armor_ba_7dof_xyz_refiner_.setFallbackRefiner(singleRefiner());
    }

    SinglePoseRefineMethod PoseRefineRunner::singleMethod() const {
        return single_method_;
    }

    void PoseRefineRunner::setDualMethod(DualPoseRefineMethod method) {
        dual_method_ = method;
        dual_armor_ba_3dof_ypd_refiner_.setFallbackRefiner(singleRefiner());
        dual_armor_ba_7dof_xyz_refiner_.setFallbackRefiner(singleRefiner());
    }

    DualPoseRefineMethod PoseRefineRunner::dualMethod() const {
        return dual_method_;
    }

    std::string PoseRefineRunner::methodName() const {
        return "single_" + toString(single_method_) + "__dual_" + toString(dual_method_);
    }

    const IPoseRefiner &PoseRefineRunner::singleRefiner() const {
        switch (single_method_) {
            case SinglePoseRefineMethod::NONE:
                return none_refiner_;
            case SinglePoseRefineMethod::YAW_SEARCH:
                return yaw_search_refiner_;
            case SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE:
                return yaw_search_then_distance_refiner_;
            case SinglePoseRefineMethod::POSE_ONLY_BA_6DOF:
                return pose_only_ba_6dof_refiner_;
            case SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ:
                return pose_only_ba_4dof_xyz_refiner_;
            case SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_YPD:
                return pose_only_ba_4dof_ypd_refiner_;
        }

        return none_refiner_;
    }

    PoseRefineBatchOutput PoseRefineRunner::refine(const std::vector<PoseRefineInput> &inputs) const {
        switch (dual_method_) {
            case DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD:
                return dual_armor_ba_3dof_ypd_refiner_.refine(inputs);
            case DualPoseRefineMethod::DUAL_ARMOR_BA_7DOF_XYZ:
                return dual_armor_ba_7dof_xyz_refiner_.refine(inputs);
            case DualPoseRefineMethod::NONE:
                return singleRefiner().refine(inputs);
        }
        return singleRefiner().refine(inputs);
    }

    double PoseRefineRunner::calculateInitialError(const PoseRefineInput &input) const {
        return calculateReprojectionError(input.armor_type,
                                          input.image_corners,
                                          input.initial_rvec,
                                          input.initial_tvec,
                                          input.camera_matrix,
                                          input.distortion_coefficients);
    }

} // namespace armor_detector::pose
