#include "armor_detector/pose/PoseRefineRunner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

namespace armor_detector::pose {

    void PoseRefineRunner::setMethod(PoseRefineMethod method) {
        method_ = method;
    }

    PoseRefineMethod PoseRefineRunner::method() const {
        return method_;
    }

    std::string PoseRefineRunner::methodName() const {
        return toString(method_);
    }

    PoseRefineOutput PoseRefineRunner::refine(const PoseRefineInput &input) const {
        switch (method_) {
            case PoseRefineMethod::NONE:
                return none_refiner_.refine(input);
            case PoseRefineMethod::YAW_SEARCH:
                return yaw_search_refiner_.refine(input);
            case PoseRefineMethod::POSE_ONLY_BA_6DOF:
                return pose_only_ba_6dof_refiner_.refine(input);
            case PoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ:
                return pose_only_ba_4dof_xyz_refiner_.refine(input);
            case PoseRefineMethod::POSE_ONLY_BA_4DOF_YPD:
                return pose_only_ba_4dof_ypd_refiner_.refine(input);
        }

        PoseRefineOutput output;
        output.rvec = input.initial_rvec;
        output.tvec = input.initial_tvec;
        output.success = false;
        output.reprojection_error_px = calculateInitialError(input);
        return output;
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
