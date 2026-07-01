#include "armor_detector/pose/PoseRefineRunner.hpp"
#include "armor_detector/tools/angle.hpp"

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

    PoseRefineOutput PoseRefineRunner::refine(const PoseRefineInput &input,
                                              const PoseErrorFunction &calculate_error) const {
        switch (method_) {
            case PoseRefineMethod::NONE:
                return none_refiner_.refine(input, calculate_error);
            case PoseRefineMethod::YAW_SEARCH:
                return yaw_search_refiner_.refine(input, calculate_error);
        }
        PoseRefineOutput output;
        output.xyz_gimbal = input.initial_xyz_gimbal;
        output.yaw_rad = input.initial_yaw_rad;
        output.success = false;
        output.reprojection_error_px = calculate_error(input.initial_xyz_gimbal, input.initial_yaw_rad);
        return output;
    }

    double PoseRefineRunner::calculateInitialError(const PoseRefineInput &input,
                                                   const PoseErrorFunction &calculate_error) const {
        return calculate_error(input.initial_xyz_gimbal, input.initial_yaw_rad);
    }

} // namespace armor_detector::pose
