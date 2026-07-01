#include "armor_detector/pose/NoneRefiner.hpp"

namespace armor_detector::pose {

    PoseRefineOutput NoneRefiner::refine(const PoseRefineInput &input,
                                         const PoseErrorFunction &calculate_error) const {
        PoseRefineOutput output;
        output.xyz_gimbal = input.initial_xyz_gimbal;
        output.yaw_rad = input.initial_yaw_rad;
        output.reprojection_error_px = calculate_error(output.xyz_gimbal, output.yaw_rad);
        output.success = true;
        return output;
    }

} // namespace armor_detector::pose
