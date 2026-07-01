#include "armor_detector/pose/YawSearchRefiner.hpp"
#include "armor_detector/pose/PoseSearch.hpp"

namespace armor_detector::pose {

    PoseRefineOutput YawSearchRefiner::refine(const PoseRefineInput &input,
                                              const PoseErrorFunction &calculate_error) const {
        PoseRefineOutput output;
        output.xyz_gimbal = input.initial_xyz_gimbal;
        output.yaw_rad = input.initial_yaw_rad;
        output.success = false;

        auto yaw_error_func = [&](double yaw) -> double {
            return calculate_error(input.initial_xyz_gimbal, yaw);
        };

        output.yaw_rad = refineYawFromPnp(input.initial_yaw_rad, yaw_error_func);
        output.reprojection_error_px = calculate_error(output.xyz_gimbal, output.yaw_rad);
        output.success = true;
        return output;
    }

} // namespace armor_detector::pose
