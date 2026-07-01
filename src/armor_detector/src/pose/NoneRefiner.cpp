#include "armor_detector/pose/NoneRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

namespace armor_detector::pose {

    PoseRefineOutput NoneRefiner::refine(const PoseRefineInput &input) const {
        PoseRefineOutput output;
        output.rvec = input.initial_rvec;
        output.tvec = input.initial_tvec;
        output.reprojection_error_px = calculateReprojectionError(input.armor_type,
                                                                  input.image_corners,
                                                                  output.rvec,
                                                                  output.tvec,
                                                                  input.camera_matrix,
                                                                  input.distortion_coefficients);
        output.success = true;
        return output;
    }

} // namespace armor_detector::pose
