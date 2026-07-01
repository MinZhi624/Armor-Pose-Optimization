#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"

namespace armor_detector::pose {

    class YawSearchRefiner : public IPoseRefiner {
    public:
        PoseRefineOutput refine(const PoseRefineInput &input,
                                const PoseErrorFunction &calculate_error) const override;
    };

} // namespace armor_detector::pose
