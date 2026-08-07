#pragma once

#include "armor_detector/pose/IndependentPoseRefiner.hpp"

namespace armor_detector::pose {

    class PoseOnlyBa4DofXYZRefiner : public IndependentPoseRefiner {
    protected:
        PoseRefineOutput refineOne(const PoseRefineInput &input) const override;
    };

} // namespace armor_detector::pose
