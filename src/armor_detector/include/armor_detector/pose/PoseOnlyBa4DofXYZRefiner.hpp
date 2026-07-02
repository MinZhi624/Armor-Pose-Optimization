#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"

namespace armor_detector::pose {

    class PoseOnlyBa4DofXYZRefiner : public IPoseRefiner {
    public:
        PoseRefineOutput refine(const PoseRefineInput &input) const override;
    };

} // namespace armor_detector::pose
