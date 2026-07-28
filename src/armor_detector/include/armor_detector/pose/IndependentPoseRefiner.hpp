#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"

namespace armor_detector::pose {
    class IndependentPoseRefiner : public IPoseRefiner {
    public:
        PoseRefineBatchOutput refine(const std::vector<PoseRefineInput> &inputs) const final;

    protected:
        virtual PoseRefineOutput refineOne(const PoseRefineInput &input) const = 0;
    };

} // namespace armor_detector::pose
