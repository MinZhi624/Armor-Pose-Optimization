#pragma once

#include "armor_detector/pose/PoseRefineData.hpp"

namespace armor_detector::pose {

    class IPoseRefiner {
    public:
        virtual ~IPoseRefiner() = default;
        virtual PoseRefineBatchOutput refine(const std::vector<PoseRefineInput> &inputs) const = 0;
    };

} // namespace armor_detector::pose
