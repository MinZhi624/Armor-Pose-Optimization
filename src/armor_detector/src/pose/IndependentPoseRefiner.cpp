#include "armor_detector/pose/IndependentPoseRefiner.hpp"

namespace armor_detector::pose {
    PoseRefineBatchOutput IndependentPoseRefiner::refine(const std::vector<PoseRefineInput> &inputs) const {
        PoseRefineBatchOutput output;
        output.items.reserve(inputs.size());
        for (const auto &input : inputs) {
            output.items.push_back(refineOne(input));
        }
        return output;
    }

} // namespace armor_detector::pose
