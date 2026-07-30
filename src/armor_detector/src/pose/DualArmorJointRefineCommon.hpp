#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"
#include "armor_detector/pose/PoseProjection.hpp"

#include <cstddef>
#include <vector>

namespace armor_detector::pose::dual_armor_detail {

    struct PairIndices {
        std::size_t armor_a_index = 0;
        std::size_t armor_b_index = 0;
    };

    bool findUniquePair(const std::vector<PoseRefineInput> &inputs, PairIndices &pair);
    bool initializeSharedYaw(const ArmorPose &pose_a,
                             const ArmorPose &pose_b,
                             double yaw_offset_rad,
                             double &shared_yaw_rad);
    bool selectPlusCandidate(bool plus_usable, double plus_cost, bool minus_usable, double minus_cost);
    PoseRefineOutput pnpOutput(const PoseRefineInput &input);
    void setPairPnpFallback(const std::vector<PoseRefineInput> &inputs,
                            const PairIndices &pair,
                            PoseRefineBatchOutput &output,
                            const PoseRefineSolverSummary *solver_summary = nullptr);
    void copyFallbackItems(const std::vector<PoseRefineInput> &inputs,
                           const PairIndices &pair,
                           const IPoseRefiner *fallback_refiner,
                           PoseRefineBatchOutput &output);

} // namespace armor_detector::pose::dual_armor_detail
