#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"

namespace armor_detector::pose {

    class DualArmorBa3DofYPDRefiner : public IPoseRefiner {
    public:
        PoseRefineBatchOutput refine(const std::vector<PoseRefineInput> &inputs) const final;

        void setFallbackRefiner(const IPoseRefiner &fallback_refiner);

    private:
        const IPoseRefiner *fallback_refiner_ = nullptr;
    };

} // namespace armor_detector::pose
