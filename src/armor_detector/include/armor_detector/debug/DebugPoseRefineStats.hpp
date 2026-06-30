#pragma once

#include "armor_detector/debug/IDebugObserver.hpp"
#include "armor_detector/debug/DebugData.hpp"

namespace armor_detector::debug {
    class DebugPoseRefineStats : public IDebugObserver {
    public:
        explicit DebugPoseRefineStats(std::size_t report_interval = 50);
        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;
    
    private:
        void reportAndReset();

        std::size_t frame_count_ = 0;
        std::size_t report_interval_;

        std::string method_;
        double error_sum_px_ = 0.0;
        std::size_t error_count_ = 0;
        std::size_t fail_count_ = 0;
        std::size_t success_count_ = 0;
    };
} // namespace armor_detector::debug