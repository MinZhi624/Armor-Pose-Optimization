#pragma once

#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

#include <chrono>
#include <cstddef>
#include <map>
#include <string>

namespace armor_detector::debug {

    /**
     * @brief 阶段耗时统计 Observer。
     *
     * 通过 onDetection 收集各阶段耗时，定期打印平均值。
     * onFrameStart / onFrameEnd 用于统计总帧时间。
     */
    class DebugTiming : public IDebugObserver {
    public:
        explicit DebugTiming(std::size_t report_interval = 50);

        void onFrameStart(DebugFrameContext &context) override;
        void onDetection(DebugFrameContext &context, const DetectionDebugData &data) override;
        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;
        void onFrameEnd(DebugFrameContext &context) override;

    private:
        std::size_t stats_interval_ = 50;
        std::size_t detected_frame_count_ = 0;
        std::size_t detection_window_count_ = 0;
        std::chrono::steady_clock::time_point frame_start_;

        std::map<std::string, double> stage_accum_;
        std::map<std::string, std::size_t> stage_count_;

        std::size_t total_frame_count_ = 0;
        std::size_t frame_window_count_ = 0;
        double frame_time_sum_ms_ = 0.0;

        // pose 计时
        std::chrono::steady_clock::time_point detect_end_;
        double pose_accum_ = 0.0;
        std::size_t pose_count_ = 0;
    };

} // namespace armor_detector::debug
