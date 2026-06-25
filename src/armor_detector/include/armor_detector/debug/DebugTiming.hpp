#pragma once

#include "armor_detector/debug/IDebugObserver.hpp"

#include <chrono>
#include <cstddef>
#include <map>
#include <string>

namespace armor_detector::debug
{

/**
 * @brief 阶段耗时统计 Observer。
 *
 * 第一版建议只统计帧总耗时和每个阶段的平均耗时，不参与算法结果。
 */
class DebugTiming : public IDebugObserver
{
public:
    explicit DebugTiming(std::size_t report_interval = 50);

    void onFrameStart(const DebugFrameContext & context) override;
    void onPreprocess(const DebugFrameContext & context, const PreprocessDebugData & data) override;
    void onLights(const DebugFrameContext & context, const LightDebugData & data) override;
    void onArmorMatch(const DebugFrameContext & context, const ArmorMatchDebugData & data) override;
    void onClassification(const DebugFrameContext & context, const ClassificationDebugData & data) override;
    void onPoseSolved(const DebugFrameContext & context, const PoseDebugData & data) override;
    void onFrameEnd(const DebugFrameContext & context) override;

private:
    void mark(const std::string & stage_name);

    std::size_t report_interval_ = 50;
    std::size_t frame_count_ = 0;
    std::chrono::steady_clock::time_point frame_start_;
    std::chrono::steady_clock::time_point last_mark_;
    std::map<std::string, double> stage_time_sums_;
    double frame_time_sum_ms_ = 0.0;
};

} // namespace armor_detector::debug
