#pragma once

#include "armor_detector/debug/DebugData.hpp"

namespace armor_detector::debug
{

/**
 * @brief Debug 观察者接口。
 *
 * 主检测流程只在阶段结束时向 DebugHub 发送事件。具体的显示、记录、
 * 统计、ROI 保存等功能由不同 Observer 自己响应。
 *
 * 默认空实现的目的：每个 Observer 只重写自己关心的事件，避免为了
 * 一个 debug 功能被迫实现所有阶段。
 */
class IDebugObserver
{
public:
    virtual ~IDebugObserver() = default;

    virtual void onFrameStart(DebugFrameContext & context) {}
    virtual void onPreprocess(DebugFrameContext & context, const PreprocessDebugData & data) {}
    virtual void onLights(DebugFrameContext & context, const LightDebugData & data) {}
    virtual void onArmorMatch(DebugFrameContext & context, const ArmorMatchDebugData & data) {}
    virtual void onClassification(DebugFrameContext & context, const ClassificationDebugData & data) {}
    virtual void onPoseSolved(DebugFrameContext & context, const PoseDebugData & data) {}
    virtual void onFrameEnd(DebugFrameContext & context) {}
    virtual void onKey(const DebugKeyEvent & event) {}
};

} // namespace armor_detector::debug
