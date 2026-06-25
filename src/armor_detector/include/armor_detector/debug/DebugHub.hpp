#pragma once

#include "armor_detector/debug/IDebugObserver.hpp"

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace armor_detector::debug
{

/**
 * @brief Debug 事件分发器。
 *
 * DebugHub 不负责绘制、保存、统计等具体行为，只保存 Observer 列表，
 * 并把主流程发出的阶段事件转发给所有已注册 Observer。
 */
class DebugHub
{
public:
    void addObserver(std::shared_ptr<IDebugObserver> observer)
    {
        if (observer) {
            observers_.push_back(std::move(observer));
        }
    }

    void clearObservers() { observers_.clear(); }
    std::size_t observerCount() const { return observers_.size(); }
    bool empty() const { return observers_.empty(); }

    void onFrameStart(const DebugFrameContext & context)
    {
        for (auto & observer : observers_) observer->onFrameStart(context);
    }

    void onPreprocess(const DebugFrameContext & context, const PreprocessDebugData & data)
    {
        for (auto & observer : observers_) observer->onPreprocess(context, data);
    }

    void onLights(const DebugFrameContext & context, const LightDebugData & data)
    {
        for (auto & observer : observers_) observer->onLights(context, data);
    }

    void onArmorMatch(const DebugFrameContext & context, const ArmorMatchDebugData & data)
    {
        for (auto & observer : observers_) observer->onArmorMatch(context, data);
    }

    void onClassification(const DebugFrameContext & context, const ClassificationDebugData & data)
    {
        for (auto & observer : observers_) observer->onClassification(context, data);
    }

    void onPoseSolved(const DebugFrameContext & context, const PoseDebugData & data)
    {
        for (auto & observer : observers_) observer->onPoseSolved(context, data);
    }

    void onFrameEnd(const DebugFrameContext & context)
    {
        for (auto & observer : observers_) observer->onFrameEnd(context);
    }

    void onKey(const DebugKeyEvent & event)
    {
        for (auto & observer : observers_) observer->onKey(event);
    }

private:
    std::vector<std::shared_ptr<IDebugObserver>> observers_;
};

} // namespace armor_detector::debug
