#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 装甲板匹配可视化 Observer。
 *
 * 用于显示候选装甲板、被拒绝装甲板及其几何拒绝原因。
 */
class DebugArmorMatchView : public IDebugObserver
{
public:
    explicit DebugArmorMatchView(DebugGUI & gui);

    void onArmorMatch(const DebugFrameContext & context, const ArmorMatchDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
};

} // namespace armor_detector::debug
