#pragma once

#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 处理数字键图层切换的 Observer。
 *
 * 只关心 onKey 事件。收到 TOGGLE_LAYER 时切换 DebugLayerState 并打印日志。
 * 不负责图层绘制——绘制由各 View Observer 根据 layer state 自行判断。
 */
class DebugLayerController : public IDebugObserver
{
public:
    explicit DebugLayerController(DebugLayerState & layer_state);

    void onKey(const DebugKeyEvent & event) override;

private:
    DebugLayerState & layer_state_;
};

} // namespace armor_detector::debug
