#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 灯条检测可视化 Observer。
 *
 * 建议绘制 accepted lights、rejected lights 和拒绝原因。只读主流程数据，
 * 不改变 LightDetector 输出。
 */
class DebugLightView : public IDebugObserver
{
public:
    explicit DebugLightView(DebugGUI & gui);

    void onLights(const DebugFrameContext & context, const LightDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
};

} // namespace armor_detector::debug
