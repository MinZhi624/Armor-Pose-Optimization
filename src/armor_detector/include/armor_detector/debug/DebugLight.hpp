#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    /**
     * @brief 灯条检测可视化 Observer。
     *
     * 绘制 accepted lights、rejected lights 和拒绝原因。
     * 通过 DebugLayerState 控制是否渲染。
     */
    class DebugLightView : public IDebugObserver {
    public:
        DebugLightView(DebugGUI &gui, DebugLayerState &layer_state);

        void onDetection(DebugFrameContext &context, const DetectionDebugData &data) override;

    private:
        DebugGUI *gui_ = nullptr;
        DebugLayerState &layer_state_;
    };

} // namespace armor_detector::debug
