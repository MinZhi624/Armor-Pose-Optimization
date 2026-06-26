#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    /**
     * @brief 数字分类可视化 Observer。
     *
     * 建议显示装甲板类别、置信度、number ROI 拼图。低置信度样本可交给
     * DebugRoiRecorder 保存。
     */
    class DebugClassificationView : public IDebugObserver {
    public:
        DebugClassificationView(DebugGUI &gui, DebugLayerState &layer_state);

        void onDetection(DebugFrameContext &context, const DetectionDebugData &data) override;

    private:
        DebugGUI *gui_ = nullptr;
        DebugLayerState &layer_state_;
    };

} // namespace armor_detector::debug
