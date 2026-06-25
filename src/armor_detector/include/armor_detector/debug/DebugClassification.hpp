#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 数字分类可视化 Observer。
 *
 * 建议显示装甲板类别、置信度、number ROI 拼图。低置信度样本可交给
 * DebugRoiRecorder 保存。
 */
class DebugClassificationView : public IDebugObserver
{
public:
    explicit DebugClassificationView(DebugGUI & gui);

    void onClassification(DebugFrameContext & context, const ClassificationDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
};

} // namespace armor_detector::debug
