#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 预处理中间结果显示 Observer。
 *
 * 把 gray / binary / color_mask 等中间图拼成窗口图像，提交给 DebugGUI。
 * 通过 DebugLayerState 控制是否渲染。关闭时清理旧的 preprocess_debug 窗口。
 */
class DebugPreprocessView : public IDebugObserver
{
public:
    DebugPreprocessView(DebugGUI & gui, DebugLayerState & layer_state);

    void onPreprocess(DebugFrameContext & context, const PreprocessDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
    DebugLayerState & layer_state_;
    bool was_shown_ = false;
};

} // namespace armor_detector::debug
