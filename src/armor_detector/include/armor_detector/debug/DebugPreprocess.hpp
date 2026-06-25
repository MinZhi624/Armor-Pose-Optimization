#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 预处理中间结果显示 Observer。
 *
 * 负责把 gray / binary / color_mask 等中间图拼成窗口图像，提交给 DebugGUI。
 */
class DebugPreprocessView : public IDebugObserver
{
public:
    explicit DebugPreprocessView(DebugGUI & gui);

    void onPreprocess(DebugFrameContext & context, const PreprocessDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
};

} // namespace armor_detector::debug
