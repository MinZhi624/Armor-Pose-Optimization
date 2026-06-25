#pragma once

#include "armor_detector/debug/DebugData.hpp"

namespace armor_detector::debug {

    /**
     * @brief 把 cv::waitKey 返回的原始按键转换为 debug 语义动作。
     */
    class DebugKeyHandler {
    public:
        DebugKeyEvent translate(int raw_key) const;
    };

} // namespace armor_detector::debug
