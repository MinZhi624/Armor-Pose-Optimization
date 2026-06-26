#pragma once

#include "armor_detector/debug/DebugData.hpp"

#include <mutex>
#include <string>
#include <unordered_map>

namespace armor_detector::debug {

    /**
     * @brief 线程安全的 debug 图层开关状态表。
     *
     * DebugGUI 在独立线程运行，数字键切换和 Observer 绘制可能并发读写。
     * 所有公开方法均通过 std::mutex 保护内部 map。
     */
    class DebugLayerState {
    public:
        void setEnabled(DebugLayer layer, bool enabled) {
            std::lock_guard<std::mutex> lock(mutex_);
            states_[layer] = enabled;
        }

        bool enabled(DebugLayer layer) const {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = states_.find(layer);
            return it != states_.end() && it->second;
        }

        /**
         * @brief 切换指定图层的开关状态，返回切换后的新值。
         */
        bool toggle(DebugLayer layer) {
            std::lock_guard<std::mutex> lock(mutex_);
            bool new_state = !states_[layer];
            states_[layer] = new_state;
            return new_state;
        }

        /**
         * @brief 返回图层的可读名称，用于日志输出。
         */
        static std::string name(DebugLayer layer) {
            switch (layer) {
                case DebugLayer::DETECT_STAGE_1:  return "DETECT_STAGE_1";
                case DebugLayer::DETECT_STAGE_2:  return "DETECT_STAGE_2";
                case DebugLayer::DETECT_STAGE_3:  return "DETECT_STAGE_3";
                case DebugLayer::DETECT_STAGE_4:  return "DETECT_STAGE_4";
                case DebugLayer::POSE:            return "POSE";
                case DebugLayer::RESULT:          return "RESULT";
                default:                          return "UNKNOWN";
            }
        }

    private:
        mutable std::mutex mutex_;
        std::unordered_map<DebugLayer, bool> states_;
    };

} // namespace armor_detector::debug
