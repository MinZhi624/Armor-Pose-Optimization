#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    /**
     * @brief YOLO 检测结果 Debug Observer。
     *
     * 4 个阶段槽位：
     *   Stage 1 — letterbox 输入图
     *   Stage 2 — score 阈值后的候选框（画在 display 上）
     *   Stage 3 — NMS kept + NMS 抑制 + 类别/颜色过滤的拒绝目标（画在 display 上）
     *   Stage 4 — 最终检测结果（画在 display 上）
     */
    class DebugYoloView : public IDebugObserver {
    public:
        DebugYoloView(DebugGUI &gui, DebugLayerState &layer_state);

        void onDetection(DebugFrameContext &context, const DetectionDebugData &data) override;

    private:
        void showStage1(const DetectionDebugData &data);
        void drawDetections(cv::Mat &display, const std::vector<ClassifiedArmor> &armors,
                            const cv::Scalar &color, const std::string &label);
        void drawRejected(cv::Mat &display, const std::vector<RejectedArmor> &rejected);

        DebugGUI *gui_ = nullptr;
        DebugLayerState &layer_state_;
    };

} // namespace armor_detector::debug
