#pragma once

#include "armor_detector/debug/IDebugObserver.hpp"

#include <opencv2/core.hpp>

#include <filesystem>
#include <vector>

namespace armor_detector::debug {

    /**
     * @brief ROI 录制 Observer。
     *
     * 缓存分类阶段产生的 ROI，在收到 SAVE_ROI 按键事件时写入磁盘。
     */
    class DebugRoiRecorder : public IDebugObserver {
    public:
        explicit DebugRoiRecorder(std::filesystem::path output_dir);

        void onClassification(DebugFrameContext &context, const ClassificationDebugData &data) override;
        void onKey(const DebugKeyEvent &event) override;

    private:
        std::filesystem::path output_dir_;
        std::vector<cv::Mat> pending_rois_;
    };

} // namespace armor_detector::debug
