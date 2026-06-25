#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    /**
     * @brief PnP / 坐标结果可视化 Observer。
     *
     * 当前阶段只显示 camera 与项目习惯中的 gimbal 坐标。world 坐标需要真实
     * 云台姿态，留到跨包阶段再接入。
     */
    class DebugPoseView : public IDebugObserver {
    public:
        explicit DebugPoseView(DebugGUI &gui);

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;

    private:
        DebugGUI *gui_ = nullptr;
    };

} // namespace armor_detector::debug
