#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    class DebugPoseRefineView : public IDebugObserver {
    public:
        DebugPoseRefineView(DebugGUI &gui, DebugLayerState &layer_state, bool show_perturbations);

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;

    private:
        DebugGUI *gui_ = nullptr;
        DebugLayerState &layer_state_;
        bool show_perturbations_ = false;
    };

} // namespace armor_detector::debug
