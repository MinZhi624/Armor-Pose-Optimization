#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    class DebugCornerCorrectionView : public IDebugObserver {
    public:
        DebugCornerCorrectionView(DebugGUI &gui, DebugLayerState &layer_state);

        void onCornerCorrection(DebugFrameContext &context, const CornerCorrectionDebugData &data) override;

    private:
        DebugGUI *gui_ = nullptr;
        DebugLayerState &layer_state_;
        bool was_shown_ = false;
    };

} // namespace armor_detector::debug
