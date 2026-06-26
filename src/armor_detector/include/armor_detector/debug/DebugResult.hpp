#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

class DebugResultView : public IDebugObserver {
public:
    DebugResultView(DebugGUI &gui, DebugLayerState &layer_state);
    void onDetection(DebugFrameContext &context, const DetectionDebugData &data) override;

private:
    DebugGUI *gui_ = nullptr;
    DebugLayerState &layer_state_;
};

} // namespace armor_detector::debug
