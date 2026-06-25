#include "armor_detector/debug/DebugKeyHandler.hpp"

namespace armor_detector::debug {

    DebugKeyEvent DebugKeyHandler::translate(int raw_key) const {
        DebugKeyAction action = DebugKeyAction::NONE;
        DebugLayer layer = DebugLayer::UNKNOWN;

        if (raw_key == 27 || raw_key == 'q' || raw_key == 'Q') {
            action = DebugKeyAction::EXIT;
        }
        else if (raw_key == ' ' || raw_key == 'p' || raw_key == 'P') {
            action = DebugKeyAction::PAUSE_TOGGLE;
        }
        else if (raw_key == 'n' || raw_key == 'N' || raw_key == 65363 || raw_key == 2555904 || raw_key == 0x270000) {
            action = DebugKeyAction::STEP_FRAME;
        }
        else if (raw_key == 's' || raw_key == 'S') {
            action = DebugKeyAction::SAVE_ROI;
        }
        else if (raw_key == '+' || raw_key == '=') {
            action = DebugKeyAction::PLAYBACK_RATE_UP;
        }
        else if (raw_key == '-' || raw_key == '_') {
            action = DebugKeyAction::PLAYBACK_RATE_DOWN;
        }
        else if (raw_key == '1') {
            action = DebugKeyAction::TOGGLE_LAYER;
            layer = DebugLayer::PREPROCESS;
        }
        else if (raw_key == '2') {
            action = DebugKeyAction::TOGGLE_LAYER;
            layer = DebugLayer::LIGHTS;
        }
        else if (raw_key == '3') {
            action = DebugKeyAction::TOGGLE_LAYER;
            layer = DebugLayer::ARMOR_MATCH;
        }
        else if (raw_key == '4') {
            action = DebugKeyAction::TOGGLE_LAYER;
            layer = DebugLayer::CLASSIFICATION;
        }
        else if (raw_key == '5') {
            action = DebugKeyAction::TOGGLE_LAYER;
            layer = DebugLayer::POSE;
        }
        else if (raw_key == '6') {
            action = DebugKeyAction::TOGGLE_LAYER;
            layer = DebugLayer::RESULT;
        }

        return {action, raw_key, layer};
    }

} // namespace armor_detector::debug
