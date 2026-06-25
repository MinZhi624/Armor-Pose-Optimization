#include "armor_detector/debug/DebugLayerController.hpp"

#include <rclcpp/rclcpp.hpp>

namespace armor_detector::debug
{

DebugLayerController::DebugLayerController(DebugLayerState & layer_state)
    : layer_state_(layer_state)
{
}

void DebugLayerController::onKey(const DebugKeyEvent & event)
{
    if (event.action != DebugKeyAction::TOGGLE_LAYER) {
        return;
    }
    if (event.layer == DebugLayer::UNKNOWN) {
        return;
    }

    bool new_state = layer_state_.toggle(event.layer);
    RCLCPP_INFO(rclcpp::get_logger("DEBUG"),
                "layer %s: %s",
                DebugLayerState::name(event.layer).c_str(),
                new_state ? "ON" : "OFF");
}

} // namespace armor_detector::debug
