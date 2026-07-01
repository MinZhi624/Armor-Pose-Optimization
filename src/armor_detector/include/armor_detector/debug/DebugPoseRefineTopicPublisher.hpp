#pragma once

#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

#include <armor_interfaces/msg/pose_refine_debug.hpp>

#include <rclcpp/rclcpp.hpp>

namespace armor_detector::debug {

    class DebugPoseRefineTopicPublisher : public IDebugObserver {
    public:
        DebugPoseRefineTopicPublisher(rclcpp::Node &node, DebugLayerState &layer_state);

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;

    private:
        rclcpp::Publisher<armor_interfaces::msg::PoseRefineDebug>::SharedPtr publisher_;
        DebugLayerState &layer_state_;
    };

} // namespace armor_detector::debug
