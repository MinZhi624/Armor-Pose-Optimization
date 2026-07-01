#include "armor_detector/debug/DebugPoseRefineTopicPublisher.hpp"

namespace armor_detector::debug {

    DebugPoseRefineTopicPublisher::DebugPoseRefineTopicPublisher(rclcpp::Node &node, DebugLayerState &layer_state)
        : layer_state_(layer_state) {
        publisher_ = node.create_publisher<armor_interfaces::msg::PoseRefineDebug>("/debug/pose_refine", 10);
    }

    void DebugPoseRefineTopicPublisher::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!layer_state_.enabled(DebugLayer::POSE)) {
            return;
        }

        // Only publish the first successful record
        for (const auto &record : data.refine_records) {
            if (record.success) {
                armor_interfaces::msg::PoseRefineDebug msg;
                msg.header.stamp = context.stamp;
                msg.header.frame_id = "gimbal";
                msg.origin_yaw_rad = record.initial_yaw_rad;
                msg.final_yaw_rad = record.final_yaw_rad;
                msg.error_delta_px = record.delta_reprojection_error_px;
                publisher_->publish(msg);
                return;
            }
        }
    }

} // namespace armor_detector::debug
