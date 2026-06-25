#include "armor_detector/debug/DebugPoseMarkerPublisher.hpp"
#include "armor_detector/tools/armor_geometry.hpp"

#include <Eigen/Geometry>

namespace armor_detector::debug {

    DebugPoseMarkerPublisher::DebugPoseMarkerPublisher(rclcpp::Node &node, DebugLayerState &layer_state) :
        layer_state_(layer_state) {
        publisher_ =
            node.create_publisher<visualization_msgs::msg::MarkerArray>("/armor_markers", rclcpp::SensorDataQoS());
    }

    void DebugPoseMarkerPublisher::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {

        if (!layer_state_.enabled(DebugLayer::POSE)) {
            publishDeleteAll();
            return;
        }

        publishCubes(context, data.solved_armors);
    }

    void DebugPoseMarkerPublisher::onKey(const DebugKeyEvent &event) {
        if (event.action == DebugKeyAction::TOGGLE_LAYER && event.layer == DebugLayer::POSE) {
            if (!layer_state_.enabled(DebugLayer::POSE)) {
                publishDeleteAll();
            }
        }
    }

    void DebugPoseMarkerPublisher::publishDeleteAll() {
        if (!has_published_) {
            return;
        }
        visualization_msgs::msg::MarkerArray msg;
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        msg.markers.push_back(delete_marker);
        publisher_->publish(msg);
        has_published_ = false;
    }

    void DebugPoseMarkerPublisher::publishCubes(DebugFrameContext &context, const std::vector<SolvedArmor> &armors) {
        visualization_msgs::msg::MarkerArray msg;

        // DELETEALL 清空上一帧
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "gimbal";
        delete_marker.header.stamp = context.stamp;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        msg.markers.push_back(delete_marker);

        for (std::size_t i = 0; i < armors.size(); ++i) {
            const auto &armor = armors[i];

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "gimbal";
            marker.header.stamp = context.stamp;
            marker.ns = "armor_pose";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // 位置：gimbal 坐标系
            marker.pose.position.x = armor.pose.xyz_gimbal.x();
            marker.pose.position.y = armor.pose.xyz_gimbal.y();
            marker.pose.position.z = armor.pose.xyz_gimbal.z();

            // 姿态：yaw 来自 PnP，pitch 固定 15°，roll 固定 0
            Eigen::Quaterniond q = Eigen::AngleAxisd(armor.pose.ypr_gimbal.x(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(tools::ARMOR_PITCH_RAD, Eigen::Vector3d::UnitY());
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            // 尺寸
            marker.scale.x = 0.01; // 厚度
            marker.scale.y =
                (armor.geometry.type == ArmorType::LARGE) ? tools::LARGE_ARMOR_WIDTH : tools::SMALL_ARMOR_WIDTH;
            marker.scale.z = tools::SMALL_ARMOR_HEIGHT;

            // 颜色：紫色
            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 0.5;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration::from_seconds(0);

            msg.markers.push_back(marker);
        }

        publisher_->publish(msg);
        has_published_ = true;
    }

} // namespace armor_detector::debug
