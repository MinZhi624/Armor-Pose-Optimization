#pragma once

#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

namespace armor_detector::debug {

    /**
     * @brief 装甲板 3D pose marker 发布 Observer。
     *
     * 通过 /armor_markers 发布 MarkerArray，每个 SolvedArmor 一个 CUBE marker。
     * 由 debug.pose 图层控制开关，不依赖 OpenCV GUI。
     */
    class DebugPoseMarkerPublisher : public IDebugObserver {
    public:
        DebugPoseMarkerPublisher(rclcpp::Node &node, DebugLayerState &layer_state);

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;
        void onKey(const DebugKeyEvent &event) override;

    private:
        void publishDeleteAll();
        void publishCubes(DebugFrameContext &context, const std::vector<SolvedArmor> &armors);

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
        DebugLayerState &layer_state_;
        bool has_published_ = false;
    };

} // namespace armor_detector::debug
