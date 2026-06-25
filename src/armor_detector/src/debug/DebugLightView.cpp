#include "armor_detector/debug/DebugLight.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugLightView::DebugLightView(DebugGUI & gui, DebugLayerState & layer_state)
    : gui_(&gui), layer_state_(layer_state)
{
}

void DebugLightView::onLights(
    DebugFrameContext & context,
    const LightDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;
    if (!layer_state_.enabled(DebugLayer::LIGHTS)) return;
    if (context.display_bgr.empty()) return;

    cv::Mat & display = context.display_bgr;

    // 绘制 accepted lights：绿色 rotatedRect + 端点
    for (const auto & light : data.accepted_lights) {
        cv::Point2f vertices[4];
        light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
        cv::circle(display, light.top, 1, cv::Scalar(255, 0, 255), -1);
        cv::circle(display, light.bottom, 1, cv::Scalar(255, 0, 255), -1);
    }

    // 绘制 rejected lights：红色 + 原因
    for (const auto & rejected : data.rejected_lights) {
        cv::Point2f vertices[4];
        rejected.light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        cv::putText(display, rejected.detail,
                    cv::Point(rejected.light.center.x + 5, rejected.light.center.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
}

} // namespace armor_detector::debug
