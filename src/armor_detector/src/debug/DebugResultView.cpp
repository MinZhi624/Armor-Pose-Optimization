#include "armor_detector/debug/DebugResult.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug {

DebugResultView::DebugResultView(DebugGUI &gui, DebugLayerState &layer_state)
    : gui_(&gui), layer_state_(layer_state) {}

void DebugResultView::onDetection(
    DebugFrameContext &context, const DetectionDebugData &data) {
    if (!gui_ || !gui_->enabled()) return;
    if (!layer_state_.enabled(DebugLayer::RESULT)) return;
    if (context.display_bgr.empty()) return;

    cv::Mat &display = context.display_bgr;
    for (const auto &armor : data.final_armors) {
        const auto &c = armor.geometry.corners;
        cv::line(display, c[0], c[2], cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
        cv::line(display, c[1], c[3], cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
    }
}

} // namespace armor_detector::debug
