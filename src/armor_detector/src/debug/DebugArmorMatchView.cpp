#include "armor_detector/debug/DebugArmorMatch.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug {

    DebugArmorMatchView::DebugArmorMatchView(DebugGUI &gui, DebugLayerState &layer_state) :
        gui_(&gui), layer_state_(layer_state) {
    }

    void DebugArmorMatchView::onArmorMatch(DebugFrameContext &context, const ArmorMatchDebugData &data) {
        if (!gui_ || !gui_->enabled()) {
            return;
        }
        if (!layer_state_.enabled(DebugLayer::ARMOR_MATCH)) {
            return;
        }
        if (context.display_bgr.empty()) {
            return;
        }

        cv::Mat &display = context.display_bgr;

        // Draw accepted candidates: cyan four-point outline
        for (const auto &candidate : data.candidates) {
            const auto &corners = candidate.geometry.corners;
            for (int i = 0; i < 4; ++i) {
                cv::line(display, corners[i], corners[(i + 1) % 4], cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            }
        }

        // Draw rejected armors: red four-point outline + detail text
        for (const auto &rejected : data.rejected_armors) {
            const auto &corners = rejected.geometry.corners;
            for (int i = 0; i < 4; ++i) {
                cv::line(display, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            }
            // Center point for text
            cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;
            cv::putText(display,
                        rejected.detail,
                        cv::Point(static_cast<int>(center.x) + 5, static_cast<int>(center.y)),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.45,
                        cv::Scalar(0, 0, 255),
                        1,
                        cv::LINE_AA);
        }
    }

} // namespace armor_detector::debug
