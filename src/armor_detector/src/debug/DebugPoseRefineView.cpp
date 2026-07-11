#include "armor_detector/debug/DebugPoseRefine.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug {

    namespace {
        const cv::Scalar kProjectedCornerColor(255, 255, 0);

        cv::Point2f centerOf(const std::array<cv::Point2f, 4> &corners) {
            return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
        }

        void drawPolygon(cv::Mat &display, const std::array<cv::Point2f, 4> &corners) {
            for (int i = 0; i < 4; ++i) {
                cv::line(display, corners[i], corners[(i + 1) % 4], kProjectedCornerColor, 1, cv::LINE_AA);
            }
        }

        void drawCenterPoint(cv::Mat &display, const std::array<cv::Point2f, 4> &corners) {
            const cv::Point2f center = centerOf(corners);
            cv::circle(display, center, 1, kProjectedCornerColor, -1, cv::LINE_AA);
        }
    } // namespace

    DebugPoseRefineView::DebugPoseRefineView(DebugGUI &gui, DebugLayerState &layer_state) :
        gui_(&gui), layer_state_(layer_state) {
    }

    void DebugPoseRefineView::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!gui_ || !gui_->enabled()) {
            return;
        }
        if (!layer_state_.enabled(DebugLayer::POSE_REFINE)) {
            return;
        }
        if (context.display_bgr.empty()) {
            return;
        }

        cv::Mat &display = context.display_bgr;
        for (const auto &record : data.refine_records) {
            if (!record.success || !record.has_projected_corners) {
                continue;
            }

            drawPolygon(display, record.projected_corners);
            drawCenterPoint(display, record.projected_corners);
        }
    }

} // namespace armor_detector::debug
