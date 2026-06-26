#include "armor_detector/debug/DebugClassification.hpp"

#include <opencv2/imgproc.hpp>

#include <sstream>

namespace armor_detector::debug {

    DebugClassificationView::DebugClassificationView(DebugGUI &gui, DebugLayerState &layer_state) :
        gui_(&gui), layer_state_(layer_state) {
    }

    void DebugClassificationView::onDetection(DebugFrameContext &context, const DetectionDebugData &data) {
        if (data.backend != DetectionBackend::TRADITIONAL) return;
        if (!gui_ || !gui_->enabled()) return;

        const auto &classification = data.traditional.classification;

        if (!layer_state_.enabled(DebugLayer::DETECT_STAGE_4)) {
            gui_->clearFrame("number_rois");
            return;
        }

        if (context.display_bgr.empty()) {
            return;
        }

        cv::Mat &display = context.display_bgr;

        // Draw classification labels on display image
        for (const auto &armor : classification.classified_armors) {
            const auto &corners = armor.geometry.corners;
            cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;

            std::ostringstream oss;
            oss << static_cast<int>(armor.classification.name) << " "
                << static_cast<int>(armor.classification.confidence * 100) << "%";
            cv::putText(display,
                        oss.str(),
                        cv::Point(static_cast<int>(center.x) - 20, static_cast<int>(center.y) - 10),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 0),
                        2,
                        cv::LINE_AA);
        }

        // Concatenate number ROIs into a single window
        if (classification.number_rois.empty()) {
            gui_->clearFrame("number_rois");
            return;
        }

        // Convert all ROIs to BGR for consistent hconcat
        std::vector<cv::Mat> bgr_rois;
        for (const auto &roi : classification.number_rois) {
            cv::Mat bgr;
            if (roi.channels() == 1) {
                cv::cvtColor(roi, bgr, cv::COLOR_GRAY2BGR);
            }
            else {
                bgr = roi;
            }
            bgr_rois.push_back(bgr);
        }

        cv::Mat concatenated;
        cv::hconcat(bgr_rois, concatenated);
        gui_->setFrame("number_rois", concatenated);
    }

} // namespace armor_detector::debug
