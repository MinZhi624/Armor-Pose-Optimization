#include "armor_detector/debug/DebugYolo.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug {

    DebugYoloView::DebugYoloView(DebugGUI &gui, DebugLayerState &layer_state) :
        gui_(&gui), layer_state_(layer_state) {
    }

    void DebugYoloView::onDetection(DebugFrameContext &context, const DetectionDebugData &data) {
        if (data.backend != DetectionBackend::YOLO) return;
        if (!gui_ || !gui_->enabled()) return;

        if (layer_state_.enabled(DebugLayer::DETECT_STAGE_1)) {
            showStage1(data);
        } else {
            gui_->clearFrame("yolo_stage1_letterbox");
        }

        if (context.display_bgr.empty()) {
            return;
        }

        if (layer_state_.enabled(DebugLayer::DETECT_STAGE_2)) {
            drawDetections(context.display_bgr, data.yolo.stage2_score_filtered,
                           cv::Scalar(0, 255, 255), "score");
        }

        if (layer_state_.enabled(DebugLayer::DETECT_STAGE_3)) {
            drawDetections(context.display_bgr, data.yolo.stage3_nms_kept,
                           cv::Scalar(0, 255, 0), "nms");
            drawRejected(context.display_bgr, data.yolo.stage3_nms_rejected);
            drawRejected(context.display_bgr, data.yolo.stage3_filter_rejected);
        }

        if (layer_state_.enabled(DebugLayer::DETECT_STAGE_4)) {
            drawDetections(context.display_bgr, data.final_armors,
                           cv::Scalar(255, 0, 255), "final");
        }
    }

    void DebugYoloView::showStage1(const DetectionDebugData &data) {
        if (data.yolo.stage1_letterbox.empty()) {
            return;
        }

        cv::Mat stage_view = data.yolo.stage1_letterbox;
        if (!data.yolo.stage1_source_roi.empty()) {
            cv::Mat source_resized;
            cv::resize(data.yolo.stage1_source_roi, source_resized, data.yolo.stage1_letterbox.size());
            cv::hconcat(source_resized, data.yolo.stage1_letterbox, stage_view);
            cv::putText(stage_view,
                        "source_roi",
                        cv::Point(10, 24),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.60,
                        cv::Scalar(0, 165, 255),
                        2,
                        cv::LINE_AA);
            cv::putText(stage_view,
                        "letterbox",
                        cv::Point(data.yolo.stage1_letterbox.cols + 10, 24),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.60,
                        cv::Scalar(0, 165, 255),
                        2,
                        cv::LINE_AA);
        }

        gui_->setFrame("yolo_stage1_letterbox", stage_view);
    }

    void DebugYoloView::drawDetections(cv::Mat &display,
                                        const std::vector<ClassifiedArmor> &armors,
                                        const cv::Scalar &color,
                                        const std::string &label) {
        for (size_t i = 0; i < armors.size(); ++i) {
            const auto &corners = armors[i].geometry.corners;
            // 画四边形
            for (int j = 0; j < 4; ++j) {
                cv::line(display, corners[j], corners[(j + 1) % 4], color, 2, cv::LINE_AA);
            }
            // 标注名称和置信度
            std::string text = label + " " +
                std::to_string(static_cast<int>(armors[i].classification.name)) +
                " " + std::to_string(armors[i].classification.confidence).substr(0, 4);
            cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
            cv::putText(display, text, cv::Point(center.x - 30, center.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
        }
    }

    void DebugYoloView::drawRejected(cv::Mat &display,
                                      const std::vector<RejectedArmor> &rejected) {
        for (const auto &rej : rejected) {
            const auto &corners = rej.geometry.corners;
            cv::Scalar color;
            if (rej.reason == DebugRejectReason::BAD_COLOR) {
                color = cv::Scalar(255, 0, 0);  // 蓝色：颜色不匹配
            } else if (rej.reason == DebugRejectReason::TYPE_MISMATCH) {
                color = cv::Scalar(0, 0, 255);  // 红色：不支持的类别
            } else if (rej.reason == DebugRejectReason::LOW_CONFIDENCE) {
                color = cv::Scalar(0, 165, 255);  // 橙色：低置信度
            } else {
                color = cv::Scalar(128, 128, 128);  // 灰色：NMS 抑制
            }
            for (int j = 0; j < 4; ++j) {
                cv::line(display, corners[j], corners[(j + 1) % 4], color, 1, cv::LINE_AA);
            }
            cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
            cv::putText(display, rej.detail,
                        cv::Point(center.x - 20, center.y + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv::LINE_AA);
        }
    }

} // namespace armor_detector::debug
