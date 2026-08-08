#include "armor_detector/debug/DebugPoseRefine.hpp"

#include <opencv2/imgproc.hpp>

#include <array>

namespace armor_detector::debug {

    namespace {
        const cv::Scalar kBaselineColor(255, 0, 255);
        const cv::Scalar kPerturbationColor(0, 0, 255);
        constexpr std::array<const char *, 4> kPerturbationWindowNames = {
            "pose_perturb_dir_yaw",
            "pose_perturb_dir_pitch",
            "pose_perturb_distance",
            "pose_perturb_pose_yaw",
        };
        constexpr std::array<const char *, 4> kPerturbationLabels = {
            "dir_yaw",
            "dir_pitch",
            "distance",
            "pose_yaw",
        };

        cv::Point2f centerOf(const std::array<cv::Point2f, 4> &corners) {
            return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
        }

        void drawPolygon(cv::Mat &display, const std::array<cv::Point2f, 4> &corners, const cv::Scalar &color) {
            for (int i = 0; i < 4; ++i) {
                cv::line(display, corners[i], corners[(i + 1) % 4], color, 1, cv::LINE_AA);
            }
        }

        void drawCenterPoint(cv::Mat &display, const std::array<cv::Point2f, 4> &corners, const cv::Scalar &color) {
            const cv::Point2f center = centerOf(corners);
            cv::circle(display, center, 1, color, -1, cv::LINE_AA);
        }

        void clearPerturbationFrames(DebugGUI &gui) {
            for (const char *window_name : kPerturbationWindowNames) {
                gui.clearFrame(window_name);
            }
        }

        void drawOverlayLabel(cv::Mat &display, const char *perturbation_label) {
            cv::putText(display,
                        "baseline (purple) + " + std::string(perturbation_label) + " (red)",
                        cv::Point(10, 24),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(255, 255, 255),
                        2,
                        cv::LINE_AA);
        }
    } // namespace

    DebugPoseRefineView::DebugPoseRefineView(DebugGUI &gui, DebugLayerState &layer_state, bool show_perturbations) :
        gui_(&gui), layer_state_(layer_state), show_perturbations_(show_perturbations) {
    }

    void DebugPoseRefineView::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!gui_ || !gui_->enabled()) {
            return;
        }
        if (!layer_state_.enabled(DebugLayer::POSE_REFINE)) {
            clearPerturbationFrames(*gui_);
            return;
        }
        if (context.display_bgr.empty() || context.source_bgr.empty()) {
            clearPerturbationFrames(*gui_);
            return;
        }

        cv::Mat &display = context.display_bgr;
        if (!show_perturbations_) {
            clearPerturbationFrames(*gui_);
            for (const auto &record : data.refine_records) {
                if (record.success && record.has_projected_corners) {
                    drawPolygon(display, record.projected_corners, kBaselineColor);
                    drawCenterPoint(display, record.projected_corners, kBaselineColor);
                }
            }
            return;
        }

        std::array<cv::Mat, 4> perturbation_displays;
        for (std::size_t i = 0; i < perturbation_displays.size(); ++i) {
            perturbation_displays[i] = context.source_bgr.clone();
            drawOverlayLabel(perturbation_displays[i], kPerturbationLabels[i]);
        }

        bool has_perturbation = false;
        for (const auto &record : data.refine_records) {
            if (!record.success || !record.has_projected_corners) {
                continue;
            }

            drawPolygon(display, record.projected_corners, kBaselineColor);
            drawCenterPoint(display, record.projected_corners, kBaselineColor);

            if (!record.perturbation_projected_corners.available) {
                continue;
            }

            const auto &perturbed = record.perturbation_projected_corners;
            const std::array<const std::array<cv::Point2f, 4> *, 4> perturbation_corners = {
                &perturbed.dir_yaw_corners,
                &perturbed.dir_pitch_corners,
                &perturbed.distance_corners,
                &perturbed.pose_yaw_corners,
            };
            for (std::size_t i = 0; i < perturbation_displays.size(); ++i) {
                drawPolygon(perturbation_displays[i], record.projected_corners, kBaselineColor);
                drawCenterPoint(perturbation_displays[i], record.projected_corners, kBaselineColor);
                drawPolygon(perturbation_displays[i], *perturbation_corners[i], kPerturbationColor);
                drawCenterPoint(perturbation_displays[i], *perturbation_corners[i], kPerturbationColor);
            }
            has_perturbation = true;
        }

        if (!has_perturbation) {
            clearPerturbationFrames(*gui_);
            return;
        }
        for (std::size_t i = 0; i < perturbation_displays.size(); ++i) {
            gui_->setFrame(kPerturbationWindowNames[i], perturbation_displays[i]);
        }
    }

} // namespace armor_detector::debug
