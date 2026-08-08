#include "armor_detector/debug/DebugCornerCorrection.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace armor_detector::debug {

    namespace {
        cv::Point2f centerOf(const std::array<cv::Point2f, 4> &corners) {
            return (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
        }

        void drawCornerPoints(cv::Mat &display, const std::array<cv::Point2f, 4> &corners, const cv::Scalar &color) {
            for (const auto &corner : corners) {
                cv::circle(display, corner, 0, color, -1);
            }
        }

        void drawPolygon(cv::Mat &display,
                         const std::array<cv::Point2f, 4> &corners,
                         const cv::Scalar &color,
                         int thickness) {
            for (int i = 0; i < 4; ++i) {
                cv::line(display, corners[i], corners[(i + 1) % 4], color, thickness, cv::LINE_AA);
            }
        }

        void
        drawLightEndpoints(cv::Mat &display, const LightBar &left, const LightBar &right, const cv::Scalar &color) {
            cv::circle(display, left.top, 0, color, -1);
            cv::circle(display, left.bottom, 0, color, -1);
            cv::circle(display, right.top, 0, color, -1);
            cv::circle(display, right.bottom, 0, color, -1);
        }

        void drawDetail(cv::Mat &display,
                        const std::array<cv::Point2f, 4> &corners,
                        const std::string &detail,
                        const cv::Scalar &color) {
            if (detail.empty()) {
                return;
            }
            cv::Point2f center = centerOf(corners);
            cv::putText(display,
                        detail,
                        cv::Point(static_cast<int>(center.x) - 20, static_cast<int>(center.y) + 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.35,
                        color,
                        1,
                        cv::LINE_AA);
        }
    } // namespace

    DebugCornerCorrectionView::DebugCornerCorrectionView(DebugGUI &gui, DebugLayerState &layer_state) :
        gui_(&gui), layer_state_(layer_state) {
    }

    void DebugCornerCorrectionView::onCornerCorrection(DebugFrameContext &context,
                                                       const CornerCorrectionDebugData &data) {
        if (!gui_ || !gui_->enabled())
            return;
        if (!layer_state_.enabled(DebugLayer::CORNER_CORRECTION)) {
            if (was_shown_) {
                gui_->clearFrame("corner_pca_debug");
                was_shown_ = false;
            }
            return;
        }
        if (context.display_bgr.empty())
            return;

        was_shown_ = true;

        cv::Mat &display = context.display_bgr;

        for (const auto &record : data.records) {
            if (!record.accepted) {
                const auto &rejected_corners = record.corrected ? record.output_corners : record.original_corners;
                drawCornerPoints(display, record.original_corners, cv::Scalar(0, 255, 0));
                drawPolygon(display, rejected_corners, cv::Scalar(0, 80, 255), 1);
                drawCornerPoints(display, rejected_corners, cv::Scalar(0, 0, 255));
                if (record.has_raw_lights) {
                    drawLightEndpoints(display, record.left_raw_light, record.right_raw_light, cv::Scalar(255, 255, 0));
                }
                if (record.has_output_lights) {
                    drawLightEndpoints(
                        display, record.left_output_light, record.right_output_light, cv::Scalar(255, 0, 128));
                }
                drawDetail(display, rejected_corners, record.detail, cv::Scalar(0, 80, 255));
                continue;
            }

            if (record.corrected) {
                drawCornerPoints(display, record.original_corners, cv::Scalar(0, 255, 0));
                drawCornerPoints(display, record.output_corners, cv::Scalar(255, 0, 255));
                for (int i = 0; i < 4; ++i)
                    cv::line(display,
                             record.original_corners[i],
                             record.output_corners[i],
                             cv::Scalar(192, 192, 192),
                             1,
                             cv::LINE_AA);
                if (record.has_raw_lights) {
                    drawLightEndpoints(display, record.left_raw_light, record.right_raw_light, cv::Scalar(255, 255, 0));
                }
                if (record.has_output_lights) {
                    drawLightEndpoints(
                        display, record.left_output_light, record.right_output_light, cv::Scalar(255, 0, 128));
                }
            }
            else {
                drawCornerPoints(display, record.original_corners, cv::Scalar(0, 255, 0));
                drawPolygon(display, record.original_corners, cv::Scalar(128, 128, 128), 1);
                drawDetail(display, record.original_corners, record.detail, cv::Scalar(128, 128, 128));
            }
        }

        // ---- PCA debug window: gray ROI + overlay per armor ----
        std::vector<cv::Mat> pca_group_rows;
        int max_group_w = 0;
        for (std::size_t i = 0; i < data.records.size(); ++i) {
            const auto &record = data.records[i];
            if (record.left_gray_roi.empty() && record.left_pca_viz.empty())
                continue;

            // Row 1: gray ROI (left | right)
            cv::Mat row1_left, row1_right;
            if (!record.left_gray_roi.empty())
                cv::cvtColor(record.left_gray_roi, row1_left, cv::COLOR_GRAY2BGR);
            else
                row1_left = cv::Mat::zeros(record.left_pca_viz.size(), CV_8UC3);
            if (!record.right_gray_roi.empty())
                cv::cvtColor(record.right_gray_roi, row1_right, cv::COLOR_GRAY2BGR);
            else
                row1_right = cv::Mat::zeros(record.right_pca_viz.size(), CV_8UC3);

            // Row 2: PCA viz (left | right)
            cv::Mat row2_left = record.left_pca_viz.empty() ? cv::Mat::zeros(record.left_gray_roi.size(), CV_8UC3)
                                                            : record.left_pca_viz;
            cv::Mat row2_right = record.right_pca_viz.empty() ? cv::Mat::zeros(record.right_gray_roi.size(), CV_8UC3)
                                                              : record.right_pca_viz;

            // Pad left/right in each row to same height
            auto padPair = [](cv::Mat &a, cv::Mat &b) {
                int h = std::max(a.rows, b.rows);
                auto padTo = [h](cv::Mat &img) {
                    if (img.rows < h) {
                        cv::Mat pad = cv::Mat::zeros(h - img.rows, img.cols, CV_8UC3);
                        cv::vconcat(img, pad, img);
                    }
                };
                padTo(a);
                padTo(b);
            };
            padPair(row1_left, row1_right);
            padPair(row2_left, row2_right);

            cv::Mat row1, row2;
            cv::hconcat(row1_left, row1_right, row1);
            cv::hconcat(row2_left, row2_right, row2);

            // Pad two rows to same width
            int rw = std::max(row1.cols, row2.cols);
            auto padToWidth = [rw](cv::Mat &img) {
                if (img.cols < rw) {
                    cv::Mat pad = cv::Mat::zeros(img.rows, rw - img.cols, CV_8UC3);
                    cv::hconcat(img, pad, img);
                }
            };
            padToWidth(row1);
            padToWidth(row2);

            max_group_w = std::max(max_group_w, row1.cols);
            max_group_w = std::max(max_group_w, row2.cols);
            pca_group_rows.push_back(row1);
            pca_group_rows.push_back(row2);
        }

        if (!pca_group_rows.empty()) {
            for (auto &row : pca_group_rows)
                if (row.cols < max_group_w) {
                    cv::Mat pad = cv::Mat::zeros(row.rows, max_group_w - row.cols, CV_8UC3);
                    cv::hconcat(row, pad, row);
                }
            cv::Mat combined;
            cv::vconcat(pca_group_rows, combined);
            cv::resize(combined, combined, cv::Size(), 3.0, 3.0, cv::INTER_NEAREST);
            gui_->setFrame("corner_pca_debug", combined);
        }
    }

} // namespace armor_detector::debug
