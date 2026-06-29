#include "armor_detector/debug/DebugCornerCorrection.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug {

    DebugCornerCorrectionView::DebugCornerCorrectionView(DebugGUI &gui, DebugLayerState &layer_state)
        : gui_(&gui), layer_state_(layer_state) {
    }

    void DebugCornerCorrectionView::onCornerCorrection(
        DebugFrameContext &context, const CornerCorrectionDebugData &data) {
        if (!gui_ || !gui_->enabled()) return;
        if (!layer_state_.enabled(DebugLayer::CORNER_CORRECTION)) {
            if (was_shown_) {
                gui_->clearFrame("corner_pca_debug");
                was_shown_ = false;
            }
            return;
        }
        if (context.display_bgr.empty()) return;

        was_shown_ = true;

        cv::Mat &display = context.display_bgr;

        for (const auto &record : data.records) {
            if (record.corrected) {
                for (int i = 0; i < 4; ++i)
                    cv::circle(display, record.original_corners[i], 0, cv::Scalar(0, 255, 0), -1);
                for (int i = 0; i < 4; ++i)
                    cv::circle(display, record.output_corners[i], 0, cv::Scalar(255, 0, 255), -1);
                for (int i = 0; i < 4; ++i)
                    cv::line(display, record.original_corners[i], record.output_corners[i],
                             cv::Scalar(192, 192, 192), 1, cv::LINE_AA);
                if (record.has_raw_lights) {
                    cv::circle(display, record.left_raw_light.top, 0, cv::Scalar(255, 255, 0), -1);
                    cv::circle(display, record.left_raw_light.bottom, 0, cv::Scalar(255, 255, 0), -1);
                    cv::circle(display, record.right_raw_light.top, 0, cv::Scalar(255, 255, 0), -1);
                    cv::circle(display, record.right_raw_light.bottom, 0, cv::Scalar(255, 255, 0), -1);
                }
                if (record.has_output_lights) {
                    cv::circle(display, record.left_output_light.top, 0, cv::Scalar(255, 0, 128), -1);
                    cv::circle(display, record.left_output_light.bottom, 0, cv::Scalar(255, 0, 128), -1);
                    cv::circle(display, record.right_output_light.top, 0, cv::Scalar(255, 0, 128), -1);
                    cv::circle(display, record.right_output_light.bottom, 0, cv::Scalar(255, 0, 128), -1);
                }
            } else {
                for (int i = 0; i < 4; ++i)
                    cv::circle(display, record.original_corners[i], 0, cv::Scalar(0, 255, 0), -1);
                cv::Point2f center = (record.original_corners[0] + record.original_corners[1] +
                                      record.original_corners[2] + record.original_corners[3]) * 0.25f;
                cv::putText(display, record.detail,
                            cv::Point(static_cast<int>(center.x) - 20, static_cast<int>(center.y) + 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
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
            cv::Mat row2_left = record.left_pca_viz.empty()
                                    ? cv::Mat::zeros(record.left_gray_roi.size(), CV_8UC3)
                                    : record.left_pca_viz;
            cv::Mat row2_right = record.right_pca_viz.empty()
                                     ? cv::Mat::zeros(record.right_gray_roi.size(), CV_8UC3)
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
