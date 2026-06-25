#include "armor_detector/debug/DebugPreprocess.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugPreprocessView::DebugPreprocessView(DebugGUI & gui) : gui_(&gui) {}

void DebugPreprocessView::onPreprocess(
    DebugFrameContext & context,
    const PreprocessDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;
    if (context.source_bgr.empty()) return;

    // 原图 resize 0.5
    cv::Mat src_half;
    cv::resize(context.source_bgr, src_half, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    // 三张单通道图转 BGR 后 resize 0.5
    auto toHalfBgr = [](const cv::Mat & src) -> cv::Mat {
        if (src.empty()) return {};
        cv::Mat bgr, half;
        cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
        cv::resize(bgr, half, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        return half;
    };

    cv::Mat gray_half = toHalfBgr(data.gray);
    cv::Mat binary_half = toHalfBgr(data.img_thre);
    cv::Mat color_half = toHalfBgr(data.color_mask);

    // 空图用黑图替代，保证 2x2 网格尺寸一致
    cv::Mat black = cv::Mat::zeros(src_half.size(), CV_8UC3);
    if (gray_half.empty()) gray_half = black.clone();
    if (binary_half.empty()) binary_half = black.clone();
    if (color_half.empty()) color_half = black.clone();

    // 2x2 网格: [source | gray] [binary | color_mask]
    cv::Mat top, bottom, combined;
    cv::hconcat(src_half, gray_half, top);
    cv::hconcat(binary_half, color_half, bottom);
    cv::vconcat(top, bottom, combined);

    // 文字标注（遵循 DebugGUI.md: FONT_HERSHEY_SIMPLEX, scale 0.60, thickness 2）
    int w = src_half.cols;
    int h = src_half.rows;
    cv::putText(combined, "source", cv::Point(10, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    cv::putText(combined, "gray", cv::Point(w + 10, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    cv::putText(combined, "binary", cv::Point(10, h + 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    cv::putText(combined, "color_mask", cv::Point(w + 10, h + 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);

    gui_->setFrame("preprocess_debug", combined);
}

} // namespace armor_detector::debug
