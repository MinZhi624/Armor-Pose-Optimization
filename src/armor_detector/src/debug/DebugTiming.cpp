#include "armor_detector/debug/DebugTiming.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <sstream>

namespace armor_detector::debug {

    DebugTiming::DebugTiming(std::size_t report_interval) :
        report_interval_(report_interval > 0 ? report_interval : 50) {
    }

    void DebugTiming::onFrameStart(DebugFrameContext & /*context*/) {
        ++frame_count_;
        frame_start_ = std::chrono::steady_clock::now();
        last_mark_ = frame_start_;
        last_stage_ms_.clear();
    }

    void DebugTiming::mark(const std::string &stage_name) {
        auto now = std::chrono::steady_clock::now();
        double dt_ms = std::chrono::duration<double, std::milli>(now - last_mark_).count();
        stage_time_sums_[stage_name] += dt_ms;
        last_stage_ms_.emplace_back(stage_name, dt_ms);
        last_mark_ = now;
    }

    void DebugTiming::onPreprocess(DebugFrameContext & /*context*/, const PreprocessDebugData & /*data*/) {
        mark("preprocess");
    }

    void DebugTiming::onLights(DebugFrameContext & /*context*/, const LightDebugData & /*data*/) {
        mark("lights");
    }

    void DebugTiming::onArmorMatch(DebugFrameContext & /*context*/, const ArmorMatchDebugData & /*data*/) {
        mark("armor_match");
    }

    void DebugTiming::onClassification(DebugFrameContext & /*context*/, const ClassificationDebugData & /*data*/) {
        mark("classification");
    }

    void DebugTiming::onPoseSolved(DebugFrameContext & /*context*/, const PoseDebugData & /*data*/) {
        mark("pose");
    }

    void DebugTiming::onFrameEnd(DebugFrameContext &context) {
        auto now = std::chrono::steady_clock::now();
        last_frame_ms_ = std::chrono::duration<double, std::milli>(now - frame_start_).count();
        frame_time_sum_ms_ += last_frame_ms_;

        // 每 report_interval 帧打印平均耗时（无论有无 GUI）
        if (frame_count_ > 0 && frame_count_ % report_interval_ == 0) {
            std::ostringstream marks_ss;
            for (const auto &[name, sum] : stage_time_sums_) {
                double avg = sum / static_cast<double>(report_interval_);
                marks_ss << name << ":" << std::fixed << std::setprecision(1) << avg << "ms ";
            }
            std::string marks_str = marks_ss.str();
            if (!marks_str.empty()) {
                marks_str.pop_back();
            }

            double avg_total = frame_time_sum_ms_ / static_cast<double>(report_interval_);
            RCLCPP_INFO(rclcpp::get_logger("DEBUG_TIMING"), "【平均用时】总:%.1fms [%s]", avg_total, marks_str.c_str());

            frame_time_sum_ms_ = 0.0;
            for (auto &[k, v] : stage_time_sums_) {
                v = 0.0;
            }
        }

        // GUI 绘制：仅 display_bgr 非空时
        if (context.display_bgr.empty()) {
            return;
        }

        auto drawText = [&](const std::string &text, int line_index) {
            int y = 24 * line_index;
            cv::putText(context.display_bgr,
                        text,
                        cv::Point(10, y),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.60,
                        cv::Scalar(0, 165, 255),
                        2,
                        cv::LINE_AA);
        };

        std::ostringstream total_ss;
        total_ss << std::fixed << std::setprecision(2) << last_frame_ms_;
        drawText("Process: " + total_ss.str() + " ms", 1);
    }

} // namespace armor_detector::debug
