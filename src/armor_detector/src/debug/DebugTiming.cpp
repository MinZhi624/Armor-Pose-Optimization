#include "armor_detector/debug/DebugTiming.hpp"

#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <sstream>

namespace armor_detector::debug {

    namespace {
        const char *backendName(DetectionBackend backend) {
            return backend == DetectionBackend::YOLO ? "yolo" : "traditional";
        }
    } // namespace

    DebugTiming::DebugTiming(std::size_t report_interval) :
        stats_interval_(report_interval > 0 ? report_interval : 50) {
    }

    void DebugTiming::onFrameStart(DebugFrameContext & /*context*/) {
        frame_start_ = std::chrono::steady_clock::now();
        ++total_frame_count_;
    }

    void DebugTiming::onDetection(DebugFrameContext & /*context*/, const DetectionDebugData &data) {
        for (const auto &timing : data.timings) {
            stage_accum_[timing.name] += timing.elapsed_ms;
            stage_count_[timing.name] += 1;
        }

        ++detected_frame_count_;
        ++detection_window_count_;

        // 记录 detect 结束时间（供 onPoseSolved 计算 pose 耗时）
        detect_end_ = std::chrono::steady_clock::now();

        if (detection_window_count_ < stats_interval_) {
            return;
        }

        // 检测总耗时
        double detect_total_avg = 0.0;
        auto total_sum = stage_accum_.find("detect_total");
        auto total_count = stage_count_.find("detect_total");
        if (total_sum != stage_accum_.end() && total_count != stage_count_.end() && total_count->second > 0) {
            detect_total_avg = total_sum->second / static_cast<double>(total_count->second);
        }

        // 帧总耗时
        double frame_avg = (frame_window_count_ > 0)
            ? frame_time_sum_ms_ / static_cast<double>(frame_window_count_)
            : 0.0;

        // 构建单行输出
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "[" << backendName(data.backend) << "] 最近" << detection_window_count_ << "帧: ";
        ss << "[total:" << frame_avg << "ms] [";
        ss << "detect:" << detect_total_avg << "ms [";

        bool first = true;
        for (const auto &[name, total] : stage_accum_) {
            if (name == "detect_total") continue;
            auto count_it = stage_count_.find(name);
            if (count_it == stage_count_.end() || count_it->second == 0) continue;
            if (!first) ss << ", ";
            ss << name << ":" << (total / static_cast<double>(count_it->second)) << "ms";
            first = false;
        }

        ss << "]";

        if (pose_count_ > 0) {
            ss << ", pose:" << (pose_accum_ / static_cast<double>(pose_count_)) << "ms";
        }

        ss << "]";
        RCLCPP_INFO(rclcpp::get_logger("DebugTiming"), "%s", ss.str().c_str());

        // 重置
        stage_accum_.clear();
        stage_count_.clear();
        detection_window_count_ = 0;
        pose_accum_ = 0.0;
        pose_count_ = 0;
    }

    void DebugTiming::onPoseSolved(DebugFrameContext & /*context*/, const PoseDebugData & /*data*/) {
        auto now = std::chrono::steady_clock::now();
        double pose_ms = std::chrono::duration<double, std::milli>(now - detect_end_).count();
        pose_accum_ += pose_ms;
        ++pose_count_;
    }

    void DebugTiming::onFrameEnd(DebugFrameContext &context) {
        auto now = std::chrono::steady_clock::now();
        double last_frame_ms = std::chrono::duration<double, std::milli>(now - frame_start_).count();
        frame_time_sum_ms_ += last_frame_ms;
        ++frame_window_count_;

        if (frame_window_count_ >= stats_interval_) {
            frame_time_sum_ms_ = 0.0;
            frame_window_count_ = 0;
        }

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
        total_ss << std::fixed << std::setprecision(2) << last_frame_ms;
        drawText("Process: " + total_ss.str() + " ms", 1);
    }

} // namespace armor_detector::debug
