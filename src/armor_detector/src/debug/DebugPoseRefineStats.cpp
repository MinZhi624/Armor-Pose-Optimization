#include "armor_detector/debug/DebugPoseRefineStats.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <sstream>

namespace armor_detector::debug {
    DebugPoseRefineStats::DebugPoseRefineStats(std::size_t report_interval) :
        report_interval_(report_interval > 0 ? report_interval : 50) {
    }

    void DebugPoseRefineStats::onPoseSolved(DebugFrameContext & /*context*/, const PoseDebugData &data) 
    {
        ++frame_count_;

        for (const auto & record : data.refine_records) {
            method_ = record.method;

            if (record.success) {
                error_sum_px_ += record.reprojection_error_px;
                ++success_count_;
            } else {
                ++fail_count_;
            }

            if (frame_count_ < report_interval_) {
                return;
            }
            
            reportAndReset();
        }
    }

    void DebugPoseRefineStats::reportAndReset() {
        const double avg_error_px =
            success_count_ > 0 ? error_sum_px_ / static_cast<double>(success_count_) :
            0.0;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "最近" << frame_count_ << "帧: ";
        ss << "method=" << (method_.empty() ? "unknown" : method_);
        ss << " err:" << avg_error_px << "px";
        ss << " success:" << success_count_;
        ss << " fail:" << fail_count_;

        RCLCPP_INFO(rclcpp::get_logger("DebugPoseRefineStats"), "%s",
        ss.str().c_str());

        frame_count_ = 0;
        method_.clear();
        error_sum_px_ = 0.0;
        success_count_ = 0;
        fail_count_ = 0;
    }

}