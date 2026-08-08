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
            ++sample_count_;

            if (record.success) {
                initial_error_sum_px_ += record.initial_reprojection_error_px;
                final_error_sum_px_ += record.final_reprojection_error_px;
                ++success_count_;
                abs_yaw_delta_sum_ += std::abs(record.delta_yaw_rad);
                xyz_delta_norm_sum_ += record.delta_xyz_gimbal.norm();
            } else {
                ++fail_count_;
            }
        }

        if (frame_count_ >= report_interval_) {
            reportAndReset();
        }
    }

    void DebugPoseRefineStats::reportAndReset() {
        const double avg_initial_error_px =
            success_count_ > 0 ? initial_error_sum_px_ / static_cast<double>(success_count_) : 0.0;
        const double avg_final_error_px =
            success_count_ > 0 ? final_error_sum_px_ / static_cast<double>(success_count_) : 0.0;
        const double avg_error_improvement = avg_initial_error_px - avg_final_error_px;
        const double avg_abs_yaw_delta =
            success_count_ > 0 ? abs_yaw_delta_sum_ / static_cast<double>(success_count_) : 0.0;
        const double avg_xyz_delta_norm =
            success_count_ > 0 ? xyz_delta_norm_sum_ / static_cast<double>(success_count_) : 0.0;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "最近" << frame_count_ << "帧: ";
        ss << "method=" << (method_.empty() ? "unknown" : method_);
        ss << " samples:" << sample_count_;
        ss << " success:" << success_count_;
        ss << " fail:" << fail_count_;
        ss << " init_err:" << avg_initial_error_px << "px";
        ss << " final_err:" << avg_final_error_px << "px";
        ss << " improv:" << avg_error_improvement << "px";
        ss << " |yaw|:" << avg_abs_yaw_delta << "rad";
        ss << " |xyz|:" << avg_xyz_delta_norm << "m";

        RCLCPP_INFO(rclcpp::get_logger("DebugPoseRefineStats"), "%s",
        ss.str().c_str());

        frame_count_ = 0;
        method_.clear();
        initial_error_sum_px_ = 0.0;
        final_error_sum_px_ = 0.0;
        sample_count_ = 0;
        success_count_ = 0;
        fail_count_ = 0;
        abs_yaw_delta_sum_ = 0.0;
        xyz_delta_norm_sum_ = 0.0;
    }

}