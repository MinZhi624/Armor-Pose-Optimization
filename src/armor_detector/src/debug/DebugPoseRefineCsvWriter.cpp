#include "armor_detector/debug/DebugPoseRefineCsvWriter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace armor_detector::debug {

    DebugPoseRefineCsvWriter::DebugPoseRefineCsvWriter(const std::string &root_dir,
                                                       const std::string &video,
                                                       const std::string &method) {
        // Generate timestamp
        const auto now = std::chrono::system_clock::now();
        const auto tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&tt);
        std::ostringstream ts;
        ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

        // Build path: <root_dir>/debug/pose_refine/log/<video>/<method>/<timestamp>.csv
        csv_path_ = std::filesystem::path(root_dir) / "debug" / "pose_refine" / "log" / video / method /
                    (ts.str() + ".csv");
        ensureDir();

        file_.open(csv_path_);
        if (file_.is_open()) {
            writeHeader();
            RCLCPP_INFO(rclcpp::get_logger("DebugPoseRefineCsvWriter"),
                        "CSV 路径: %s", csv_path_.string().c_str());
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("DebugPoseRefineCsvWriter"),
                         "无法创建 CSV 文件: %s", csv_path_.string().c_str());
        }
    }

    void DebugPoseRefineCsvWriter::ensureDir() {
        std::filesystem::create_directories(csv_path_.parent_path());
    }

    void DebugPoseRefineCsvWriter::writeHeader() {
        file_ << "frame_index,stamp_sec,stamp_nanosec,"
              << "armor_index,armor_name,armor_type,confidence,"
              << "center_x_px,center_y_px,"
              << "method,success,"
              << "initial_x_m,initial_y_m,initial_z_m,"
              << "final_x_m,final_y_m,final_z_m,"
              << "delta_x_m,delta_y_m,delta_z_m,"
              << "initial_yaw_rad,final_yaw_rad,delta_yaw_rad,"
              << "initial_error_px,final_error_px,delta_error_px\n";
    }

    void DebugPoseRefineCsvWriter::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!file_.is_open()) {
            return;
        }

        for (const auto &record : data.refine_records) {
            file_ << std::fixed << std::setprecision(6)
                  << context.frame_index << ","
                  << context.stamp.sec << "," << context.stamp.nanosec << ","
                  << record.armor_index << ","
                  << record.armor_name << ","
                  << record.armor_type << ","
                  << record.confidence << ","
                  << record.center_x_px << "," << record.center_y_px << ","
                  << record.method << ","
                  << (record.success ? "true" : "false") << ","
                  << record.initial_xyz_gimbal.x() << ","
                  << record.initial_xyz_gimbal.y() << ","
                  << record.initial_xyz_gimbal.z() << ","
                  << record.final_xyz_gimbal.x() << ","
                  << record.final_xyz_gimbal.y() << ","
                  << record.final_xyz_gimbal.z() << ","
                  << record.delta_xyz_gimbal.x() << ","
                  << record.delta_xyz_gimbal.y() << ","
                  << record.delta_xyz_gimbal.z() << ","
                  << record.initial_yaw_rad << ","
                  << record.final_yaw_rad << ","
                  << record.delta_yaw_rad << ","
                  << record.initial_reprojection_error_px << ","
                  << record.final_reprojection_error_px << ","
                  << record.delta_reprojection_error_px << "\n";
        }
    }

} // namespace armor_detector::debug
