#include "armor_detector/debug/DebugPoseRefineCsvWriter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace armor_detector::debug {

    DebugPoseRefineCsvWriter::DebugPoseRefineCsvWriter(const std::string &root_dir,
                                                       const std::string &video,
                                                       const std::string &corner_method,
                                                       const std::string &refine_method) :
        corner_method_(corner_method) {
        // Generate timestamp
        const auto now = std::chrono::system_clock::now();
        const auto tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&tt);
        std::ostringstream ts;
        ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

        // Build path: <root_dir>/debug/pose_refine/log/<video>/<corner_method>/<refine_method>/<timestamp>.csv
        csv_path_ = std::filesystem::path(root_dir) / "debug" / "pose_refine" / "log" / video / corner_method /
            refine_method / (ts.str() + ".csv");
        ensureDir();

        file_.open(csv_path_);
        if (file_.is_open()) {
            writeHeader();
            RCLCPP_INFO(rclcpp::get_logger("DebugPoseRefineCsvWriter"),
                        "CSV 路径: %s (corner=%s, refine=%s)",
                        csv_path_.string().c_str(),
                        corner_method_.c_str(),
                        refine_method.c_str());
        }
        else {
            RCLCPP_ERROR(
                rclcpp::get_logger("DebugPoseRefineCsvWriter"), "无法创建 CSV 文件: %s", csv_path_.string().c_str());
        }
    }

    void DebugPoseRefineCsvWriter::ensureDir() {
        std::filesystem::create_directories(csv_path_.parent_path());
    }

    void DebugPoseRefineCsvWriter::writeHeader() {
        file_ << "frame_index,stamp_sec,stamp_nanosec," << "armor_index,armor_name,armor_type,confidence,"
              << "center_x_px,center_y_px," << "corner_method,method,success," << "initial_x_m,initial_y_m,initial_z_m,"
              << "final_x_m,final_y_m,final_z_m," << "delta_x_m,delta_y_m,delta_z_m,"
              << "initial_dir_yaw_rad,final_dir_yaw_rad,delta_dir_yaw_rad,"
              << "initial_dir_pitch_rad,final_dir_pitch_rad,delta_dir_pitch_rad,"
              << "initial_distance_m,final_distance_m,delta_distance_m,"
              << "initial_pose_yaw_rad,final_pose_yaw_rad,delta_pose_yaw_rad,"
              << "initial_reproj_sum_px,final_reproj_sum_px,delta_reproj_sum_px,"
              << "initial_reproj_mean_px,final_reproj_mean_px,delta_reproj_mean_px,"
              << "ba_model_initial_reproj_mean_px,ba_model_final_reproj_mean_px,"
              << "initial_cost,final_cost,delta_cost,num_iterations,termination_type\n";
    }

    void DebugPoseRefineCsvWriter::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!file_.is_open()) {
            return;
        }

        for (const auto &record : data.refine_records) {
            const auto writeCost = [&](double cost) {
                if (record.has_solver_summary) {
                    file_ << cost;
                }
            };

            file_ << std::fixed << std::setprecision(6) << context.frame_index << "," << context.stamp.sec << ","
                  << context.stamp.nanosec << "," << record.armor_index << "," << record.armor_name << ","
                  << record.armor_type << "," << record.confidence << "," << record.center_x_px << ","
                  << record.center_y_px << "," << corner_method_ << "," << record.method << ","
                  << (record.success ? "true" : "false") << "," << record.initial_xyz_gimbal.x() << ","
                  << record.initial_xyz_gimbal.y() << "," << record.initial_xyz_gimbal.z() << ","
                  << record.final_xyz_gimbal.x() << "," << record.final_xyz_gimbal.y() << ","
                  << record.final_xyz_gimbal.z() << "," << record.delta_xyz_gimbal.x() << ","
                  << record.delta_xyz_gimbal.y() << "," << record.delta_xyz_gimbal.z() << ","
                  << record.initial_dir_yaw_rad << "," << record.final_dir_yaw_rad << "," << record.delta_dir_yaw_rad
                  << "," << record.initial_dir_pitch_rad << "," << record.final_dir_pitch_rad << ","
                  << record.delta_dir_pitch_rad << "," << record.initial_distance_m << "," << record.final_distance_m
                  << "," << record.delta_distance_m << "," << record.initial_yaw_rad << "," << record.final_yaw_rad
                  << "," << record.delta_yaw_rad << "," << record.initial_reproj_sum_px << ","
                  << record.final_reproj_sum_px << "," << record.delta_reproj_sum_px << ","
                  << record.initial_reproj_mean_px << "," << record.final_reproj_mean_px << ","
                  << record.delta_reproj_mean_px << "," << record.ba_model_initial_reproj_mean_px << ","
                  << record.ba_model_final_reproj_mean_px << ",";
            writeCost(record.initial_cost);
            file_ << ",";
            writeCost(record.final_cost);
            file_ << ",";
            writeCost(record.delta_cost);
            file_ << "," << record.num_iterations << "," << record.termination_type << "\n";
        }
    }

} // namespace armor_detector::debug
