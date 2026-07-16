#include "armor_detector/debug/DebugPoseLandscapeCsvWriter.hpp"

#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <system_error>

#include "armor_detector/tools/angle.hpp"

namespace armor_detector::debug {

    namespace {
        constexpr const char *kLoggerName = "DebugPoseLandscapeCsvWriter";

        std::string timestampString() {
            const auto now = std::chrono::system_clock::now();
            const auto time = std::chrono::system_clock::to_time_t(now);
            std::tm local_time{};
            localtime_r(&time, &local_time);
            const auto milliseconds =
                std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

            std::ostringstream stream;
            stream << std::put_time(&local_time, "%Y%m%d_%H%M%S") << "_" << std::setfill('0') << std::setw(3)
                   << milliseconds;
            return stream.str();
        }

        std::string sanitizePathComponent(std::string value) {
            if (value.empty()) {
                return "manual";
            }
            for (char &character : value) {
                const auto unsigned_character = static_cast<unsigned char>(character);
                if (!std::isalnum(unsigned_character) && character != '-' && character != '_') {
                    character = '_';
                }
            }
            return value;
        }

        std::string csvEscape(const std::string &value) {
            if (value.find_first_of(",\"\r\n") == std::string::npos) {
                return value;
            }
            std::string escaped = "\"";
            for (char character : value) {
                if (character == '\"') {
                    escaped += "\"\"";
                }
                else {
                    escaped += character;
                }
            }
            escaped += "\"";
            return escaped;
        }

        std::string yamlQuote(const std::string &value) {
            std::string quoted = "\"";
            for (char character : value) {
                switch (character) {
                    case '\\':
                        quoted += "\\\\";
                        break;
                    case '\"':
                        quoted += "\\\"";
                        break;
                    case '\n':
                        quoted += "\\n";
                        break;
                    case '\r':
                        quoted += "\\r";
                        break;
                    case '\t':
                        quoted += "\\t";
                        break;
                    default:
                        quoted += character;
                        break;
                }
            }
            quoted += "\"";
            return quoted;
        }

        void writeNumber(std::ostream &stream, double value) {
            if (std::isfinite(value)) {
                stream << value;
            }
        }

        void writeYamlNumber(std::ostream &stream, double value) {
            if (std::isfinite(value)) {
                stream << value;
            }
            else {
                stream << "null";
            }
        }

        const char *boolString(bool value) {
            return value ? "true" : "false";
        }

        const PoseLandscapeMarker *findMarker(const PoseLandscapeSample &sample, const std::string &name) {
            for (const auto &marker : sample.markers) {
                if (marker.name == name) {
                    return &marker;
                }
            }
            return nullptr;
        }

        void writeIndexMarker(std::ostream &stream, const PoseLandscapeMarker *marker) {
            if (marker == nullptr) {
                stream << ",,,,,";
                return;
            }
            writeNumber(stream, marker->distance_m);
            stream << ",";
            writeNumber(stream, marker->pose_yaw_rad);
            stream << "," << csvEscape(marker->status) << ",";
            writeNumber(stream, marker->actual_metric.cost);
            stream << ",";
            writeNumber(stream, marker->slice_metric.cost);
            stream << ",";
        }

        void writeYamlMetric(std::ostream &stream, const std::string &indent, const PoseLandscapeMetric &metric) {
            stream << indent << "valid: " << boolString(metric.valid) << "\n";
            stream << indent << "status: " << yamlQuote(metric.status) << "\n";
            stream << indent << "cost: ";
            writeYamlNumber(stream, metric.cost);
            stream << "\n";
            stream << indent << "mean_residual_px: ";
            writeYamlNumber(stream, metric.mean_residual_px);
            stream << "\n";
        }

        void writeYamlMarker(std::ostream &stream, const std::string &name, const PoseLandscapeMarker *marker) {
            stream << "  " << name << ":\n";
            if (marker == nullptr) {
                stream << "    available: false\n";
                stream << "    status: \"not_present\"\n";
                return;
            }
            stream << "    available: " << boolString(marker->available) << "\n";
            stream << "    inside_grid: " << boolString(marker->inside_grid) << "\n";
            stream << "    status: " << yamlQuote(marker->status) << "\n";
            stream << "    distance_m: ";
            writeYamlNumber(stream, marker->distance_m);
            stream << "\n";
            stream << "    pose_yaw_rad: ";
            writeYamlNumber(stream, marker->pose_yaw_rad);
            stream << "\n";
            stream << "    pose_yaw_deg: ";
            writeYamlNumber(stream,
                            std::isfinite(marker->pose_yaw_rad) ? tools::radToDeg(marker->pose_yaw_rad)
                                                                : std::numeric_limits<double>::quiet_NaN());
            stream << "\n";
            stream << "    direction_delta_yaw_rad: ";
            writeYamlNumber(stream, marker->direction_delta_yaw_rad);
            stream << "\n";
            stream << "    direction_delta_pitch_rad: ";
            writeYamlNumber(stream, marker->direction_delta_pitch_rad);
            stream << "\n";
            stream << "    actual:\n";
            writeYamlMetric(stream, "      ", marker->actual_metric);
            stream << "    fixed_pnp_slice:\n";
            writeYamlMetric(stream, "      ", marker->slice_metric);
        }

        bool writeGridCsv(const std::filesystem::path &path, const PoseLandscapeSample &sample) {
            std::ofstream file(path);
            if (!file.is_open()) {
                return false;
            }
            file << std::setprecision(17);
            file << "distance_index,pose_yaw_index,distance_m,pose_yaw_rad,pose_yaw_deg,cost,mean_residual_px,status\n";
            for (const auto &point : sample.grid) {
                file << point.distance_index << "," << point.pose_yaw_index << ",";
                writeNumber(file, point.distance_m);
                file << ",";
                writeNumber(file, point.pose_yaw_rad);
                file << ",";
                writeNumber(file,
                            std::isfinite(point.pose_yaw_rad) ? tools::radToDeg(point.pose_yaw_rad)
                                                              : std::numeric_limits<double>::quiet_NaN());
                file << ",";
                writeNumber(file, point.cost);
                file << ",";
                writeNumber(file, point.mean_residual_px);
                file << "," << csvEscape(point.status) << "\n";
            }
            return file.good();
        }

        bool writeMarkersCsv(const std::filesystem::path &path, const PoseLandscapeSample &sample) {
            std::ofstream file(path);
            if (!file.is_open()) {
                return false;
            }
            file << std::setprecision(17);
            file << "name,available,inside_grid,status,distance_m,pose_yaw_rad,pose_yaw_deg,"
                 << "actual_valid,actual_status,actual_cost,actual_mean_residual_px,"
                 << "slice_valid,slice_status,slice_cost,slice_mean_residual_px,"
                 << "direction_delta_yaw_rad,direction_delta_pitch_rad\n";
            for (const auto &marker : sample.markers) {
                file << csvEscape(marker.name) << "," << boolString(marker.available) << ","
                     << boolString(marker.inside_grid) << "," << csvEscape(marker.status) << ",";
                writeNumber(file, marker.distance_m);
                file << ",";
                writeNumber(file, marker.pose_yaw_rad);
                file << ",";
                writeNumber(file,
                            std::isfinite(marker.pose_yaw_rad) ? tools::radToDeg(marker.pose_yaw_rad)
                                                               : std::numeric_limits<double>::quiet_NaN());
                file << "," << boolString(marker.actual_metric.valid) << "," << csvEscape(marker.actual_metric.status)
                     << ",";
                writeNumber(file, marker.actual_metric.cost);
                file << ",";
                writeNumber(file, marker.actual_metric.mean_residual_px);
                file << "," << boolString(marker.slice_metric.valid) << "," << csvEscape(marker.slice_metric.status)
                     << ",";
                writeNumber(file, marker.slice_metric.cost);
                file << ",";
                writeNumber(file, marker.slice_metric.mean_residual_px);
                file << ",";
                writeNumber(file, marker.direction_delta_yaw_rad);
                file << ",";
                writeNumber(file, marker.direction_delta_pitch_rad);
                file << "\n";
            }
            return file.good();
        }

        bool writeMetadataYaml(const std::filesystem::path &path,
                               const DebugFrameContext &context,
                               const PoseLandscapeSample &sample,
                               const std::filesystem::path &run_root) {
            std::ofstream file(path);
            if (!file.is_open()) {
                return false;
            }
            file << std::setprecision(17);
            file << "schema_version: 1\n";
            file << "run_root: " << yamlQuote(run_root.string()) << "\n";
            file << "frame:\n";
            file << "  index: " << context.frame_index << "\n";
            file << "  stamp_sec: " << context.stamp.sec << "\n";
            file << "  stamp_nanosec: " << context.stamp.nanosec << "\n";
            file << "sample:\n";
            file << "  status: " << yamlQuote(sample.status) << "\n";
            file << "  valid_grid_minimum: " << boolString(sample.valid) << "\n";
            file << "armor:\n";
            file << "  index: " << sample.armor_index << "\n";
            file << "  name: " << sample.armor_name << "\n";
            file << "  type: " << yamlQuote(sample.armor_type) << "\n";
            file << "  confidence: ";
            writeYamlNumber(file, sample.confidence);
            file << "\n";
            file << "camera:\n";
            file << "  matrix: [";
            for (std::size_t i = 0; i < sample.camera_matrix.size(); ++i) {
                if (i != 0) {
                    file << ", ";
                }
                writeYamlNumber(file, sample.camera_matrix[i]);
            }
            file << "]\n";
            file << "  distortion_coefficients: [";
            for (std::size_t i = 0; i < sample.distortion_coefficients.size(); ++i) {
                if (i != 0) {
                    file << ", ";
                }
                writeYamlNumber(file, sample.distortion_coefficients[i]);
            }
            file << "]\n";
            file << "image_corners_px:\n";
            for (const auto &corner : sample.image_corners) {
                file << "  - [";
                writeYamlNumber(file, corner.x);
                file << ", ";
                writeYamlNumber(file, corner.y);
                file << "]\n";
            }
            file << "fixed_pnp_slice:\n";
            file << "  dir_yaw_rad: ";
            writeYamlNumber(file, sample.fixed_dir_yaw_rad);
            file << "\n";
            file << "  dir_pitch_rad: ";
            writeYamlNumber(file, sample.fixed_dir_pitch_rad);
            file << "\n";
            file << "  distance_m: ";
            writeYamlNumber(file, sample.pnp_distance_m);
            file << "\n";
            file << "  pose_yaw_rad: ";
            writeYamlNumber(file, sample.pnp_pose_yaw_rad);
            file << "\n";
            file << "grid:\n";
            file << "  physical_min_distance_m: ";
            writeYamlNumber(file, sample.physical_min_distance_m);
            file << "\n";
            file << "  physical_max_distance_m: ";
            writeYamlNumber(file, sample.physical_max_distance_m);
            file << "\n";
            file << "  requested_distance_min_m: ";
            writeYamlNumber(file, sample.requested_distance_min_m);
            file << "\n";
            file << "  requested_distance_max_m: ";
            writeYamlNumber(file, sample.requested_distance_max_m);
            file << "\n";
            file << "  actual_distance_min_m: ";
            writeYamlNumber(file, sample.actual_distance_min_m);
            file << "\n";
            file << "  actual_distance_max_m: ";
            writeYamlNumber(file, sample.actual_distance_max_m);
            file << "\n";
            file << "  distance_step_m: ";
            writeYamlNumber(file, sample.distance_step_m);
            file << "\n";
            file << "  requested_pose_yaw_min_rad: ";
            writeYamlNumber(file, sample.requested_pose_yaw_min_rad);
            file << "\n";
            file << "  requested_pose_yaw_max_rad: ";
            writeYamlNumber(file, sample.requested_pose_yaw_max_rad);
            file << "\n";
            file << "  actual_pose_yaw_min_rad: ";
            writeYamlNumber(file, sample.actual_pose_yaw_min_rad);
            file << "\n";
            file << "  actual_pose_yaw_max_rad: ";
            writeYamlNumber(file, sample.actual_pose_yaw_max_rad);
            file << "\n";
            file << "  pose_yaw_step_rad: ";
            writeYamlNumber(file, sample.pose_yaw_step_rad);
            file << "\n";
            file << "  distance_count: " << sample.distance_count << "\n";
            file << "  pose_yaw_count: " << sample.pose_yaw_count << "\n";
            file << "  point_count: " << sample.grid.size() << "\n";
            file << "  huber_loss_scale_px: ";
            writeYamlNumber(file, sample.huber_loss_scale_px);
            file << "\n";
            file << "timing_ms:\n";
            file << "  yaw_search: " << sample.yaw_search_elapsed_ms << "\n";
            file << "  ba_4dof_ypd: " << sample.ba_elapsed_ms << "\n";
            file << "  grid: " << sample.grid_elapsed_ms << "\n";
            file << "  total: " << sample.scan_elapsed_ms << "\n";
            file << "yaw_search:\n";
            file << "  success: " << boolString(sample.yaw_search_success) << "\n";
            file << "  status: " << yamlQuote(sample.yaw_search_status) << "\n";
            file << "ba_4dof_ypd:\n";
            file << "  success: " << boolString(sample.ba_success) << "\n";
            file << "  status: " << yamlQuote(sample.ba_status) << "\n";
            file << "  solver_summary_available: " << boolString(sample.ba_solver_summary_available) << "\n";
            file << "  initial_cost: ";
            writeYamlNumber(file, sample.ba_initial_cost);
            file << "\n";
            file << "  final_cost: ";
            writeYamlNumber(file, sample.ba_final_cost);
            file << "\n";
            file << "  num_iterations: " << sample.ba_num_iterations << "\n";
            file << "  termination_type: " << yamlQuote(sample.ba_termination_type) << "\n";
            file << "markers:\n";
            writeYamlMarker(file, "pnp", findMarker(sample, "pnp"));
            writeYamlMarker(file, "yaw_search", findMarker(sample, "yaw_search"));
            writeYamlMarker(file, "ba_4dof_ypd", findMarker(sample, "ba_4dof_ypd"));
            writeYamlMarker(file, "grid_min", findMarker(sample, "grid_min"));
            return file.good();
        }
    } // namespace

    DebugPoseLandscapeCsvWriter::DebugPoseLandscapeCsvWriter(const std::string &root_dir, const std::string &video) {
        const std::filesystem::path root =
            root_dir.empty() ? std::filesystem::current_path() : std::filesystem::path(root_dir);
        const std::filesystem::path base_root =
            root / "debug" / "pose_refine" / "pose_landscape" / sanitizePathComponent(video) / timestampString();
        run_root_ = base_root;
        for (std::size_t suffix = 1; std::filesystem::exists(run_root_); ++suffix) {
            run_root_ = base_root.string() + "_" + std::to_string(suffix);
        }

        std::error_code error;
        std::filesystem::create_directories(run_root_, error);
        if (error) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                         "无法创建 pose landscape 输出目录 %s: %s",
                         run_root_.string().c_str(),
                         error.message().c_str());
            return;
        }
        index_file_.open(run_root_ / "index.csv");
        if (!index_file_.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "无法创建 index.csv: %s", run_root_.string().c_str());
            return;
        }
        writeIndexHeader();
        RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Pose landscape 输出目录: %s", run_root_.string().c_str());
    }

    void DebugPoseLandscapeCsvWriter::writeIndexHeader() {
        index_file_ << "frame_index,stamp_sec,stamp_nanosec,armor_index,armor_name,armor_type,confidence,status,"
                    << "sample_dir,distance_count,pose_yaw_count,scan_elapsed_ms,grid_elapsed_ms,yaw_search_elapsed_ms,"
                    << "ba_elapsed_ms,ba_success,ba_status,ba_initial_cost,ba_final_cost,ba_num_iterations,"
                    << "ba_termination_type,"
                    << "pnp_distance_m,pnp_pose_yaw_rad,pnp_status,pnp_actual_cost,pnp_slice_cost,"
                    << "yaw_search_distance_m,yaw_search_pose_yaw_rad,yaw_search_status,yaw_search_actual_cost,"
                    << "yaw_search_slice_cost," << "ba_4dof_ypd_distance_m,ba_4dof_ypd_pose_yaw_rad,ba_4dof_ypd_status,"
                    << "ba_4dof_ypd_actual_cost,ba_4dof_ypd_slice_cost,"
                    << "grid_min_distance_m,grid_min_pose_yaw_rad,grid_min_status,grid_min_actual_cost,"
                    << "grid_min_slice_cost\n";
    }

    bool DebugPoseLandscapeCsvWriter::writeSample(const DebugFrameContext &context, const PoseLandscapeSample &sample) {
        const std::filesystem::path sample_dir = run_root_ /
            ("frame_" + std::to_string(context.frame_index) + "_armor_" + std::to_string(sample.armor_index));
        std::error_code error;
        std::filesystem::create_directories(sample_dir, error);
        if (error) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                         "无法创建 pose landscape 样本目录 %s: %s",
                         sample_dir.string().c_str(),
                         error.message().c_str());
            return false;
        }

        const bool grid_written = writeGridCsv(sample_dir / "grid.csv", sample);
        const bool markers_written = writeMarkersCsv(sample_dir / "markers.csv", sample);
        const bool metadata_written = writeMetadataYaml(sample_dir / "metadata.yaml", context, sample, run_root_);
        if (!grid_written || !markers_written || !metadata_written) {
            RCLCPP_ERROR(rclcpp::get_logger(kLoggerName),
                         "写入 pose landscape 样本失败: %s (grid=%s, markers=%s, metadata=%s)",
                         sample_dir.string().c_str(),
                         boolString(grid_written),
                         boolString(markers_written),
                         boolString(metadata_written));
            return false;
        }

        if (!index_file_.is_open()) {
            return false;
        }
        index_file_ << std::setprecision(17) << context.frame_index << "," << context.stamp.sec << ","
                    << context.stamp.nanosec << "," << sample.armor_index << "," << sample.armor_name << ","
                    << csvEscape(sample.armor_type) << ",";
        writeNumber(index_file_, sample.confidence);
        index_file_ << "," << csvEscape(sample.status) << "," << csvEscape(sample_dir.string()) << ","
                    << sample.distance_count << "," << sample.pose_yaw_count << "," << sample.scan_elapsed_ms << ","
                    << sample.grid_elapsed_ms << "," << sample.yaw_search_elapsed_ms << "," << sample.ba_elapsed_ms
                    << "," << boolString(sample.ba_success) << "," << csvEscape(sample.ba_status) << ",";
        writeNumber(index_file_, sample.ba_initial_cost);
        index_file_ << ",";
        writeNumber(index_file_, sample.ba_final_cost);
        index_file_ << "," << sample.ba_num_iterations << "," << csvEscape(sample.ba_termination_type) << ",";
        writeIndexMarker(index_file_, findMarker(sample, "pnp"));
        writeIndexMarker(index_file_, findMarker(sample, "yaw_search"));
        writeIndexMarker(index_file_, findMarker(sample, "ba_4dof_ypd"));
        writeIndexMarker(index_file_, findMarker(sample, "grid_min"));
        index_file_ << "\n";
        index_file_.flush();
        return index_file_.good();
    }

    void DebugPoseLandscapeCsvWriter::onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) {
        if (!index_file_.is_open()) {
            return;
        }
        for (const auto &sample : data.landscape_samples) {
            (void)writeSample(context, sample);
        }
    }

} // namespace armor_detector::debug
