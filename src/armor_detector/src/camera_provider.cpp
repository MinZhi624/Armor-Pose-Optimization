#include "armor_detector/camera_provider.hpp"

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace armor_detector
{

void CameraProvider::init()
{
    // 从 config/camera_info.yaml 加载相机参数
    std::string package_share = ament_index_cpp::get_package_share_directory("armor_detector");
    std::string yaml_path = std::filesystem::path(package_share) / "config" / "camera_info.yaml";

    YAML::Node config = YAML::LoadFile(yaml_path);

    // 读取 camera_matrix (3x3)
    auto cm_data = config["camera_matrix"]["data"].as<std::vector<double>>();
    camera_info_.camera_matrix = cv::Matx33d(
        cm_data[0], cm_data[1], cm_data[2],
        cm_data[3], cm_data[4], cm_data[5],
        cm_data[6], cm_data[7], cm_data[8]
    );

    // 读取 distortion_coefficients (1x5)
    auto dc_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
    camera_info_.distortion_coefficients = cv::Vec<double, 5>(
        dc_data[0], dc_data[1], dc_data[2], dc_data[3], dc_data[4]
    );

    // 读取 rectification_matrix (3x3)
    auto rm_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
    camera_info_.rectification_matrix = cv::Matx33d(
        rm_data[0], rm_data[1], rm_data[2],
        rm_data[3], rm_data[4], rm_data[5],
        rm_data[6], rm_data[7], rm_data[8]
    );

    // 读取 projection_matrix (3x4)
    auto pm_data = config["projection_matrix"]["data"].as<std::vector<double>>();
    camera_info_.projection_matrix = cv::Matx34d(
        pm_data[0],  pm_data[1],  pm_data[2],  pm_data[3],
        pm_data[4],  pm_data[5],  pm_data[6],  pm_data[7],
        pm_data[8],  pm_data[9],  pm_data[10], pm_data[11]
    );
}

Frame CameraProvider::receiveImage(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    Frame frame;
    frame.image = cv_ptr->image;
    frame.timestamp = msg->header.stamp;
    return frame;
}

CameraInfo CameraProvider::getCameraInfo()
{
    return camera_info_;
}

} // namespace armor_detector
