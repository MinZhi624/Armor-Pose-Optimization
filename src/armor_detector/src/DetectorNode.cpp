#include "armor_detector/CameraProvider.hpp"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui.hpp>

namespace armor_detector
{

class DetectorNode : public rclcpp::Node
{
private:
    CameraProvider camera_provider_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    int frame_count_ = 0;

    void run(const sensor_msgs::msg::Image::SharedPtr & msg);

public:
    DetectorNode() : Node("armor_detector_node_cpp")
    {
        camera_provider_.init();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); }
        );

        RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
    }
};

void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);
    RCLCPP_INFO(this->get_logger(), "图像时间为:%lf", frame.timestamp.sec + frame.timestamp.nanosec / 1e9);
    cv::imshow("image_raw", frame.image);
    int key = cv::waitKey(1);
    if (key == 27) {  // ESC 退出
        cv::destroyAllWindows();
        rclcpp::shutdown();
    }
}

} // namespace armor_detector

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
