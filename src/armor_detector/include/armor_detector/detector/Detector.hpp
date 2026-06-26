#pragma once

#include "armor_detector/detector/DetectionBackend.hpp"
#include "armor_detector/detector/DetectionResult.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace armor_detector {

    class IArmorDetectorBackend; // forward declaration —内部类型，不暴露给外部

    class Detector {
    public:
        struct Params {
            // 选择
            DetectionBackend backend_type = DetectionBackend::TRADITIONAL;
            LightBarColor target_color = LightBarColor::BLUE;

            // Traditional 后端参数
            struct {
                struct {
                    int gray_threshold = 100;
                    int color_threshold = 100;
                } preprocess;
                struct {
                    int min_contours_area = 30;
                    float min_contours_ratio = 0.06f;
                    float max_contours_ratio = 0.5f;
                } light;
                struct {
                    double max_angle_diff = 15.0;
                    double min_length_ratio = 0.7;
                    double min_x_diff_ratio = 0.75;
                    double max_y_diff_ratio = 1.0;
                    double max_distance_ratio = 0.8;
                    double min_distance_ratio = 0.1;
                } armor;
                struct {
                    std::string model_path;
                    float threshold = 0.5f;
                } number;
            } traditional;

            // YOLO 后端参数
            struct {
                std::string model_path = "model/robot_0526.onnx";
                std::string device = "CPU";
                int input_size = 640;
                float score_threshold = 0.65f;
                float nms_threshold = 0.45f;
                float min_confidence = 0.65f;
                int max_raw_candidates = 100;
                bool use_roi = false;
                int roi_x = 0;
                int roi_y = 0;
                int roi_width = -1;
                int roi_height = -1;
            } yolo;
        };

        explicit Detector(const Params &params);
        ~Detector();

        DetectionResult detect(const cv::Mat &img_bgr);

        DetectionBackend backend() const;

    private:
        std::unique_ptr<IArmorDetectorBackend> backend_;
        DetectionBackend backend_type_;
    };

} // namespace armor_detector
