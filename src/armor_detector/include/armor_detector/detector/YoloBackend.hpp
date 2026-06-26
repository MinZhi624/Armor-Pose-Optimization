#pragma once

#include "armor_detector/detector/IArmorDetectorBackend.hpp"

#include <opencv2/core.hpp>
#include <openvino/openvino.hpp>

#include <array>

namespace armor_detector {

    struct YoloParams {
        std::string model_path = "model/robot_0526.onnx";
        std::string device = "CPU";
        int input_size = 640;
        float score_threshold = 0.65f;
        float nms_threshold = 0.45f;
        float min_confidence = 0.65f;
        int max_raw_candidates = 100;
        bool use_roi = false;
        cv::Rect roi;
    };

    class YoloArmorDetectorBackend : public IArmorDetectorBackend {
    public:
        explicit YoloArmorDetectorBackend(const YoloParams &params, LightBarColor target_color);
        ~YoloArmorDetectorBackend() override;

        DetectionResult detect(const cv::Mat &img_bgr) override;
        DetectionBackend backend() const override {
            return DetectionBackend::YOLO;
        }

    private:
        static constexpr int kNumClasses = 9;
        static constexpr int kNumColors = 4;
        static constexpr int kOutputCols = 22;

        struct RawDetection {
            std::array<cv::Point2f, 4> keypoints;
            float score = 0.0f;
            float nms_score = 0.0f;
            int class_id = -1;
            int color_id = -1;
            ArmorName name = ArmorName::NONE;
            ArmorType type = ArmorType::NONE;
            LightBarColor color = LightBarColor::NONE;
        };

        cv::Mat letterbox(const cv::Mat &img, float &scale);
        std::vector<RawDetection> parse(const float *data, int num_proposals, float scale);
        std::vector<ClassifiedArmor> convert(const std::vector<RawDetection> &detections);

        ov::CompiledModel compiled_model_;
        ov::InferRequest infer_request_;
        YoloParams params_;
        LightBarColor target_color_;
    };

} // namespace armor_detector
