#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

namespace armor_detector {

    class NumberClassifier {
    public:
        struct Params {
            std::string model_path;
            float confidence_threshold = 0.5f;
        };

        NumberClassifier() = default;
        explicit NumberClassifier(const Params &params);

        std::vector<ClassifiedArmor> classify(const std::vector<ArmorCandidate> &candidates, const cv::Mat &img_bgr);

        const debug::ClassificationDebugData &getClassificationDebugData() const {
            return classification_debug_;
        }

    private:
        ArmorClassification classifyOne(const ArmorGeometry &geometry, const cv::Mat &img_bgr);
        cv::Mat getNumberROI(const cv::Mat &img_bgr, const ArmorGeometry &geometry) const;
        cv::Mat getArmorPattern(const cv::Mat &img_bgr, const ArmorGeometry &geometry) const;
        bool checkArmorName(const ArmorClassification &classification) const;
        bool checkArmorType(const ArmorGeometry &geometry, const ArmorClassification &classification) const;
        std::vector<ClassifiedArmor> deduplicate(std::vector<ClassifiedArmor> armors);

        static cv::Mat softmax(const cv::Mat &logits);
        static float sigmoid(float x);
        static ArmorName intToArmorName(int id);

        Params params_;
        cv::dnn::Net net_;
        bool model_loaded_ = false;
        debug::ClassificationDebugData classification_debug_;
    };

} // namespace armor_detector
