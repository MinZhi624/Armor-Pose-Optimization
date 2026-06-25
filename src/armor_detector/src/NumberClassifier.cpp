#include "armor_detector/NumberClassifier.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace armor_detector {

    NumberClassifier::NumberClassifier(const Params &params) : params_(params) {
        if (params_.model_path.empty()) {
            return;
        }
        try {
            net_ = cv::dnn::readNetFromONNX(params_.model_path);
            model_loaded_ = !net_.empty();
        }
        catch (const cv::Exception &) {
            model_loaded_ = false;
        }
    }

    cv::Mat NumberClassifier::softmax(const cv::Mat &logits) {
        cv::Mat exps;
        cv::exp(logits, exps);
        float sum = static_cast<float>(cv::sum(exps)[0]);
        return exps / sum;
    }

    float NumberClassifier::sigmoid(float x) {
        return 1.0f / (1.0f + std::exp(-x));
    }

    ArmorName NumberClassifier::intToArmorName(int id) {
        switch (id) {
            case 0:
                return ArmorName::NONE;
            case 1:
                return ArmorName::ONE;
            case 2:
                return ArmorName::TWO;
            case 3:
                return ArmorName::THREE;
            case 4:
                return ArmorName::FOUR;
            case 5:
                return ArmorName::FIVE;
            default:
                return ArmorName::NONE;
        }
    }

    cv::Mat NumberClassifier::getNumberROI(const cv::Mat &img_bgr, const ArmorGeometry &geometry) const {
        const auto &left = geometry.paired_lights[0];
        const auto &right = geometry.paired_lights[1];

        // Source points from the armor corners (left side top/bottom, right side bottom/top)
        std::vector<cv::Point2f> src_pts = {left.top, left.bottom, right.bottom, right.top};

        // Destination: 28x28 ROI with 7px vertical padding
        std::vector<cv::Point2f> dst_pts = {{0.f, 7.f}, {0.f, 21.f}, {28.f, 21.f}, {28.f, 7.f}};

        cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
        cv::Mat roi;
        cv::warpPerspective(img_bgr, roi, M, cv::Size(28, 28));
        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);

        return roi;
    }

    cv::Mat NumberClassifier::getArmorPattern(const cv::Mat &img_bgr, const ArmorGeometry &geometry) const {
        // Armor corners: left_top, right_top, right_bottom, left_bottom
        cv::Point2f armor_top_left = geometry.corners[0];
        cv::Point2f armor_top_right = geometry.corners[1];
        cv::Point2f armor_bottom_right = geometry.corners[2];
        cv::Point2f armor_bottom_left = geometry.corners[3];

        // Bounding box of the armor pattern
        float min_x = std::min({armor_top_left.x, armor_top_right.x, armor_bottom_right.x, armor_bottom_left.x});
        float max_x = std::max({armor_top_left.x, armor_top_right.x, armor_bottom_right.x, armor_bottom_left.x});
        float min_y = std::min({armor_top_left.y, armor_top_right.y, armor_bottom_right.y, armor_bottom_left.y});
        float max_y = std::max({armor_top_left.y, armor_top_right.y, armor_bottom_right.y, armor_bottom_left.y});

        // Check bounds
        if (min_x < 0 || min_y < 0 || max_x >= img_bgr.cols || max_y >= img_bgr.rows) {
            return cv::Mat();
        }

        // Extract ROI using bounding box and clone to avoid lifetime issues
        cv::Rect roi_rect(static_cast<int>(min_x),
                          static_cast<int>(min_y),
                          static_cast<int>(max_x - min_x),
                          static_cast<int>(max_y - min_y));

        if (roi_rect.width <= 0 || roi_rect.height <= 0) {
            return cv::Mat();
        }

        return img_bgr(roi_rect).clone();
    }

    ArmorClassification NumberClassifier::classifyOne(const ArmorGeometry &geometry, const cv::Mat &img_bgr) {
        ArmorClassification result;
        result.name = ArmorName::NONE;
        result.confidence = 0.0f;

        cv::Mat roi = getNumberROI(img_bgr, geometry);
        if (roi.empty()) {
            return result;
        }
        result.number_roi = roi.clone();

        // Prepare input blob: 1x1x28x28, normalized to [0,1]
        cv::Mat blob;
        cv::dnn::blobFromImage(roi, blob, 1.0 / 255.0, cv::Size(28, 28), 0, false, false);
        net_.setInput(blob);

        // Forward pass
        cv::Mat outputs = net_.forward();

        // Model output shape: 1x11
        // outputs[0] = objectness logit
        // outputs[1..10] = 10 class logits
        // Reshape to 1x11
        outputs = outputs.reshape(1, {1, 11});

        float obj_logit = outputs.at<float>(0, 0);

        // Class logits: indices 1-10
        cv::Mat class_logits(1, 10, CV_32F);
        for (int i = 0; i < 10; ++i) {
            class_logits.at<float>(0, i) = outputs.at<float>(0, i + 1);
        }
        cv::Mat probs = softmax(class_logits);

        // Merge 10 classes into 6: NONE(0+6+7+8+9), ONE(1), TWO(2), THREE(3), FOUR(4), FIVE(5)
        float merged[6] = {probs.at<float>(0, 0) + probs.at<float>(0, 6) + probs.at<float>(0, 7) +
                               probs.at<float>(0, 8) + probs.at<float>(0, 9),
                           probs.at<float>(0, 1),
                           probs.at<float>(0, 2),
                           probs.at<float>(0, 3),
                           probs.at<float>(0, 4),
                           probs.at<float>(0, 5)};

        // Find best label
        int best_id = 0;
        for (int i = 1; i < 6; ++i) {
            if (merged[i] > merged[best_id]) {
                best_id = i;
            }
        }

        result.confidence = sigmoid(obj_logit) * merged[best_id];
        result.name = intToArmorName(best_id);

        return result;
    }

    bool NumberClassifier::checkArmorName(const ArmorClassification &classification) const {
        return classification.name != ArmorName::NONE && classification.confidence > params_.confidence_threshold;
    }

    bool NumberClassifier::checkArmorType(const ArmorGeometry &geometry,
                                          const ArmorClassification &classification) const {
        if (geometry.type == ArmorType::LARGE) {
            return classification.name == ArmorName::ONE;
        }
        else {
            return classification.name != ArmorName::ONE;
        }
    }

    std::vector<ClassifiedArmor> NumberClassifier::deduplicate(std::vector<ClassifiedArmor> armors) {
        std::vector<ClassifiedArmor> result;
        std::vector<bool> removed(armors.size(), false);

        for (std::size_t i = 0; i < armors.size(); ++i) {
            if (removed[i]) {
                continue;
            }

            for (std::size_t j = i + 1; j < armors.size(); ++j) {
                if (removed[j]) {
                    continue;
                }

                const auto &geo_i = armors[i].geometry;
                const auto &geo_j = armors[j].geometry;

                // Check if they share any light bar
                bool same_left = (geo_i.paired_lights[0].id == geo_j.paired_lights[0].id);
                bool same_right = (geo_i.paired_lights[1].id == geo_j.paired_lights[1].id);
                bool share_any = same_left || same_right;

                if (!share_any) {
                    continue;
                }

                // Shared same side: keep smaller ROI area
                if (same_left || same_right) {
                    bool roi_i_empty = armors[i].classification.number_roi.empty();
                    bool roi_j_empty = armors[j].classification.number_roi.empty();

                    double area_i = roi_i_empty ? 1e9 : armors[i].classification.number_roi.total();
                    double area_j = roi_j_empty ? 1e9 : armors[j].classification.number_roi.total();

                    if (area_i <= area_j) {
                        removed[j] = true;
                    }
                    else {
                        removed[i] = true;
                        break;
                    }
                }
            }

            if (!removed[i]) {
                result.push_back(armors[i]);
            }
        }

        return result;
    }

    std::vector<ClassifiedArmor> NumberClassifier::classify(const std::vector<ArmorCandidate> &candidates,
                                                            const cv::Mat &img_bgr) {
        classification_debug_.classified_armors.clear();
        classification_debug_.number_rois.clear();

        if (!model_loaded_) {
            return {};
        }

        std::vector<ClassifiedArmor> all_results;

        for (const auto &candidate : candidates) {
            const auto &geometry = candidate.geometry;

            // Classify
            ArmorClassification classification = classifyOne(geometry, img_bgr);

            // Get pattern for debug display
            cv::Mat pattern = getArmorPattern(img_bgr, geometry);

            ClassifiedArmor classified;
            classified.geometry = geometry;
            classified.classification = classification;
            classified.classification.pattern = pattern;

            // Filter: check name and type
            if (!checkArmorName(classification)) {
                continue;
            }

            if (!checkArmorType(geometry, classification)) {
                continue;
            }

            all_results.push_back(classified);

            // Store ROI clone for debug display
            if (!classification.number_roi.empty()) {
                classification_debug_.number_rois.push_back(classification.number_roi.clone());
            }
        }

        // Deduplicate
        auto deduplicated = deduplicate(all_results);

        classification_debug_.classified_armors = deduplicated;
        return deduplicated;
    }

} // namespace armor_detector
