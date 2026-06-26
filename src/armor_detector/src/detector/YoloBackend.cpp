#include "armor_detector/detector/YoloBackend.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace armor_detector {

    namespace {
        struct ClassInfo {
            ArmorName name;
            ArmorType type;
            const char *label;
        };

        // RobotDetectionModel classes: G, 1, 2, 3, 4, 5, O, Bs, Bb.
        static constexpr ClassInfo kClassMapping[9] = {
            {ArmorName::NONE, ArmorType::NONE, "sentry"},
            {ArmorName::ONE, ArmorType::SMALL, "one"},
            {ArmorName::TWO, ArmorType::SMALL, "two"},
            {ArmorName::THREE, ArmorType::SMALL, "three"},
            {ArmorName::FOUR, ArmorType::SMALL, "four"},
            {ArmorName::FIVE, ArmorType::SMALL, "five"},
            {ArmorName::NONE, ArmorType::NONE, "outpost"},
            {ArmorName::NONE, ArmorType::NONE, "base_small"},
            {ArmorName::NONE, ArmorType::LARGE, "base_big"},
        };

        // RobotDetectionModel reference filtering implies: 0=blue, 1=red, 2=gray, 3=purple.
        static constexpr LightBarColor kColorMapping[4] = {
            LightBarColor::BLUE,
            LightBarColor::RED,
            LightBarColor::NONE,
            LightBarColor::NONE,
        };

        float sigmoid(float x) {
            if (x >= 0.0f) {
                return 1.0f / (1.0f + std::exp(-x));
            }
            float e = std::exp(x);
            return e / (1.0f + e);
        }

        ArmorType inferArmorType(const std::array<cv::Point2f, 4> &corners) {
            const float top_width = cv::norm(corners[1] - corners[0]);
            const float bottom_width = cv::norm(corners[2] - corners[3]);
            const float left_height = cv::norm(corners[3] - corners[0]);
            const float right_height = cv::norm(corners[2] - corners[1]);
            const float width = (top_width + bottom_width) * 0.5f;
            const float height = (left_height + right_height) * 0.5f;
            if (height <= 1e-3f) {
                return ArmorType::NONE;
            }
            return (width / height > 3.2f) ? ArmorType::LARGE : ArmorType::SMALL;
        }

        std::string shapeToString(const ov::Shape &shape) {
            std::ostringstream oss;
            oss << "[";
            for (std::size_t i = 0; i < shape.size(); ++i) {
                if (i > 0) {
                    oss << ", ";
                }
                oss << shape[i];
            }
            oss << "]";
            return oss.str();
        }
    } // namespace

    YoloArmorDetectorBackend::YoloArmorDetectorBackend(const YoloParams &params, LightBarColor target_color) :
        params_(params), target_color_(target_color) {
        ov::Core core;
        auto model = core.read_model(params_.model_path);

        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input()
            .preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({255.0, 255.0, 255.0});
        ppp.input().model().set_layout("NCHW");
        ppp.output().tensor().set_element_type(ov::element::f32);
        model = ppp.build();

        compiled_model_ = core.compile_model(model, params_.device);
        infer_request_ = compiled_model_.create_infer_request();
        RCLCPP_INFO(rclcpp::get_logger("YoloBackend"),
                    "RobotDetectionModel loaded: %s on %s",
                    params_.model_path.c_str(),
                    params_.device.c_str());
    }

    YoloArmorDetectorBackend::~YoloArmorDetectorBackend() = default;

    cv::Mat YoloArmorDetectorBackend::letterbox(const cv::Mat &img, float &scale) {
        int w = img.cols;
        int h = img.rows;
        int target = params_.input_size;
        float scale_x = static_cast<float>(target) / w;
        float scale_y = static_cast<float>(target) / h;
        scale = std::min(scale_x, scale_y);
        int new_w = static_cast<int>(w * scale);
        int new_h = static_cast<int>(h * scale);
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(new_w, new_h));
        cv::Mat padded = cv::Mat::zeros(target, target, CV_8UC3);
        resized.copyTo(padded(cv::Rect(0, 0, new_w, new_h)));
        return padded;
    }

    std::vector<YoloArmorDetectorBackend::RawDetection>
    YoloArmorDetectorBackend::parse(const float *data, int num_proposals, float scale) {
        std::vector<RawDetection> detections;
        for (int i = 0; i < num_proposals; ++i) {
            const float *row = data + i * kOutputCols;
            float confidence = sigmoid(row[8]);
            if (confidence < params_.score_threshold) {
                continue;
            }

            int best_color = -1;
            float best_color_score = -std::numeric_limits<float>::infinity();
            for (int c = 0; c < kNumColors; ++c) {
                float score = row[9 + c];
                if (score > best_color_score) {
                    best_color_score = score;
                    best_color = c;
                }
            }

            int best_class = -1;
            float best_class_score = -std::numeric_limits<float>::infinity();
            for (int c = 0; c < kNumClasses; ++c) {
                float score = row[13 + c];
                if (score > best_class_score) {
                    best_class_score = score;
                    best_class = c;
                }
            }

            if (best_color < 0 || best_class < 0) {
                continue;
            }

            RawDetection det;
            det.score = confidence;
            det.nms_score = best_class_score;
            det.class_id = best_class;
            det.color_id = best_color;
            det.name = kClassMapping[best_class].name;
            det.type = kClassMapping[best_class].type;
            det.color = kColorMapping[best_color];

            // RobotDetectionModel keypoints are TL, BL, BR, TR; project to TL, TR, BR, BL.
            det.keypoints = {{
                cv::Point2f(row[0] / scale, row[1] / scale),
                cv::Point2f(row[6] / scale, row[7] / scale),
                cv::Point2f(row[4] / scale, row[5] / scale),
                cv::Point2f(row[2] / scale, row[3] / scale),
            }};

            if (det.name != ArmorName::NONE) {
                det.type = inferArmorType(det.keypoints);
            }

            detections.push_back(det);
        }
        return detections;
    }

    std::vector<ClassifiedArmor> YoloArmorDetectorBackend::convert(const std::vector<RawDetection> &detections) {
        std::vector<ClassifiedArmor> result;
        for (const auto &det : detections) {
            ClassifiedArmor armor;
            armor.geometry.corners = {det.keypoints[0], det.keypoints[1], det.keypoints[2], det.keypoints[3]};
            armor.geometry.type = det.type;
            armor.classification.name = det.name;
            armor.classification.confidence = det.score;
            result.push_back(armor);
        }
        return result;
    }

    DetectionResult YoloArmorDetectorBackend::detect(const cv::Mat &img_bgr) {
        DetectionResult result;
        result.debug.backend = DetectionBackend::YOLO;
        using Clock = std::chrono::steady_clock;
        using ms = std::chrono::duration<double, std::milli>;

        cv::Mat input = img_bgr;
        cv::Point2f roi_offset(0.0f, 0.0f);
        if (params_.use_roi && params_.roi.width > 0 && params_.roi.height > 0) {
            cv::Rect roi = params_.roi & cv::Rect(0, 0, img_bgr.cols, img_bgr.rows);
            if (roi.width > 0 && roi.height > 0) {
                input = img_bgr(roi).clone();
                roi_offset = cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y));
            }
        }
        result.debug.yolo.stage1_source_roi = input;

        auto t0 = Clock::now();
        float scale;
        cv::Mat letterboxed = letterbox(input, scale);
        result.debug.yolo.stage1_letterbox = letterboxed;
        auto t1 = Clock::now();

        ov::Tensor input_tensor(
            ov::element::u8,
            ov::Shape{1, static_cast<size_t>(params_.input_size), static_cast<size_t>(params_.input_size), 3},
            letterboxed.data);
        infer_request_.set_input_tensor(input_tensor);
        infer_request_.infer();
        auto output = infer_request_.get_output_tensor(0);
        auto t2 = Clock::now();

        auto output_shape = output.get_shape();
        const float *output_data = output.data<float>();
        cv::Mat output_mat;
        if (output_shape.size() == 3 && output_shape[2] == kOutputCols) {
            output_mat =
                cv::Mat(static_cast<int>(output_shape[1]), kOutputCols, CV_32F, const_cast<float *>(output_data));
        }
        else if (output_shape.size() == 3 && output_shape[1] == kOutputCols) {
            cv::Mat channel_first(
                kOutputCols, static_cast<int>(output_shape[2]), CV_32F, const_cast<float *>(output_data));
            cv::transpose(channel_first, output_mat);
        }
        else if (output_shape.size() == 2 && output_shape[1] == kOutputCols) {
            output_mat =
                cv::Mat(static_cast<int>(output_shape[0]), kOutputCols, CV_32F, const_cast<float *>(output_data));
        }
        else {
            throw std::runtime_error("Unsupported RobotDetectionModel output shape: " + shapeToString(output_shape));
        }

        auto raw = parse(output_mat.ptr<float>(0), output_mat.rows, scale);
        if (roi_offset.x != 0.0f || roi_offset.y != 0.0f) {
            for (auto &d : raw) {
                for (auto &point : d.keypoints) {
                    point += roi_offset;
                }
            }
        }
        std::sort(raw.begin(), raw.end(), [](const RawDetection &lhs, const RawDetection &rhs) {
            return lhs.score > rhs.score;
        });
        if (static_cast<int>(raw.size()) > params_.max_raw_candidates) {
            raw.resize(params_.max_raw_candidates);
        }

        for (const auto &d : raw) {
            ClassifiedArmor a;
            a.geometry.corners = {d.keypoints[0], d.keypoints[1], d.keypoints[2], d.keypoints[3]};
            a.geometry.type = d.type;
            a.classification.name = d.name;
            a.classification.confidence = d.score;
            result.debug.yolo.stage2_score_filtered.push_back(a);
        }

        std::vector<RawDetection> class_color_filtered;
        for (const auto &d : raw) {
            if (d.name == ArmorName::NONE || d.type == ArmorType::NONE) {
                debug::RejectedArmor rej;
                rej.geometry.corners = {d.keypoints[0], d.keypoints[1], d.keypoints[2], d.keypoints[3]};
                rej.reason = debug::DebugRejectReason::TYPE_MISMATCH;
                rej.detail = "unsupported_class=" + std::to_string(d.class_id);
                result.debug.yolo.stage3_filter_rejected.push_back(rej);
                continue;
            }
            if (d.color != target_color_) {
                debug::RejectedArmor rej;
                rej.geometry.corners = {d.keypoints[0], d.keypoints[1], d.keypoints[2], d.keypoints[3]};
                rej.reason = debug::DebugRejectReason::BAD_COLOR;
                rej.detail = "color_id=" + std::to_string(d.color_id);
                result.debug.yolo.stage3_filter_rejected.push_back(rej);
                continue;
            }
            class_color_filtered.push_back(d);
        }

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        for (const auto &d : class_color_filtered) {
            float x_min = std::min({d.keypoints[0].x, d.keypoints[1].x, d.keypoints[2].x, d.keypoints[3].x});
            float y_min = std::min({d.keypoints[0].y, d.keypoints[1].y, d.keypoints[2].y, d.keypoints[3].y});
            float x_max = std::max({d.keypoints[0].x, d.keypoints[1].x, d.keypoints[2].x, d.keypoints[3].x});
            float y_max = std::max({d.keypoints[0].y, d.keypoints[1].y, d.keypoints[2].y, d.keypoints[3].y});
            int width = std::max(1, static_cast<int>(x_max - x_min));
            int height = std::max(1, static_cast<int>(y_max - y_min));
            boxes.emplace_back(static_cast<int>(x_min), static_cast<int>(y_min), width, height);
            scores.push_back(d.score);
        }
        std::vector<int> nms_indices;
        cv::dnn::NMSBoxes(boxes, scores, params_.score_threshold, params_.nms_threshold, nms_indices);

        std::vector<bool> kept(class_color_filtered.size(), false);
        for (int idx : nms_indices) {
            if (idx >= 0 && static_cast<std::size_t>(idx) < kept.size()) {
                kept[static_cast<std::size_t>(idx)] = true;
            }
        }

        std::vector<RawDetection> nms_result;
        for (std::size_t i = 0; i < class_color_filtered.size(); ++i) {
            if (kept[i]) {
                nms_result.push_back(class_color_filtered[i]);
            }
            else {
                debug::RejectedArmor rej;
                rej.geometry.corners = {class_color_filtered[i].keypoints[0],
                                        class_color_filtered[i].keypoints[1],
                                        class_color_filtered[i].keypoints[2],
                                        class_color_filtered[i].keypoints[3]};
                rej.reason = debug::DebugRejectReason::UNKNOWN;
                rej.detail = "nms_suppressed";
                result.debug.yolo.stage3_nms_rejected.push_back(rej);
            }
        }
        result.debug.yolo.stage3_nms_kept = convert(nms_result);
        auto t3 = Clock::now();

        auto armors = convert(nms_result);
        std::vector<ClassifiedArmor> filtered;
        for (auto &a : armors) {
            if (a.classification.confidence >= params_.min_confidence) {
                filtered.push_back(a);
            }
            else {
                debug::RejectedArmor rej;
                rej.geometry = a.geometry;
                rej.reason = debug::DebugRejectReason::LOW_CONFIDENCE;
                rej.detail = "conf=" + std::to_string(a.classification.confidence);
                result.debug.yolo.stage3_filter_rejected.push_back(rej);
            }
        }
        auto t4 = Clock::now();

        result.armors = filtered;
        result.debug.final_armors = filtered;
        result.debug.yolo.stage4_final = filtered;

        result.debug.timings.push_back({"letterbox", std::chrono::duration_cast<ms>(t1 - t0).count()});
        result.debug.timings.push_back({"infer", std::chrono::duration_cast<ms>(t2 - t1).count()});
        result.debug.timings.push_back({"decode", std::chrono::duration_cast<ms>(t3 - t2).count()});
        result.debug.timings.push_back({"filter", std::chrono::duration_cast<ms>(t4 - t3).count()});
        result.debug.timings.push_back({"detect_total", std::chrono::duration_cast<ms>(t4 - t0).count()});

        return result;
    }

} // namespace armor_detector
