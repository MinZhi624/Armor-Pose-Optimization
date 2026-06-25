# Armor Match & Number Classification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement armor plate matching (pairing light bars into armor candidates) and CNN-based number classification, with debug visualization for both stages.

**Architecture:** ArmorDetector takes sorted light bars and pairs them via geometric heuristics (angle, length, distance, color). NumberClassifier crops number ROIs, runs an ONNX CNN via OpenCV DNN, and deduplicates overlapping classifications. Both integrate into the existing DebugObserver pipeline with dedicated visualization views.

**Tech Stack:** C++17, OpenCV (core, imgproc, dnn), ROS 2, Eigen3, YAML-CPP, ONNX model

---

## File Structure

| File | Responsibility |
|------|---------------|
| `include/armor_detector/detector/ArmorDetector.hpp` | ArmorDetector class declaration |
| `src/detector/ArmorDetector.cpp` | ArmorDetector implementation (match, buildGeometry, checks) |
| `include/armor_detector/NumberClassifier.hpp` | NumberClassifier class declaration |
| `src/NumberClassifier.cpp` | NumberClassifier implementation (classify, CNN inference, dedup) |
| `include/armor_detector/debug/DebugArmorMatch.hpp` | DebugArmorMatchView declaration (update: add layer_state) |
| `src/debug/DebugArmorMatchView.cpp` | DebugArmorMatchView implementation |
| `include/armor_detector/debug/DebugClassification.hpp` | DebugClassificationView declaration (update: add layer_state) |
| `src/debug/DebugClassificationView.cpp` | DebugClassificationView implementation |
| `include/armor_detector/debug/DebugData.hpp` | Add new reject reasons to DebugRejectReason |
| `include/armor_detector/DetectorNode.hpp` | Add armor_detector_ and number_classifier_ members |
| `src/DetectorNode.cpp` | Integrate match+classify into run(), add params, register debug views |
| `CMakeLists.txt` | Add NumberClassifier.cpp, DebugArmorMatchView.cpp, DebugClassificationView.cpp, install model/ |
| `config/config.yaml` | Add armor match and number classifier parameters |
| `model/number_cnn.onnx` | ONNX model file (user-provided placeholder directory) |

---

## Task 1: Update DebugRejectReason Enum

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugData.hpp:84-93`

- [ ] **Step 1: Add new reject reasons to the enum**

Open `src/armor_detector/include/armor_detector/debug/DebugData.hpp` and replace the `DebugRejectReason` enum:

```cpp
enum class DebugRejectReason {
    UNKNOWN,
    TOO_SMALL,
    TOO_LARGE,
    BAD_RATIO,
    BAD_ANGLE,
    BAD_COLOR,
    LOW_CONFIDENCE,
    PNP_FAILED,
    BAD_LENGTH_RATIO,
    BAD_X_DIFF,
    BAD_Y_DIFF,
    BAD_DISTANCE,
    DUPLICATE,
    TYPE_MISMATCH,
};
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugData.hpp
git commit -m "feat: add armor match and classification reject reasons"
```

---

## Task 2: Implement ArmorDetector Header

**Files:**
- Modify: `src/armor_detector/include/armor_detector/detector/ArmorDetector.hpp`

- [ ] **Step 1: Write the ArmorDetector header**

Replace the contents of `src/armor_detector/include/armor_detector/detector/ArmorDetector.hpp` with:

```cpp
#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/tools/angle.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace armor_detector
{

class ArmorDetector
{
public:
    struct Params {
        double max_angle_diff = 15.0;
        double min_length_ratio = 0.7;
        double min_x_diff_ratio = 0.75;
        double max_y_diff_ratio = 1.0;
        double max_distance_ratio = 0.8;
        double min_distance_ratio = 0.1;
        LightBarColor target_color = LightBarColor::BLUE;
    };

    explicit ArmorDetector(const Params & params);

    std::vector<ArmorCandidate> match(const std::vector<LightBar> & lights);

    const debug::ArmorMatchDebugData & getArmorMatchDebugData() const { return armor_debug_; }

private:
    ArmorGeometry buildGeometry(const LightBar & left, const LightBar & right) const;
    bool checkLightColor(const LightBar & left, const LightBar & right) const;
    bool checkArmorGeometry(const ArmorGeometry & geometry, std::string * detail) const;

    Params params_;
    debug::ArmorMatchDebugData armor_debug_;

    static constexpr double kLargeArmorDistanceRatio = 2.8;
};

} // namespace armor_detector
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/include/armor_detector/detector/ArmorDetector.hpp
git commit -m "feat: add ArmorDetector header with match interface"
```

---

## Task 3: Implement ArmorDetector — Constructor and Color Check

**Files:**
- Modify: `src/armor_detector/src/detector/ArmorDetector.cpp`

- [ ] **Step 1: Write constructor and checkLightColor**

Replace the contents of `src/armor_detector/src/detector/ArmorDetector.cpp` with:

```cpp
#include "armor_detector/detector/ArmorDetector.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace armor_detector
{

ArmorDetector::ArmorDetector(const Params & params)
    : params_(params)
{
}

bool ArmorDetector::checkLightColor(const LightBar & left, const LightBar & right) const
{
    if (left.color != right.color) {
        return false;
    }
    if (left.color != params_.target_color) {
        return false;
    }
    return true;
}

} // namespace armor_detector
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/detector/ArmorDetector.cpp
git commit -m "feat: ArmorDetector constructor and color check"
```

---

## Task 4: Implement ArmorDetector — buildGeometry

**Files:**
- Modify: `src/armor_detector/src/detector/ArmorDetector.cpp`

- [ ] **Step 1: Add buildGeometry implementation**

Append the following to `src/armor_detector/src/detector/ArmorDetector.cpp` (before the closing `} // namespace armor_detector`):

```cpp
ArmorGeometry ArmorDetector::buildGeometry(const LightBar & left, const LightBar & right) const
{
    ArmorGeometry geo;
    geo.paired_lights = {left, right};
    geo.corners = {left.top, right.top, right.bottom, left.bottom};

    // angle_diff_deg: smallest angular difference in degrees
    double diff = std::abs(tools::radToDeg(left.angle - right.angle));
    diff = std::min(diff, 180.0 - diff);
    geo.angle_diff_deg = diff;

    // length_ratio: min/max of the two light bar lengths
    double max_len = std::max(left.length, right.length);
    double min_len = std::min(left.length, right.length);
    geo.length_diff = (max_len > 1e-6) ? (min_len / max_len) : 0.0;

    // Local coordinate differences
    double global_x_diff = right.center.x - left.center.x;
    double global_y_diff = right.center.y - left.center.y;
    double sinx = std::sin(left.angle);
    double cosx = std::cos(left.angle);
    double local_x = std::abs(-global_x_diff * sinx + global_y_diff * cosx);
    double local_y = std::abs(global_x_diff * cosx + global_y_diff * sinx);
    double mean_length = (left.length + right.length) / 2.0;

    geo.x_diff_ratio = (mean_length > 1e-6) ? (local_x / mean_length) : 0.0;
    geo.y_diff_ratio = (mean_length > 1e-6) ? (local_y / mean_length) : 0.0;

    // Distance ratio and armor type
    double distance = cv::norm(left.center - right.center);
    if (distance > 1e-6 && mean_length > 1e-6) {
        double distance_by_length = distance / mean_length;
        geo.distance_ratio = mean_length / distance;
        geo.type = (distance_by_length > kLargeArmorDistanceRatio) ? ArmorType::LARGE
                                                                  : ArmorType::SMALL;
    }
    else {
        geo.distance_ratio = 0.0;
        geo.type = ArmorType::NONE;
    }

    return geo;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/detector/ArmorDetector.cpp
git commit -m "feat: ArmorDetector buildGeometry implementation"
```

---

## Task 5: Implement ArmorDetector — checkArmorGeometry

**Files:**
- Modify: `src/armor_detector/src/detector/ArmorDetector.cpp`

- [ ] **Step 1: Add checkArmorGeometry implementation**

Append the following to `src/armor_detector/src/detector/ArmorDetector.cpp` (before the closing `} // namespace armor_detector`):

```cpp
bool ArmorDetector::checkArmorGeometry(
    const ArmorGeometry & geometry, std::string * detail) const
{
    std::ostringstream oss;

    if (geometry.type == ArmorType::NONE) {
        oss << "type=NONE";
        *detail = oss.str();
        return false;
    }

    if (geometry.angle_diff_deg > params_.max_angle_diff) {
        oss << "angle=" << geometry.angle_diff_deg << " > " << params_.max_angle_diff;
        *detail = oss.str();
        return false;
    }

    if (geometry.length_diff < params_.min_length_ratio) {
        oss << "len_ratio=" << geometry.length_diff << " < " << params_.min_length_ratio;
        *detail = oss.str();
        return false;
    }

    if (geometry.x_diff_ratio < params_.min_x_diff_ratio) {
        oss << "x_diff=" << geometry.x_diff_ratio << " < " << params_.min_x_diff_ratio;
        *detail = oss.str();
        return false;
    }

    if (geometry.y_diff_ratio > params_.max_y_diff_ratio) {
        oss << "y_diff=" << geometry.y_diff_ratio << " > " << params_.max_y_diff_ratio;
        *detail = oss.str();
        return false;
    }

    if (geometry.distance_ratio < params_.min_distance_ratio ||
        geometry.distance_ratio > params_.max_distance_ratio) {
        oss << "dist_ratio=" << geometry.distance_ratio
            << " not in [" << params_.min_distance_ratio << ", "
            << params_.max_distance_ratio << "]";
        *detail = oss.str();
        return false;
    }

    return true;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/detector/ArmorDetector.cpp
git commit -m "feat: ArmorDetector checkArmorGeometry implementation"
```

---

## Task 6: Implement ArmorDetector — match

**Files:**
- Modify: `src/armor_detector/src/detector/ArmorDetector.cpp`

- [ ] **Step 1: Add match implementation**

Append the following to `src/armor_detector/src/detector/ArmorDetector.cpp` (before the closing `} // namespace armor_detector`):

```cpp
std::vector<ArmorCandidate> ArmorDetector::match(const std::vector<LightBar> & lights)
{
    armor_debug_.candidates.clear();
    armor_debug_.rejected_armors.clear();

    std::vector<ArmorCandidate> candidates;

    // Sort by center.x to ensure left-to-right ordering
    std::vector<LightBar> sorted_lights = lights;
    std::sort(sorted_lights.begin(), sorted_lights.end(),
              [](const LightBar & a, const LightBar & b) { return a.center.x < b.center.x; });

    for (std::size_t i = 0; i < sorted_lights.size(); ++i) {
        for (std::size_t j = i + 1; j < sorted_lights.size(); ++j) {
            const auto & left = sorted_lights[i];
            const auto & right = sorted_lights[j];

            // Check color
            if (!checkLightColor(left, right)) {
                debug::RejectedArmor rejected;
                rejected.geometry.paired_lights = {left, right};
                rejected.reason = debug::DebugRejectReason::BAD_COLOR;
                rejected.detail = "color mismatch or != target";
                armor_debug_.rejected_armors.push_back(rejected);
                continue;
            }

            // Build geometry
            ArmorGeometry geometry = buildGeometry(left, right);

            // Check geometry thresholds
            std::string detail;
            if (!checkArmorGeometry(geometry, &detail)) {
                debug::RejectedArmor rejected;
                rejected.geometry = geometry;
                if (detail.find("angle") != std::string::npos) {
                    rejected.reason = debug::DebugRejectReason::BAD_ANGLE;
                }
                else if (detail.find("len_ratio") != std::string::npos) {
                    rejected.reason = debug::DebugRejectReason::BAD_LENGTH_RATIO;
                }
                else if (detail.find("x_diff") != std::string::npos) {
                    rejected.reason = debug::DebugRejectReason::BAD_X_DIFF;
                }
                else if (detail.find("y_diff") != std::string::npos) {
                    rejected.reason = debug::DebugRejectReason::BAD_Y_DIFF;
                }
                else if (detail.find("dist_ratio") != std::string::npos) {
                    rejected.reason = debug::DebugRejectReason::BAD_DISTANCE;
                }
                else {
                    rejected.reason = debug::DebugRejectReason::UNKNOWN;
                }
                rejected.detail = detail;
                armor_debug_.rejected_armors.push_back(rejected);
                continue;
            }

            // Passed all checks
            ArmorCandidate candidate;
            candidate.geometry = geometry;
            candidates.push_back(candidate);
            armor_debug_.candidates.push_back(candidate);
        }
    }

    return candidates;
}

} // namespace armor_detector
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/detector/ArmorDetector.cpp
git commit -m "feat: ArmorDetector match implementation"
```

---

## Task 7: Build and Verify ArmorDetector Compiles

**Files:** None (verification only)

- [ ] **Step 1: Build the package**

Run: `colcon build --packages-select armor_detector`
Expected: BUILD SUCCESSFUL

- [ ] **Step 2: Run lint tests**

Run: `colcon test --packages-select armor_detector`
Expected: Tests pass (uncrustify, etc.)

- [ ] **Step 3: Fix any lint/format issues**

If uncrustify fails, run: `ament_clang_format --fix src/armor_detector/src/detector/ArmorDetector.cpp src/armor_detector/include/armor_detector/detector/ArmorDetector.hpp`

Then rebuild and re-test until clean.

- [ ] **Step 4: Commit if fixes were needed**

```bash
git add -A
git commit -m "fix: format ArmorDetector files for ament lint"
```

---

## Task 8: Implement NumberClassifier Header

**Files:**
- Modify: `src/armor_detector/include/armor_detector/NumberClassifier.hpp`

- [ ] **Step 1: Write the NumberClassifier header**

Replace the contents of `src/armor_detector/include/armor_detector/NumberClassifier.hpp` with:

```cpp
#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

namespace armor_detector
{

class NumberClassifier
{
public:
    struct Params {
        std::string model_path;
        float confidence_threshold = 0.5f;
    };

    explicit NumberClassifier(const Params & params);

    std::vector<ClassifiedArmor> classify(
        const std::vector<ArmorCandidate> & candidates,
        const cv::Mat & img_bgr);

    const debug::ClassificationDebugData & getClassificationDebugData() const
    {
        return classification_debug_;
    }

private:
    ArmorClassification classifyOne(const ArmorGeometry & geometry, const cv::Mat & img_bgr);
    cv::Mat getNumberROI(const cv::Mat & img_bgr, const ArmorGeometry & geometry) const;
    cv::Mat getArmorPattern(const cv::Mat & img_bgr, const ArmorGeometry & geometry) const;
    bool checkArmorName(const ArmorClassification & classification) const;
    bool checkArmorType(
        const ArmorGeometry & geometry,
        const ArmorClassification & classification) const;
    std::vector<ClassifiedArmor> deduplicate(std::vector<ClassifiedArmor> armors);

    static cv::Mat softmax(const cv::Mat & logits);
    static float sigmoid(float x);
    static ArmorName intToArmorName(int id);

    Params params_;
    cv::dnn::Net net_;
    debug::ClassificationDebugData classification_debug_;
};

} // namespace armor_detector
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/include/armor_detector/NumberClassifier.hpp
git commit -m "feat: add NumberClassifier header"
```

---

## Task 9: Implement NumberClassifier — Constructor and Utility Methods

**Files:**
- Create: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Write constructor, softmax, sigmoid, intToArmorName**

Create `src/armor_detector/src/NumberClassifier.cpp` with:

```cpp
#include "armor_detector/NumberClassifier.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace armor_detector
{

NumberClassifier::NumberClassifier(const Params & params)
    : params_(params)
{
    if (params_.model_path.empty()) {
        throw std::runtime_error("NumberClassifier: model_path is empty");
    }
    net_ = cv::dnn::readNetFromONNX(params_.model_path);
    if (net_.empty()) {
        throw std::runtime_error("NumberClassifier: failed to load model from " +
                                 params_.model_path);
    }
}

cv::Mat NumberClassifier::softmax(const cv::Mat & logits)
{
    cv::Mat exps;
    cv::exp(logits, exps);
    float sum = static_cast<float>(cv::sum(exps)[0]);
    return exps / sum;
}

float NumberClassifier::sigmoid(float x)
{
    return 1.0f / (1.0f + std::exp(-x));
}

ArmorName NumberClassifier::intToArmorName(int id)
{
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

} // namespace armor_detector
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier constructor and utility methods"
```

---

## Task 10: Implement NumberClassifier — getNumberROI

**Files:**
- Modify: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Add getNumberROI implementation**

Append the following to `src/armor_detector/src/NumberClassifier.cpp` (before the closing `} // namespace armor_detector`):

```cpp
cv::Mat NumberClassifier::getNumberROI(
    const cv::Mat & img_bgr, const ArmorGeometry & geometry) const
{
    const auto & left = geometry.paired_lights[0];
    const auto & right = geometry.paired_lights[1];

    // Source points from the armor corners (left side top/bottom, right side bottom/top)
    std::vector<cv::Point2f> src_pts = {
        left.top,
        left.bottom,
        right.bottom,
        right.top
    };

    // Destination: 28x28 ROI with 7px vertical padding
    std::vector<cv::Point2f> dst_pts = {
        {0.f, 7.f},
        {0.f, 21.f},
        {28.f, 21.f},
        {28.f, 7.f}
    };

    cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::Mat roi;
    cv::warpPerspective(img_bgr, roi, M, cv::Size(28, 28));
    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);

    return roi;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier getNumberROI implementation"
```

---

## Task 11: Implement NumberClassifier — getArmorPattern

**Files:**
- Modify: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Add getArmorPattern implementation**

Append the following to `src/armor_detector/src/NumberClassifier.cpp` (before the closing `} // namespace armor_detector`):

```cpp
cv::Mat NumberClassifier::getArmorPattern(
    const cv::Mat & img_bgr, const ArmorGeometry & geometry) const
{
    // Armor corners: left_top, right_top, right_bottom, left_bottom
    cv::Point2f armor_top_left = geometry.corners[0];
    cv::Point2f armor_top_right = geometry.corners[1];
    cv::Point2f armor_bottom_right = geometry.corners[2];
    cv::Point2f armor_bottom_left = geometry.corners[3];

    // Bounding box of the armor pattern
    float min_x = std::min({armor_top_left.x, armor_top_right.x,
                            armor_bottom_right.x, armor_bottom_left.x});
    float max_x = std::max({armor_top_left.x, armor_top_right.x,
                            armor_bottom_right.x, armor_bottom_left.x});
    float min_y = std::min({armor_top_left.y, armor_top_right.y,
                            armor_bottom_right.y, armor_bottom_left.y});
    float max_y = std::max({armor_top_left.y, armor_top_right.y,
                            armor_bottom_right.y, armor_bottom_left.y});

    // Check bounds
    if (min_x < 0 || min_y < 0 ||
        max_x >= img_bgr.cols || max_y >= img_bgr.rows) {
        return cv::Mat();
    }

    // Extract ROI using bounding box and clone to avoid lifetime issues
    cv::Rect roi_rect(
        static_cast<int>(min_x), static_cast<int>(min_y),
        static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));

    if (roi_rect.width <= 0 || roi_rect.height <= 0) {
        return cv::Mat();
    }

    return img_bgr(roi_rect).clone();
}
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier getArmorPattern implementation"
```

---

## Task 12: Implement NumberClassifier — classifyOne

**Files:**
- Modify: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Add classifyOne implementation**

Append the following to `src/armor_detector/src/NumberClassifier.cpp` (before the closing `} // namespace armor_detector`):

```cpp
ArmorClassification NumberClassifier::classifyOne(
    const ArmorGeometry & geometry, const cv::Mat & img_bgr)
{
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
    float merged[6] = {
        probs.at<float>(0, 0) + probs.at<float>(0, 6) + probs.at<float>(0, 7) +
            probs.at<float>(0, 8) + probs.at<float>(0, 9),
        probs.at<float>(0, 1),
        probs.at<float>(0, 2),
        probs.at<float>(0, 3),
        probs.at<float>(0, 4),
        probs.at<float>(0, 5)
    };

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
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier classifyOne with CNN inference"
```

---

## Task 13: Implement NumberClassifier — checkArmorName and checkArmorType

**Files:**
- Modify: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Add checkArmorName and checkArmorType implementations**

Append the following to `src/armor_detector/src/NumberClassifier.cpp` (before the closing `} // namespace armor_detector`):

```cpp
bool NumberClassifier::checkArmorName(const ArmorClassification & classification) const
{
    return classification.name != ArmorName::NONE &&
           classification.confidence > params_.confidence_threshold;
}

bool NumberClassifier::checkArmorType(
    const ArmorGeometry & geometry,
    const ArmorClassification & classification) const
{
    if (geometry.type == ArmorType::LARGE) {
        return classification.name == ArmorName::ONE;
    }
    else {
        return classification.name != ArmorName::ONE;
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier checkArmorName and checkArmorType"
```

---

## Task 14: Implement NumberClassifier — deduplicate

**Files:**
- Modify: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Add deduplicate implementation**

Append the following to `src/armor_detector/src/NumberClassifier.cpp` (before the closing `} // namespace armor_detector`):

```cpp
std::vector<ClassifiedArmor> NumberClassifier::deduplicate(
    std::vector<ClassifiedArmor> armors)
{
    std::vector<ClassifiedArmor> result;
    std::vector<bool> removed(armors.size(), false);

    for (std::size_t i = 0; i < armors.size(); ++i) {
        if (removed[i]) continue;

        for (std::size_t j = i + 1; j < armors.size(); ++j) {
            if (removed[j]) continue;

            const auto & geo_i = armors[i].geometry;
            const auto & geo_j = armors[j].geometry;

            // Check if they share any light bar
            bool same_left = (geo_i.paired_lights[0].id == geo_j.paired_lights[0].id);
            bool same_right = (geo_i.paired_lights[1].id == geo_j.paired_lights[1].id);
            bool share_any = same_left || same_right;

            if (!share_any) continue;

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
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier deduplicate implementation"
```

---

## Task 15: Implement NumberClassifier — classify

**Files:**
- Modify: `src/armor_detector/src/NumberClassifier.cpp`

- [ ] **Step 1: Add classify implementation**

Append the following to `src/armor_detector/src/NumberClassifier.cpp` (before the closing `} // namespace armor_detector`):

```cpp
std::vector<ClassifiedArmor> NumberClassifier::classify(
    const std::vector<ArmorCandidate> & candidates,
    const cv::Mat & img_bgr)
{
    classification_debug_.classified_armors.clear();
    classification_debug_.number_rois.clear();

    std::vector<ClassifiedArmor> all_results;

    for (const auto & candidate : candidates) {
        const auto & geometry = candidate.geometry;

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
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/NumberClassifier.cpp
git commit -m "feat: NumberClassifier classify with deduplication"
```

---

## Task 16: Update CMakeLists.txt

**Files:**
- Modify: `src/armor_detector/CMakeLists.txt`

- [ ] **Step 1: Add NumberClassifier.cpp and debug view sources to the executable**

In `CMakeLists.txt`, add the new source files to the `add_executable` call. Change:

```cmake
add_executable(armor_detector_node
  src/DetectorNode.cpp
  src/detector/ArmorDetector.cpp
  src/detector/Detector.cpp
  src/CameraProvider.cpp
  src/PoseSolver.cpp
  src/detector/LightDetector.cpp
  src/debug/DebugGUI.cpp
  src/debug/DebugMsg.cpp
  src/debug/DebugKeyHandler.cpp
  src/debug/DebugPreprocessView.cpp
  src/debug/DebugLightView.cpp
  src/debug/DebugTiming.cpp
  src/debug/DebugLayerController.cpp
)
```

To:

```cmake
add_executable(armor_detector_node
  src/DetectorNode.cpp
  src/detector/ArmorDetector.cpp
  src/detector/Detector.cpp
  src/CameraProvider.cpp
  src/PoseSolver.cpp
  src/detector/LightDetector.cpp
  src/NumberClassifier.cpp
  src/debug/DebugGUI.cpp
  src/debug/DebugMsg.cpp
  src/debug/DebugKeyHandler.cpp
  src/debug/DebugPreprocessView.cpp
  src/debug/DebugLightView.cpp
  src/debug/DebugArmorMatchView.cpp
  src/debug/DebugClassificationView.cpp
  src/debug/DebugTiming.cpp
  src/debug/DebugLayerController.cpp
)
```

- [ ] **Step 2: Add model directory to install**

In `CMakeLists.txt`, change:

```cmake
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
```

To:

```cmake
install(DIRECTORY
  launch
  config
  model
  DESTINATION share/${PROJECT_NAME}
)
```

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/CMakeLists.txt
git commit -m "feat: add NumberClassifier and debug views to CMake, install model/"
```

---

## Task 17: Update config.yaml

**Files:**
- Modify: `src/armor_detector/config/config.yaml`

- [ ] **Step 1: Add armor match and number classifier parameters**

Replace the contents of `src/armor_detector/config/config.yaml` with:

```yaml
armor_detector_node_cpp:
  ros__parameters:
    # 目标颜色
    target_color: "BLUE"

    # 预处理
    gray_threshold: 100
    color_threshold: 100

    # 灯条
    min_contours_area: 30
    min_contours_ratio: 0.06
    max_contours_ratio: 0.5

    # 装甲板匹配
    max_angle_diff: 15.0
    min_length_ratio: 0.7
    min_x_diff_ratio: 0.75
    max_y_diff_ratio: 1.0
    max_distance_ratio: 0.8
    min_distance_ratio: 0.1

    # 数字分类
    model_path: "model/number_cnn.onnx"
    number_threshold: 0.5

    # Debug
    debug:
      show: true
      rosbag_control: true
      rosbag_player_node: "/rosbag2_player"
      preprocess: false
      lights: true
      armor_match: false
      classification: false
      pose: false
      stats_interval: 50
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/config/config.yaml
git commit -m "feat: add armor match and classification config defaults"
```

---

## Task 18: Update DetectorNode Header

**Files:**
- Modify: `src/armor_detector/include/armor_detector/DetectorNode.hpp`

- [ ] **Step 1: Add includes and members for ArmorDetector and NumberClassifier**

In `src/armor_detector/include/armor_detector/DetectorNode.hpp`, add includes after the existing detector includes:

```cpp
#include "armor_detector/detector/ArmorDetector.hpp"
#include "armor_detector/NumberClassifier.hpp"
```

In the private section of `DetectorNode`, add new members after `LightDetector light_detector_;`:

```cpp
    ArmorDetector armor_detector_;
    NumberClassifier number_classifier_;
```

The full private section should now look like:

```cpp
    // 检测组件
    CameraProvider camera_provider_;
    Detector detector_;
    LightDetector light_detector_;
    ArmorDetector armor_detector_;
    NumberClassifier number_classifier_;
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/include/armor_detector/DetectorNode.hpp
git commit -m "feat: add ArmorDetector and NumberClassifier to DetectorNode"
```

---

## Task 19: Update DetectorNode Implementation — initDetectors and initDebug

**Files:**
- Modify: `src/armor_detector/src/DetectorNode.cpp`

- [ ] **Step 1: Add includes for new debug views and ament_index_cpp**

At the top of `src/armor_detector/src/DetectorNode.cpp`, add:

```cpp
#include "armor_detector/debug/DebugArmorMatch.hpp"
#include "armor_detector/debug/DebugClassification.hpp"
```

- [ ] **Step 2: Update initDetectors to initialize armor_detector_ and number_classifier_**

Replace the `initDetectors()` method with:

```cpp
void DetectorNode::initDetectors()
{
    std::string target_color_str = this->declare_parameter<std::string>("target_color", "BLUE");
    LightBarColor target_color =
        (target_color_str == "RED") ? LightBarColor::RED : LightBarColor::BLUE;

    Detector::Params det_params;
    det_params.gray_threshold = this->declare_parameter<int>("gray_threshold", 100);
    det_params.color_threshold = this->declare_parameter<int>("color_threshold", 100);
    det_params.target_color = target_color;
    detector_ = Detector(det_params);

    LightDetector::Params light_params;
    light_params.min_contours_area = this->declare_parameter<int>("min_contours_area", 30);
    light_params.min_contours_ratio =
        static_cast<float>(this->declare_parameter<double>("min_contours_ratio", 0.06));
    light_params.max_contours_ratio =
        static_cast<float>(this->declare_parameter<double>("max_contours_ratio", 0.5));
    light_detector_ = LightDetector(light_params);

    ArmorDetector::Params armor_params;
    armor_params.max_angle_diff = this->declare_parameter<double>("max_angle_diff", 15.0);
    armor_params.min_length_ratio = this->declare_parameter<double>("min_length_ratio", 0.7);
    armor_params.min_x_diff_ratio = this->declare_parameter<double>("min_x_diff_ratio", 0.75);
    armor_params.max_y_diff_ratio = this->declare_parameter<double>("max_y_diff_ratio", 1.0);
    armor_params.max_distance_ratio = this->declare_parameter<double>("max_distance_ratio", 0.8);
    armor_params.min_distance_ratio = this->declare_parameter<double>("min_distance_ratio", 0.1);
    armor_params.target_color = target_color;
    armor_detector_ = ArmorDetector(armor_params);

    NumberClassifier::Params number_params;
    std::string model_relative_path =
        this->declare_parameter<std::string>("model_path", "model/number_cnn.onnx");
    auto package_share = ament_index_cpp::get_package_share_directory("armor_detector");
    number_params.model_path = package_share + "/" + model_relative_path;
    number_params.confidence_threshold =
        static_cast<float>(this->declare_parameter<double>("number_threshold", 0.5));
    number_classifier_ = NumberClassifier(number_params);
}
```

- [ ] **Step 3: Update initDebug to register new observers**

In `initDebug()`, add the following two lines after the `DebugLightView` observer registration:

```cpp
    debug_hub_.addObserver(
        std::make_shared<debug::DebugArmorMatchView>(debug_gui_, layer_state_));
    debug_hub_.addObserver(
        std::make_shared<debug::DebugClassificationView>(debug_gui_, layer_state_));
```

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/src/DetectorNode.cpp
git commit -m "feat: integrate ArmorDetector and NumberClassifier into DetectorNode"
```

---

## Task 20: Update DetectorNode — run() Pipeline

**Files:**
- Modify: `src/armor_detector/src/DetectorNode.cpp`

- [ ] **Step 1: Update run() to include armor match and classification stages**

Replace the `run()` method with:

```cpp
void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);

    debug::DebugFrameContext ctx;
    ctx.frame_index = frame_index_++;
    ctx.stamp = frame.timestamp;
    ctx.source_bgr = frame.image;

    // Only clone display_bgr when debug is enabled
    if (debug_config_.show) {
        ctx.display_bgr = frame.image.clone();
    }

    debug_hub_.onFrameStart(ctx);

    cv::Mat img_thre = detector_.preprocess(frame.image);
    debug_hub_.onPreprocess(ctx, detector_.getPreprocessDebugData());

    auto lights = light_detector_.findLights(img_thre, frame.image);
    debug_hub_.onLights(ctx, light_detector_.getLightDebugData());

    auto candidates = armor_detector_.match(lights);
    debug_hub_.onArmorMatch(ctx, armor_detector_.getArmorMatchDebugData());

    auto classified = number_classifier_.classify(candidates, frame.image);
    debug_hub_.onClassification(ctx, number_classifier_.getClassificationDebugData());

    debug_hub_.onFrameEnd(ctx);

    if (debug_config_.show) {
        debug_gui_.setFrame("debug_show", ctx.display_bgr);
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/DetectorNode.cpp
git commit -m "feat: add armor match and classification to run() pipeline"
```

---

## Task 21: Update DebugArmorMatchView Header and Implementation

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugArmorMatch.hpp`
- Create: `src/armor_detector/src/debug/DebugArmorMatchView.cpp`

- [ ] **Step 1: Update DebugArmorMatch.hpp to match DebugLightView pattern**

Replace the contents of `src/armor_detector/include/armor_detector/debug/DebugArmorMatch.hpp` with:

```cpp
#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 装甲板匹配可视化 Observer。
 *
 * 用于显示候选装甲板、被拒绝装甲板及其几何拒绝原因。
 */
class DebugArmorMatchView : public IDebugObserver
{
public:
    DebugArmorMatchView(DebugGUI & gui, DebugLayerState & layer_state);

    void onArmorMatch(DebugFrameContext & context, const ArmorMatchDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
    DebugLayerState & layer_state_;
};

} // namespace armor_detector::debug
```

- [ ] **Step 2: Write DebugArmorMatchView.cpp**

Create `src/armor_detector/src/debug/DebugArmorMatchView.cpp` with:

```cpp
#include "armor_detector/debug/DebugArmorMatch.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugArmorMatchView::DebugArmorMatchView(DebugGUI & gui, DebugLayerState & layer_state)
    : gui_(&gui), layer_state_(layer_state)
{
}

void DebugArmorMatchView::onArmorMatch(
    DebugFrameContext & context,
    const ArmorMatchDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;
    if (!layer_state_.enabled(DebugLayer::ARMOR_MATCH)) return;
    if (context.display_bgr.empty()) return;

    cv::Mat & display = context.display_bgr;

    // Draw accepted candidates: cyan four-point outline
    for (const auto & candidate : data.candidates) {
        const auto & corners = candidate.geometry.corners;
        for (int i = 0; i < 4; ++i) {
            cv::line(display, corners[i], corners[(i + 1) % 4],
                     cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        }
    }

    // Draw rejected armors: red four-point outline + detail text
    for (const auto & rejected : data.rejected_armors) {
        const auto & corners = rejected.geometry.corners;
        for (int i = 0; i < 4; ++i) {
            cv::line(display, corners[i], corners[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
        // Center point for text
        cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;
        cv::putText(display, rejected.detail,
                    cv::Point(static_cast<int>(center.x) + 5,
                              static_cast<int>(center.y)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
}

} // namespace armor_detector::debug
```

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugArmorMatch.hpp \
        src/armor_detector/src/debug/DebugArmorMatchView.cpp
git commit -m "feat: implement DebugArmorMatchView"
```

---

## Task 22: Update DebugClassificationView Header and Implementation

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugClassification.hpp`
- Create: `src/armor_detector/src/debug/DebugClassificationView.cpp`

- [ ] **Step 1: Update DebugClassification.hpp to match DebugLightView pattern**

Replace the contents of `src/armor_detector/include/armor_detector/debug/DebugClassification.hpp` with:

```cpp
#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 数字分类可视化 Observer。
 *
 * 建议显示装甲板类别、置信度、number ROI 拼图。低置信度样本可交给
 * DebugRoiRecorder 保存。
 */
class DebugClassificationView : public IDebugObserver
{
public:
    DebugClassificationView(DebugGUI & gui, DebugLayerState & layer_state);

    void onClassification(
        DebugFrameContext & context,
        const ClassificationDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
    DebugLayerState & layer_state_;
};

} // namespace armor_detector::debug
```

- [ ] **Step 2: Write DebugClassificationView.cpp**

Create `src/armor_detector/src/debug/DebugClassificationView.cpp` with:

```cpp
#include "armor_detector/debug/DebugClassification.hpp"

#include <opencv2/imgproc.hpp>

#include <sstream>

namespace armor_detector::debug
{

DebugClassificationView::DebugClassificationView(
    DebugGUI & gui, DebugLayerState & layer_state)
    : gui_(&gui), layer_state_(layer_state)
{
}

void DebugClassificationView::onClassification(
    DebugFrameContext & context,
    const ClassificationDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;

    if (!layer_state_.enabled(DebugLayer::CLASSIFICATION)) {
        gui_->clearFrame("number_rois");
        return;
    }

    if (context.display_bgr.empty()) return;

    cv::Mat & display = context.display_bgr;

    // Draw classification labels on display image
    for (const auto & armor : data.classified_armors) {
        const auto & corners = armor.geometry.corners;
        cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) / 4.0f;

        std::ostringstream oss;
        oss << static_cast<int>(armor.classification.name) << " "
            << static_cast<int>(armor.classification.confidence * 100) << "%";
        cv::putText(display, oss.str(),
                    cv::Point(static_cast<int>(center.x) - 20,
                              static_cast<int>(center.y) - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    // Concatenate number ROIs into a single window
    if (data.number_rois.empty()) {
        gui_->clearFrame("number_rois");
        return;
    }

    // Convert all ROIs to BGR for consistent hconcat
    std::vector<cv::Mat> bgr_rois;
    for (const auto & roi : data.number_rois) {
        cv::Mat bgr;
        if (roi.channels() == 1) {
            cv::cvtColor(roi, bgr, cv::COLOR_GRAY2BGR);
        }
        else {
            bgr = roi;
        }
        bgr_rois.push_back(bgr);
    }

    cv::Mat concatenated;
    cv::hconcat(bgr_rois, concatenated);
    gui_->setFrame("number_rois", concatenated);
}

} // namespace armor_detector::debug
```

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugClassification.hpp \
        src/armor_detector/src/debug/DebugClassificationView.cpp
git commit -m "feat: implement DebugClassificationView"
```

---

## Task 23: Create Model Directory Placeholder

**Files:**
- Create: `src/armor_detector/model/` directory

- [ ] **Step 1: Create the model directory**

Run: `mkdir -p src/armor_detector/model`

Note: The actual `number_cnn.onnx` file must be provided by the user and placed at `src/armor_detector/model/number_cnn.onnx`.

- [ ] **Step 2: Add a README placeholder**

Create `src/armor_detector/model/README.md` with:

```markdown
# Model Directory

Place `number_cnn.onnx` here before running the detector.

The ONNX model expects:
- Input: 1x1x28x28 grayscale image, normalized to [0,1]
- Output: 1x11 tensor
  - [0]: objectness logit
  - [1..10]: 10 class logits (softmax applied internally)
```

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/model/
git commit -m "feat: add model directory with README"
```

---

## Task 24: Build and Full Verification

**Files:** None (verification only)

- [ ] **Step 1: Build the package**

Run: `colcon build --packages-select armor_detector`
Expected: BUILD SUCCESSFUL

- [ ] **Step 2: Run lint tests**

Run: `colcon test --packages-select armor_detector`
Expected: All tests pass

- [ ] **Step 3: Fix any lint/format issues**

If uncrustify or other lint checks fail, fix them and rebuild. Common issues:
- Include order (system first, then project)
- Indentation (4 spaces)
- Column limit (120 chars)
- Namespace indentation (All)

- [ ] **Step 4: Quick smoke test (node starts without crash)**

Run: `source install/setup.bash && ros2 run armor_detector armor_detector_node --ros-args -p debug.show:=false`
Expected: Node starts, prints "armor_detector节点已启动.", Ctrl+C to exit cleanly

- [ ] **Step 5: Full rosbag test with debug**

Run: `source install/setup.bash && ros2 launch armor_detector video1.launch.py`
Expected:
- No readNetFromONNX exceptions
- debug_show window appears
- Press 2 to toggle lights
- Press 3 to toggle armor match boxes
- Press 4 to toggle classification labels and number_rois window
- Every 50 frames, terminal shows preprocess/lights/armor_match/classification average times

- [ ] **Step 6: Final commit if any fixes were needed**

```bash
git add -A
git commit -m "fix: address lint issues for armor match and classification"
```
