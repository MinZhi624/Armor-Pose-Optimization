# Light 检测移植实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从参考项目移植灯条检测算法（预处理 + 灯条检测）到当前框架，附带 debug 可视化。

**Architecture:** Detector（双阈值预处理）→ LightDetector（轮廓检测 + 几何筛选 + 颜色判定），结果通过 DebugHub 事件分发给 DebugPreprocessView 和 DebugLightView，在单窗口 `debug_show` 上叠加图层显示。

**Tech Stack:** C++17, OpenCV (imgproc, highgui), ROS2 (rclcpp), Eigen3, yaml-cpp

## Global Constraints

- LLVM 代码风格（缩进 4 格，行宽 120，`else` 前换行）
- 角度内部存储：弧度（`LightBar.angle`）
- Debug 绘制遵守 `docs/DebugGUI.md`（BGR 颜色、统一字体、单窗口图层叠加）
- `cv::Mat` 生命周期遵守 `docs/Conventions.md`（默认浅拷贝，跨帧才 clone）
- 算法逻辑完全不修改，原样移植自参考项目

---

### Task 1: 数据结构修改（LightBar + DebugFrameContext）

**Files:**
- Modify: `src/armor_detector/include/armor_detector/types/ArmorData.hpp`
- Modify: `src/armor_detector/include/armor_detector/debug/DebugData.hpp`

**Interfaces:**
- Produces: `LightBar.angle` (double, 弧度), `DebugFrameContext.display_bgr` (cv::Mat)

- [ ] **Step 1: 修改 LightBar.angle_deg → angle**

在 `src/armor_detector/include/armor_detector/types/ArmorData.hpp` 中，将 `LightBar` 结构体的 `angle_deg` 改名为 `angle`：

```cpp
struct LightBar
{
    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f top;
    cv::Point2f bottom;
    double angle = 0.0;       // 弧度（原 angle_deg，改为弧度匹配参考项目）
    double length = 0.0;
    double width = 0.0;
    int area = 0;
    int id = -1;
    LightBarColor color = LightBarColor::NONE;
};
```

- [ ] **Step 2: 在 DebugFrameContext 中添加 display_bgr**

在 `src/armor_detector/include/armor_detector/debug/DebugData.hpp` 中，给 `DebugFrameContext` 添加 `display_bgr` 字段：

```cpp
struct DebugFrameContext
{
    std::size_t frame_index = 0;
    builtin_interfaces::msg::Time stamp;
    cv::Mat source_bgr;      // 原始图像（只读）
    cv::Mat display_bgr;     // 显示用图像（Observer 可在其上绘制叠加图层）
};
```

- [ ] **Step 3: 编译验证**

```bash
cd /home/minzhi/Desktop/ws_homework1
colcon build --packages-select armor_detector
```

Expected: 编译成功（现有代码中没有引用 `angle_deg` 的地方，因为算法 stub 都是空的）。

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/types/ArmorData.hpp \
        src/armor_detector/include/armor_detector/debug/DebugData.hpp
git commit -m "refactor: LightBar.angle_deg→angle(radians), add DebugFrameContext.display_bgr"
```

---

### Task 2: Detector 预处理

**Files:**
- Modify: `src/armor_detector/include/armor_detector/detector/Detector.hpp`
- Modify: `src/armor_detector/src/detector/Detector.cpp`

**Interfaces:**
- Consumes: `LightBarColor` enum from `ArmorData.hpp`
- Consumes: `debug::PreprocessDebugData` from `DebugData.hpp`
- Produces: `armor_detector::Detector` class with `preprocess(cv::Mat) → cv::Mat` and `getPreprocessDebugData() → PreprocessDebugData`

- [ ] **Step 1: 写 Detector.hpp**

完整替换 `src/armor_detector/include/armor_detector/detector/Detector.hpp`：

```cpp
#pragma once

#include "armor_detector/types/ArmorData.hpp"
#include "armor_detector/debug/DebugData.hpp"

#include <opencv2/core.hpp>

namespace armor_detector
{

class Detector
{
public:
    struct Params {
        int gray_threshold = 100;
        int color_threshold = 100;
        LightBarColor target_color = LightBarColor::BLUE;
    };

    explicit Detector(const Params & params);

    cv::Mat preprocess(const cv::Mat & img_bgr);

    const debug::PreprocessDebugData & getPreprocessDebugData() const { return preprocess_debug_; }

private:
    Params params_;
    debug::PreprocessDebugData preprocess_debug_;
};

} // namespace armor_detector
```

- [ ] **Step 2: 写 Detector.cpp**

完整替换 `src/armor_detector/src/detector/Detector.cpp`：

```cpp
#include "armor_detector/detector/Detector.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace armor_detector
{

Detector::Detector(const Params & params) : params_(params) {}

cv::Mat Detector::preprocess(const cv::Mat & img_bgr)
{
    // COLOR 阈值
    std::vector<cv::Mat> bgr;
    cv::split(img_bgr, bgr);
    cv::Mat color_ch = (params_.target_color == LightBarColor::BLUE) ? bgr[0] : bgr[2];
    cv::Mat target_color_dim_thre;
    cv::threshold(color_ch, target_color_dim_thre, params_.color_threshold, 255, cv::THRESH_BINARY);

    // GRAY 阈值
    cv::Mat gray;
    cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_thre;
    cv::threshold(gray, gray_thre, params_.gray_threshold, 255, cv::THRESH_BINARY);

    // 对每个 color_dim 区域，检查 GRAY 碎片数，构建合并二值图
    std::vector<std::vector<cv::Point>> target_color_contours;
    cv::findContours(target_color_dim_thre, target_color_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 过滤面积太小或点数不足的区域
    target_color_contours.erase(
        std::remove_if(target_color_contours.begin(), target_color_contours.end(),
            [](const auto & c) { return c.size() < 6 || cv::contourArea(c) < 30; }),
        target_color_contours.end()
    );

    // 以空白为底图，GRAY 碎片==1 用 GRAY，>=3 为噪音跳过，其余用 COLOR_dim
    cv::Mat img_thre = cv::Mat::zeros(img_bgr.size(), CV_8UC1);
    cv::Mat roi_mask;
    for (const auto & target_color_contour : target_color_contours) {
        cv::Rect color_bb = cv::boundingRect(target_color_contour);
        cv::Rect safe_bb = color_bb & cv::Rect(0, 0, gray_thre.cols, gray_thre.rows);
        cv::Mat gray_roi = gray_thre(safe_bb);

        std::vector<std::vector<cv::Point>> gray_roi_contours;
        cv::findContours(gray_roi, gray_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int valid_count = 0;
        for (const auto & c : gray_roi_contours) {
            if (cv::contourArea(c) > 5) valid_count++;
        }

        if (valid_count == 1) {
            gray_thre(safe_bb).copyTo(img_thre(safe_bb));
        } else if (valid_count >= 3) {
            // 噪音，跳过
        } else {
            // 0 or 2：用 COLOR_dim 覆盖
            roi_mask = cv::Mat::zeros(safe_bb.size(), CV_8UC1);
            std::vector<cv::Point> local_contour;
            local_contour.reserve(target_color_contour.size());
            for (const auto & pt : target_color_contour) {
                local_contour.push_back(pt - cv::Point(safe_bb.x, safe_bb.y));
            }
            cv::fillConvexPoly(roi_mask, local_contour, 255);
            target_color_dim_thre(safe_bb).copyTo(img_thre(safe_bb), roi_mask);
        }
    }

    // 闭运算
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::dilate(img_thre, img_thre, kernel);
    cv::erode(img_thre, img_thre, kernel);

    // 填充 debug 数据
    preprocess_debug_.gray = gray_thre;
    preprocess_debug_.binary = img_thre;
    preprocess_debug_.color_mask = target_color_dim_thre;

    return img_thre;
}

} // namespace armor_detector
```

- [ ] **Step 3: 编译验证**

```bash
cd /home/minzhi/Desktop/ws_homework1
colcon build --packages-select armor_detector
```

Expected: 编译成功。

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/detector/Detector.hpp \
        src/armor_detector/src/detector/Detector.cpp
git commit -m "feat: implement Detector preprocessing (dual-threshold)"
```

---

### Task 3: LightDetector 灯条检测

**Files:**
- Modify: `src/armor_detector/include/armor_detector/detector/LightDetector.hpp`
- Modify: `src/armor_detector/src/detector/LightDetector.cpp`

**Interfaces:**
- Consumes: `LightBar`, `LightBarColor` from `ArmorData.hpp`
- Consumes: `debug::LightDebugData`, `debug::RejectedLight`, `debug::DebugRejectReason` from `DebugData.hpp`
- Consumes: `armor_detector::tools::degToRad` from `tools/angle.hpp`
- Produces: `armor_detector::LightDetector` class with `findLights(binary, img_bgr) → vector<LightBar>` and `getLightDebugData() → LightDebugData`

- [ ] **Step 1: 写 LightDetector.hpp**

完整替换 `src/armor_detector/include/armor_detector/detector/LightDetector.hpp`：

```cpp
#pragma once

#include "armor_detector/types/ArmorData.hpp"
#include "armor_detector/debug/DebugData.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace armor_detector
{

class LightDetector
{
public:
    struct Params {
        int min_contours_area = 30;
        float min_contours_ratio = 0.06f;
        float max_contours_ratio = 0.5f;
    };

    explicit LightDetector(const Params & params);

    std::vector<LightBar> findLights(const cv::Mat & binary, const cv::Mat & img_bgr);

    const debug::LightDebugData & getLightDebugData() const { return light_debug_; }

private:
    bool checkLightGeometry(const std::vector<cv::Point> & contour) const;

    LightBarColor findLightColor(const cv::Mat & img_bgr,
                                  const cv::RotatedRect & rect,
                                  const std::vector<cv::Point> & contour) const;

    LightBar createLight(const cv::RotatedRect & ellipse_rect,
                          const cv::RotatedRect & min_rect,
                          LightBarColor color) const;

    Params params_;
    debug::LightDebugData light_debug_;
};

} // namespace armor_detector
```

- [ ] **Step 2: 写 LightDetector.cpp**

完整替换 `src/armor_detector/src/detector/LightDetector.cpp`：

```cpp
#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/tools/angle.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace armor_detector
{

LightDetector::LightDetector(const Params & params) : params_(params) {}

std::vector<LightBar> LightDetector::findLights(const cv::Mat & binary, const cv::Mat & img_bgr)
{
    light_debug_ = {};

    std::vector<std::vector<cv::Point>> contours;
    std::vector<LightBar> lights;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    for (const auto & contour : contours) {
        if (!checkLightGeometry(contour)) {
            // 记录被拒绝的灯条
            cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
            debug::RejectedLight rejected;
            rejected.light.rect = min_rect;
            rejected.light.center = min_rect.center;
            rejected.reason = debug::DebugRejectReason::BAD_RATIO;
            rejected.detail = "geometry";
            light_debug_.rejected_lights.push_back(std::move(rejected));
            continue;
        }

        cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
        cv::RotatedRect ellipse_rect = cv::fitEllipse(contour);
        LightBarColor color = findLightColor(img_bgr, min_rect, contour);

        if (color == LightBarColor::NONE) {
            debug::RejectedLight rejected;
            rejected.light.rect = min_rect;
            rejected.light.center = min_rect.center;
            rejected.reason = debug::DebugRejectReason::BAD_COLOR;
            rejected.detail = "none";
            light_debug_.rejected_lights.push_back(std::move(rejected));
            continue;
        }

        LightBar light = createLight(ellipse_rect, min_rect, color);
        light.id = static_cast<int>(lights.size());
        lights.push_back(light);
        light_debug_.accepted_lights.push_back(light);
    }

    // 按 x 坐标排序
    std::sort(lights.begin(), lights.end(), [](const LightBar & a, const LightBar & b) {
        return a.center.x < b.center.x;
    });

    return lights;
}

bool LightDetector::checkLightGeometry(const std::vector<cv::Point> & contour) const
{
    if (contour.size() <= 6) return false;
    int area = cv::contourArea(contour);
    if (area <= params_.min_contours_area) return false;

    cv::RotatedRect min_rect = cv::minAreaRect(cv::Mat(contour));
    double long_length = std::max(min_rect.size.width, min_rect.size.height);
    double short_length = std::min(min_rect.size.width, min_rect.size.height);
    double ratio = short_length / long_length;
    return ratio > params_.min_contours_ratio && ratio < params_.max_contours_ratio;
}

LightBarColor LightDetector::findLightColor(const cv::Mat & img_bgr,
                                             const cv::RotatedRect & rect,
                                             const std::vector<cv::Point> & contour) const
{
    cv::Rect bbox = rect.boundingRect();
    bbox &= cv::Rect(0, 0, img_bgr.cols, img_bgr.rows);
    if (bbox.empty()) return LightBarColor::NONE;

    // 生成轮廓 mask
    cv::Mat mask = cv::Mat::zeros(bbox.size(), CV_8UC1);
    std::vector<cv::Point> local_contour;
    local_contour.reserve(contour.size());
    for (const auto & pt : contour) {
        local_contour.push_back(pt - cv::Point(bbox.x, bbox.y));
    }
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{local_contour}, -1, 255, -1);
    cv::erode(mask, mask, cv::Mat());

    // 提取 ROI 并分离 B/R 通道
    cv::Mat roi = img_bgr(bbox);
    std::vector<cv::Mat> bgr;
    cv::split(roi, bgr);

    double mean_b = cv::mean(bgr[0], mask)[0];
    double mean_r = cv::mean(bgr[2], mask)[0];

    bool is_red = (mean_r / (mean_b + 1.0)) > 1.1;
    bool is_blue = (mean_b / (mean_r + 1.0)) > 1.1;
    if (is_red) return LightBarColor::RED;
    if (is_blue) return LightBarColor::BLUE;
    return LightBarColor::NONE;
}

LightBar LightDetector::createLight(const cv::RotatedRect & ellipse_rect,
                                     const cv::RotatedRect & min_rect,
                                     LightBarColor color) const
{
    LightBar light;
    light.rect = ellipse_rect;
    light.color = color;

    // 用椭圆角度计算方向，用 minAreaRect 尺寸约束端点
    light.center = min_rect.center;
    double angle_rad = tools::degToRad(ellipse_rect.angle + 90);
    cv::Point2f dir = cv::Point2f(std::cos(angle_rad), std::sin(angle_rad));

    // 确保方向一致性
    if (std::abs(dir.y) > 0.8f) {
        if (dir.y > 0) dir = -dir;
    } else if (dir.x < 0) {
        dir = -dir;
    }

    double len = cv::norm(dir);
    if (len < 1e-6) return light;
    dir = dir / len;

    double half_len = std::max(min_rect.size.width, min_rect.size.height) / 2;
    light.top = light.center + dir * half_len;
    light.bottom = light.center - dir * half_len;
    light.angle = std::atan2(dir.y, dir.x);
    light.length = half_len * 2.0;
    light.width = std::min(min_rect.size.width, min_rect.size.height);
    light.area = static_cast<int>(min_rect.size.width * min_rect.size.height);

    return light;
}

} // namespace armor_detector
```

- [ ] **Step 3: 编译验证**

```bash
cd /home/minzhi/Desktop/ws_homework1
colcon build --packages-select armor_detector
```

Expected: 编译成功。

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/detector/LightDetector.hpp \
        src/armor_detector/src/detector/LightDetector.cpp
git commit -m "feat: implement LightDetector (contour detection + geometry filter + color)"
```

---

### Task 4: DebugPreprocessView

**Files:**
- Modify: `src/armor_detector/src/debug/DebugPreprocessView.cpp`（新建，当前不存在于 CMakeLists 中但 stub 文件存在）

**Interfaces:**
- Consumes: `debug::PreprocessDebugData` from `DebugData.hpp`
- Consumes: `DebugGUI::setFrame(name, image)` from `DebugGUI.hpp`
- Produces: `DebugPreprocessView` 实现，可通过 `debug_hub_.addObserver()` 注册

**注意:** CMakeLists.txt 当前没有编译 DebugPreprocessView.cpp。需要在 Task 6 中添加。

- [ ] **Step 1: 写 DebugPreprocessView.cpp**

创建 `src/armor_detector/src/debug/DebugPreprocessView.cpp`：

```cpp
#include "armor_detector/debug/DebugPreprocessView.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugPreprocessView::DebugPreprocessView(DebugGUI & gui) : gui_(&gui) {}

void DebugPreprocessView::onPreprocess(
    const DebugFrameContext & /*context*/,
    const PreprocessDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;

    // 把三张单通道图转为 BGR 后横向拼接
    std::vector<cv::Mat> channels;
    for (const auto & src : {data.gray, data.binary, data.color_mask}) {
        if (src.empty()) continue;
        cv::Mat bgr;
        cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
        channels.push_back(bgr);
    }

    if (channels.empty()) return;

    // 横向拼接
    cv::Mat拼图;
    cv::hconcat(channels, 拼图);

    // 文字标注（遵循 DebugGUI.md: FONT_HERSHEY_SIMPLEX, scale 0.60, thickness 2）
    auto addLabel = [&](const std::string & text, int x) {
        cv::putText(拼图, text, cv::Point(x + 10, 24),
                    cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    };

    int w = channels[0].cols;
    addLabel("gray", 0);
    addLabel("binary", w);
    if (channels.size() > 2) addLabel("color_mask", w * 2);

    gui_->setFrame("preprocess_debug", 拼图);
}

} // namespace armor_detector::debug
```

注意：上面代码中的 `拼图` 是中文变量名，实际代码请用英文变量名 `combined`。

- [ ] **Step 2: 编译验证**

此步骤需要 Task 6 的 CMakeLists 修改才能编译。先跳到 Task 5，最后统一编译。

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/src/debug/DebugPreprocessView.cpp
git commit -m "feat: implement DebugPreprocessView (gray/binary/color_mask concat display)"
```

---

### Task 5: DebugLightView

**Files:**
- Create: `src/armor_detector/src/debug/DebugLightView.cpp`

**Interfaces:**
- Consumes: `debug::LightDebugData`, `debug::RejectedLight` from `DebugData.hpp`
- Consumes: `DebugFrameContext.display_bgr` (可写)
- Consumes: `DebugGUI::setFrame(name, image)` from `DebugGUI.hpp`

- [ ] **Step 1: 写 DebugLightView.cpp**

创建 `src/armor_detector/src/debug/DebugLightView.cpp`：

```cpp
#include "armor_detector/debug/DebugLightView.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugLightView::DebugLightView(DebugGUI & gui) : gui_(&gui) {}

void DebugLightView::onLights(
    const DebugFrameContext & context,
    const LightDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;
    if (context.display_bgr.empty()) return;

    cv::Mat & display = context.display_bgr;

    // 绘制 accepted lights：绿色 rotatedRect + 端点
    for (const auto & light : data.accepted_lights) {
        cv::Point2f vertices[4];
        light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
        cv::circle(display, light.top, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(display, light.bottom, 3, cv::Scalar(0, 255, 0), -1);
    }

    // 绘制 rejected lights：红色 + 原因
    for (const auto & rejected : data.rejected_lights) {
        cv::Point2f vertices[4];
        rejected.light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
        cv::putText(display, rejected.detail,
                    cv::Point(rejected.light.center.x + 5, rejected.light.center.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
}

} // namespace armor_detector::debug
```

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/src/debug/DebugLightView.cpp
git commit -m "feat: implement DebugLightView (green accepted, red rejected)"
```

---

### Task 6: DetectorNode 串联 + 参数 + config + Observer 注册 + 编译

**Files:**
- Modify: `src/armor_detector/src/DetectorNode.cpp`
- Modify: `src/armor_detector/CMakeLists.txt`
- Modify: `src/armor_detector/config/config.yaml`
- Modify: `src/armor_detector/launch/video1.launch.py`（添加参数文件）

**Interfaces:**
- Consumes: `Detector`, `LightDetector` from previous tasks
- Consumes: `DebugPreprocessView`, `DebugLightView` from previous tasks
- Consumes: `CameraProvider::getCameraInfo()` (已有)

- [ ] **Step 1: 更新 CMakeLists.txt**

在 `src/armor_detector/CMakeLists.txt` 的 `add_executable` 中添加两个新的 .cpp 文件：

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
  src/debug/DebugPreprocessView.cpp    # 新增
  src/debug/DebugLightView.cpp         # 新增
)
```

- [ ] **Step 2: 写 config/config.yaml**

完整替换 `src/armor_detector/config/config.yaml`：

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
```

- [ ] **Step 3: 更新 launch 文件添加参数**

修改 `src/armor_detector/launch/video1.launch.py`，在节点启动时加载 config.yaml：

在 launch 文件中找到 `DeclareLaunchArgument` 或节点定义处，添加 `parameters` 参数指向 config.yaml。具体写法取决于当前 launch 文件结构，核心是：

```python
import os
from ament_index_python.packages import get_package_share_directory

config = os.path.join(
    get_package_share_directory('armor_detector'),
    'config',
    'config.yaml'
)

# 在节点定义中添加
Node(
    package='armor_detector',
    executable='armor_detector_node_cpp',
    parameters=[config],
    ...
)
```

- [ ] **Step 4: 重写 DetectorNode.cpp**

完整替换 `src/armor_detector/src/DetectorNode.cpp`：

```cpp
#include "armor_detector/CameraProvider.hpp"
#include "armor_detector/detector/Detector.hpp"
#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugHub.hpp"
#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/debug/DebugPreprocessView.hpp"
#include "armor_detector/debug/DebugLightView.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>

#include <memory>
#include <string>

namespace armor_detector
{

class DetectorNode : public rclcpp::Node
{
private:
    CameraProvider camera_provider_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // 检测组件
    Detector detector_;
    LightDetector light_detector_;

    // Debug
    debug::DebugGUI debug_gui_;
    debug::DebugHub debug_hub_;
    rclcpp::TimerBase::SharedPtr debug_key_timer_;
    rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_paused_client_;
    rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr play_next_client_;
    std::string debug_gui_window_name_;
    std::size_t frame_index_ = 0;

    void run(const sensor_msgs::msg::Image::SharedPtr & msg);
    void pollDebugKeys();

public:
    DetectorNode() : Node("armor_detector_node_cpp"),
                     detector_([] {
                         // 临时默认参数，构造函数体中会被 ROS 参数覆盖
                         Detector::Params p;
                         return p;
                     }()),
                     light_detector_([] {
                         LightDetector::Params p;
                         return p;
                     }())
    {
        // 声明参数
        this->declare_parameter<bool>("debug_gui_enabled", true);
        this->declare_parameter<bool>("debug_rosbag_control_enabled", true);
        this->declare_parameter<std::string>("debug_rosbag_player_node", "/rosbag2_player");
        this->declare_parameter<std::string>("debug_gui_window_name", "debug_show");

        // 预处理参数
        this->declare_parameter<int>("gray_threshold", 100);
        this->declare_parameter<int>("color_threshold", 100);
        this->declare_parameter<std::string>("target_color", "BLUE");

        // 灯条参数
        this->declare_parameter<int>("min_contours_area", 30);
        this->declare_parameter<double>("min_contours_ratio", 0.06);
        this->declare_parameter<double>("max_contours_ratio", 0.5);

        // 读取参数
        bool debug_gui_enabled = this->get_parameter("debug_gui_enabled").as_bool();
        bool rosbag_control_enabled = this->get_parameter("debug_rosbag_control_enabled").as_bool();
        std::string rosbag_player_node = this->get_parameter("debug_rosbag_player_node").as_string();
        debug_gui_window_name_ = this->get_parameter("debug_gui_window_name").as_string();

        // 用 ROS 参数重建检测器
        std::string target_color_str = this->get_parameter("target_color").as_string();
        LightBarColor target_color = (target_color_str == "RED") ? LightBarColor::RED : LightBarColor::BLUE;

        Detector::Params det_params;
        det_params.gray_threshold = this->get_parameter("gray_threshold").as_int();
        det_params.color_threshold = this->get_parameter("color_threshold").as_int();
        det_params.target_color = target_color;
        detector_ = Detector(det_params);

        LightDetector::Params light_params;
        light_params.min_contours_area = this->get_parameter("min_contours_area").as_int();
        light_params.min_contours_ratio = static_cast<float>(this->get_parameter("min_contours_ratio").as_double());
        light_params.max_contours_ratio = static_cast<float>(this->get_parameter("max_contours_ratio").as_double());
        light_detector_ = LightDetector(light_params);

        camera_provider_.init();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); }
        );

        // Debug GUI
        debug_gui_.setEnabled(debug_gui_enabled);
        debug_gui_.start();

        // Debug Observer 注册
        debug_hub_.addObserver(std::make_shared<debug::DebugPreprocessView>(debug_gui_));
        debug_hub_.addObserver(std::make_shared<debug::DebugLightView>(debug_gui_));

        // Rosbag service clients
        if (rosbag_control_enabled) {
            toggle_paused_client_ = this->create_client<rosbag2_interfaces::srv::TogglePaused>(
                rosbag_player_node + "/toggle_paused");
            play_next_client_ = this->create_client<rosbag2_interfaces::srv::PlayNext>(
                rosbag_player_node + "/play_next");
        }

        // 按键轮询 timer
        debug_key_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(15),
            [this]() { pollDebugKeys(); }
        );

        RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
        RCLCPP_INFO(this->get_logger(), "按键操作: [q/ESC]退出  [Space/p]暂停/继续  [n/→]单步  [s]保存ROI");
    }

    ~DetectorNode() override
    {
        debug_gui_.stop();
    }
};

void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);

    // 构建 debug 上下文
    debug::DebugFrameContext ctx;
    ctx.frame_index = frame_index_++;
    ctx.stamp = frame.timestamp;
    ctx.source_bgr = frame.image;
    ctx.display_bgr = frame.image.clone();

    // 预处理
    cv::Mat binary = detector_.preprocess(frame.image);
    debug_hub_.onPreprocess(ctx, detector_.getPreprocessDebugData());

    // 灯条检测
    auto lights = light_detector_.findLights(binary, frame.image);
    debug_hub_.onLights(ctx, light_detector_.getLightDebugData());

    // 一次性提交最终显示图像
    debug_gui_.setFrame(debug_gui_window_name_, ctx.display_bgr);
}

void DetectorNode::pollDebugKeys()
{
    auto events = debug_gui_.takeKeyEvents();
    for (const auto & event : events) {
        switch (event.action) {
        case debug::DebugKeyAction::EXIT:
            debug_gui_.stop();
            rclcpp::shutdown();
            break;

        case debug::DebugKeyAction::PAUSE_TOGGLE:
            if (!toggle_paused_client_ || !toggle_paused_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag toggle_paused service not ready");
                continue;
            }
            (void)toggle_paused_client_->async_send_request(
                std::make_shared<rosbag2_interfaces::srv::TogglePaused::Request>());
            break;

        case debug::DebugKeyAction::STEP_FRAME:
            if (!play_next_client_ || !play_next_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag play_next service not ready");
                continue;
            }
            (void)play_next_client_->async_send_request(
                std::make_shared<rosbag2_interfaces::srv::PlayNext::Request>());
            break;

        case debug::DebugKeyAction::SAVE_ROI:
            debug_hub_.onKey(event);
            break;

        default:
            break;
        }
    }
}

} // namespace armor_detector

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 5: 编译验证**

```bash
cd /home/minzhi/Desktop/ws_homework1
colcon build --packages-select armor_detector
```

Expected: 编译成功。

- [ ] **Step 6: 运行验证**

```bash
source install/setup.bash
ros2 launch armor_detector video1.launch.py
```

Expected:
- 预处理窗口 `preprocess_debug` 显示 gray/binary/color_mask 拼图
- 主窗口 `debug_show` 显示原始图像 + 绿色灯条框（accepted）/ 红色灯条框（rejected）
- ESC 退出正常

- [ ] **Step 7: Commit**

```bash
git add src/armor_detector/src/DetectorNode.cpp \
        src/armor_detector/CMakeLists.txt \
        src/armor_detector/config/config.yaml \
        src/armor_detector/launch/video1.launch.py
git commit -m "feat: wire light detection pipeline with debug observers and ROS2 params"
```

---

## 验证清单

完成所有 Task 后，运行以下验证：

1. **编译**: `colcon build --packages-select armor_detector` — 无 warning 无 error
2. **运行**: `ros2 launch armor_detector video1.launch.py` — 两个窗口正常显示
3. **预处理窗口**: gray/binary/color_mask 三张图横向拼接，有文字标注
4. **主窗口**: 原始图像上叠加绿色灯条旋转矩形和端点
5. **按键**: ESC 正常退出，Space 暂停/继续 rosbag
6. **参数**: 修改 config.yaml 中 `target_color: "RED"` 重新运行，检测红色灯条
