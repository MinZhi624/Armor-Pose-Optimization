# Light 检测移植设计

## 概述

从参考项目 `Visual-Translationo` 移植灯条检测（Light Detection）算法到当前框架 `ws_homework1`。包含预处理和灯条检测两个阶段，附带 debug 可视化验证。

## 范围

只做 Light 部分，不涉及 Armor 匹配、NumberClassifier、PoseSolver。

## 已确认决策

| 决策 | 结论 |
|---|---|
| 类结构 | 按框架拆分：Detector（预处理）+ LightDetector（灯条） |
| 数据类型 | 用框架已有的 LightBar（纯数据载体），不加方法 |
| 参数管理 | ROS2 参数系统，DetectorNode 构造函数中 declare + get |
| 推理后端 | 本次不涉及（NumberClassifier 后续再做） |
| 算法逻辑 | 完全不修改，原样移植 |
| 角度单位 | 弧度（和参考项目一致） |
| Debug 窗口 | 单窗口多图层叠加（遵守 DebugGUI.md） |
| Debug 绘制 | 遵守 DebugGUI.md 颜色/文字/图层规范 |

## 工具库

已创建 `include/armor_detector/tools/angle.hpp`，移植自参考项目 `armor_plate_common/angle.hpp`：

- `normalizeRadAngle(rad)` — 归一化到 [-π, π]
- `shortestAngularDistance(from, to)` — 最短角距离
- `degToRad(deg)` / `radToDeg(rad)` — 单位转换
- namespace: `armor_detector::tools`

## 实现步骤

### 步骤 1：LightBar 数据结构

修改 `include/armor_detector/types/ArmorData.hpp`：

- `angle_deg` 改名为 `angle`，单位改为弧度
- 保持纯数据 struct，不加构造函数和方法
- `LightBarColor` 枚举不变（RED, BLUE, NONE，和参考项目的 Color 一致）

**与 Conventions.md 的偏离说明**：Conventions.md 规定"图像平面几何角使用 deg"，但本项目选择弧度以匹配参考项目内部约定。LightBar.angle 存储弧度，后续如果需要显示，用 `tools::radToDeg()` 转换。

### 步骤 2：Detector 预处理

新建/实现 `include/armor_detector/detector/Detector.hpp` + `src/detector/Detector.cpp`：

**类设计：**
```cpp
namespace armor_detector {

class Detector {
public:
    struct Params {
        int gray_threshold = 100;
        int color_threshold = 100;
        LightBarColor target_color = LightBarColor::BLUE;
    };

    explicit Detector(const Params& params);

    // 预处理：双阈值 + 碎片合并 + 闭运算
    // 返回合并后的二值图
    cv::Mat preprocess(const cv::Mat& img_bgr);

    // 获取预处理 debug 数据（调用 preprocess 后有效）
    const debug::PreprocessDebugData& getPreprocessDebugData() const;

private:
    Params params_;
    debug::PreprocessDebugData preprocess_debug_;
};

} // namespace armor_detector
```

**算法逻辑**（原样移植自参考项目 `Detector::preprocess()`）：
1. 分离 BGR 通道，取目标颜色通道（Blue→B, Red→R），阈值化得 `target_color_dim_thre`
2. 灰度化，阈值化得 `gray_thre`
3. 对每个 color 区域，统计 GRAY 碎片数：
   - 1 个碎片 → 用 GRAY 阈值（更精确）
   - 0 或 2 个 → 用 COLOR 阈值（fillConvexPoly 覆盖）
   - 3+ 个 → 跳过（噪音）
4. 闭运算（dilate + erode，3×5 核）
5. 填充 `PreprocessDebugData`（gray, binary, color_mask）

**字段映射**：参考项目 `PreprocessDebug` → 框架 `PreprocessDebugData`：
- `gray_thre` → `gray`
- `merged_thre` → `binary`
- `target_color_dim_thre` → `color_mask`

### 步骤 3：DebugPreprocessView

实现 `src/debug/DebugPreprocessView.cpp`：

**遵守 DebugGUI.md 规范**：
- 单窗口 `debug_show`，图层名 `preprocess`
- 文字样式统一：FONT_HERSHEY_SIMPLEX, scale 0.60, thickness 2, LINE_AA
- 预处理中间图（gray/binary/color_mask）拼成一张横向拼图，通过 `gui_->setFrame("preprocess_debug", 拼图)` 显示
- 不在 Observer 中调用 `cv::imshow`

```cpp
DebugPreprocessView::DebugPreprocessView(DebugGUI& gui) : gui_(&gui) {}

void DebugPreprocessView::onPreprocess(
    const DebugFrameContext& context,
    const PreprocessDebugData& data)
{
    // 把 gray / binary / color_mask 三张单通道图转为 BGR 后横向拼接
    // 在拼图上方添加文字标注（遵循 DebugGUI.md 文字规范）
    // 提交给 DebugGUI 显示
    cv::Mat拼图 = ...;
    gui_->setFrame("preprocess_debug", 拼图);
}
```

### 步骤 4：LightDetector 灯条检测

新建/实现 `include/armor_detector/detector/LightDetector.hpp` + `src/detector/LightDetector.cpp`：

**类设计：**
```cpp
namespace armor_detector {

class LightDetector {
public:
    struct Params {
        int min_contours_area = 30;
        float min_contours_ratio = 0.06f;  // 短边/长边
        float max_contours_ratio = 0.5f;
    };

    explicit LightDetector(const Params& params);

    // 灯条检测主流程
    std::vector<LightBar> findLights(const cv::Mat& binary, const cv::Mat& img_bgr);

    // 获取 debug 数据
    const debug::LightDebugData& getLightDebugData() const;

private:
    // 单个灯条几何校验
    bool checkLightGeometry(const std::vector<cv::Point>& contour) const;

    // 颜色判定（原 getLightColor）
    LightBarColor findLightColor(const cv::Mat& img_bgr,
                                  const cv::RotatedRect& rect,
                                  const std::vector<cv::Point>& contour) const;

    // 构造 LightBar（原 Light 构造函数）
    LightBar createLight(const cv::RotatedRect& ellipse_rect,
                          const cv::RotatedRect& min_rect,
                          LightBarColor color) const;

    Params params_;
    debug::LightDebugData light_debug_;
};

} // namespace armor_detector
```

**算法逻辑**（原样移植）：

`findLights()`：
1. `cv::findContours` (RETR_EXTERNAL, CHAIN_APPROX_NONE)
2. 对每个轮廓调用 `checkLightGeometry()`
3. 通过的轮廓：`fitEllipse` + `minAreaRect` + `findLightColor()`
4. 调用 `createLight()` 构造 LightBar
5. 按 center.x 排序

`checkLightGeometry()`：
- 轮廓点数 > 6
- 面积 > min_contours_area
- 短边/长边比在 [min_contours_ratio, max_contours_ratio] 范围内

`findLightColor()`：
- 在轮廓 mask 内计算 B/R 通道均值
- `mean_r / (mean_b + 1.0) > 1.1` → RED
- `mean_b / (mean_r + 1.0) > 1.1` → BLUE
- 否则 → NONE

`createLight()`：
- 用椭圆角度计算方向向量
- 确保方向一致性（y 分量向下或 x 分量向右）
- 用 minAreaRect 的长边计算 top/bottom 端点
- angle = atan2(dir.y, dir.x)（弧度）
- length = 长边, width = 短边, area = 长×宽

### 步骤 5：DebugLightView

实现 `src/debug/DebugLightView.cpp`：

**遵守 DebugGUI.md 规范**：
- 图层 `lights`，叠加到 `debug_show` 主窗口
- accepted lights：绿色 `(0, 255, 0)` rotatedRect，thickness=2
- rejected lights：红色 `(0, 0, 255)` rotatedRect + 拒绝原因文字
- 文字样式统一：FONT_HERSHEY_SIMPLEX, scale 0.60, thickness 2
- 在 `context.display_bgr` 上直接绘制（叠加图层），不 clone 不 setFrame

```cpp
DebugLightView::DebugLightView(DebugGUI& gui) : gui_(&gui) {}

void DebugLightView::onLights(
    const DebugFrameContext& context,
    const LightDebugData& data)
{
    if (context.display_bgr.empty()) return;
    cv::Mat& display = context.display_bgr;

    // 绘制 accepted lights：绿色 rotatedRect + 端点
    for (const auto& light : data.accepted_lights) {
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
    for (const auto& rejected : data.rejected_lights) {
        cv::Point2f vertices[4];
        rejected.light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }
        // 拒绝原因文字（在灯条旁显示）
        cv::putText(display, rejected.detail,
                    rejected.light.center, cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
}
```

### 步骤 6：DebugFrameContext 支持图层叠加

修改 `include/armor_detector/debug/DebugData.hpp`，在 `DebugFrameContext` 中添加：

```cpp
struct DebugFrameContext
{
    std::size_t frame_index = 0;
    builtin_interfaces::msg::Time stamp;
    cv::Mat source_bgr;      // 原始图像（只读）
    cv::Mat display_bgr;     // 显示用图像（Observer 可在其上绘制叠加图层）
};
```

`display_bgr` 由 `DetectorNode::run()` 创建（`source_bgr.clone()`），各 Observer 按顺序在其上绘制，最终由 `run()` 调用 `debug_gui_.setFrame()` 一次性提交。

### 步骤 7：DetectorNode 串联

修改 `src/DetectorNode.cpp`：

```cpp
// 新增成员
Detector detector_;
LightDetector light_detector_;
std::size_t frame_index_ = 0;

// run() 中串联
void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr& msg) {
    Frame frame = camera_provider_.receiveImage(msg);

    // 构建 debug 上下文（display_bgr 用于图层叠加）
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
```

### 步骤 8：参数接入

在 DetectorNode 构造函数中声明参数：

```cpp
// 预处理参数
this->declare_parameter<int>("gray_threshold", 100);
this->declare_parameter<int>("color_threshold", 100);
this->declare_parameter<std::string>("target_color", "BLUE");

// 灯条参数
this->declare_parameter<int>("min_contours_area", 30);
this->declare_parameter<double>("min_contours_ratio", 0.06);
this->declare_parameter<double>("max_contours_ratio", 0.5);
```

从参数构造 Detector::Params 和 LightDetector::Params。

### 步骤 9：config.yaml

写入 `config/config.yaml`：

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

### 步骤 10：注册 Debug Observer

在 DetectorNode 构造函数中注册 Observer：

```cpp
// Debug Observer 注册（按 Debug.md 开关规范）
debug_hub_.addObserver(std::make_shared<debug::DebugPreprocessView>(debug_gui_));
debug_hub_.addObserver(std::make_shared<debug::DebugLightView>(debug_gui_));
```

## 依赖关系

```
步骤 1 (LightBar 数据结构)
  ↓
步骤 2 (Detector 预处理) ← 步骤 1
  ↓
步骤 3 (DebugPreprocessView) ← 步骤 2
  ↓
步骤 4 (LightDetector 灯条检测) ← 步骤 1
  ↓
步骤 5 (DebugLightView) ← 步骤 4
  ↓
步骤 6 (DebugFrameContext 支持图层叠加) ← 步骤 3, 5
  ↓
步骤 7 (DetectorNode 串联) ← 步骤 2, 3, 4, 5, 6
  ↓
步骤 8 (参数接入) ← 步骤 7
  ↓
步骤 9 (config.yaml) ← 步骤 8
  ↓
步骤 10 (注册 Observer) ← 步骤 7
```

## 角度单位说明

- 参考项目内部全用弧度
- 框架 `LightBar.angle` 改为弧度（偏离 Conventions.md 的"图像几何角用 deg"约定，以匹配参考项目）
- 显示时用 `tools::radToDeg()` 转换
- `DetectorArmor` 中的 `angle_diff` 在参考项目中是度（`radToDeg` 后的结果），后续移植 Armor 时注意
- LightDetector 内部计算全部使用弧度，和参考项目一致

## Debug 绘制规范（来自 DebugGUI.md）

- 默认窗口 `debug_show`，各模块作为图层叠加
- 文字：FONT_HERSHEY_SIMPLEX, scale 0.60, thickness 2, LINE_AA, 行高 24px
- 颜色 BGR，同语义同色，不同语义不同色
- 灯条 accepted: `(0, 255, 0)` 绿色, rejected: `(0, 0, 255)` 红色
- Observer 不调用 `cv::imshow`，通过 `DebugGUI::setFrame()` 提交

## 不在本次范围

- ArmorDetector（匹配 + 去重）
- NumberClassifier（CNN 推理）
- PoseSolver（PnP 位姿求解）
- ROS2 发布检测结果
