# Debug 扩展规范

本文档只记录 `armor_detector` 包内 debug 功能的扩展方式和代码规范。单位、坐标系、角点顺序、`cv::Mat` 生命周期等全局硬约定见 `Conventions.md`。

## 总体原则

- 主流程只负责算法顺序，不直接写窗口显示、ROI 保存、耗时统计等 debug 细节。
- Debug 功能通过 `IDebugObserver` 扩展，由 `DebugHub` 统一注册和分发事件。
- 所有 OpenCV 窗口、`imshow`、`waitKey`、`destroyAllWindows` 只能放在 `DebugGUI` 或其实现中。
- Observer 只读主流程数据，不反向修改算法结果。
- 新增 debug 功能优先新增一个 `DebugXXX` Observer，不要把功能塞回 `DetectorNode` 主流程。
- Debug 数据中的单位、坐标和 `cv::Mat` 生命周期必须遵守 `Conventions.md`。

## 架构概述

系统支持两种检测后端：`traditional`（传统轮廓检测）和 `yolo`（YOLO/RobotDetectionModel）。`DetectorNode::run()` 调用 `detector_->detect()` 获取统一的 `DetectionResult`，然后通过 `debug_hub_.onDetection(ctx, result.debug)` 分发给所有 observer。

每个 observer 根据 `data.backend` 判断是否处理当前后端的数据。

## 文件命名

Debug 相关文件放在：

```text
include/armor_detector/debug/
src/debug/
```

当前已有文件：

```text
DebugData.hpp                   阶段 debug 数据结构
DetectionDebugData.hpp          统一检测 debug 数据（传统 + YOLO）
IDebugObserver.hpp              Observer 接口
DebugHub.hpp                    事件分发器
DebugGUI.hpp                    GUI 统一出口
DebugKeyHandler.hpp             按键转换
DebugLayerState.hpp             图层开关状态（线程安全）
DebugLayerController.hpp        数字键切换图层
DebugTiming.hpp                 耗时统计（无头模式也打印日志）
DebugPoseMarkerPublisher.hpp    /armor_markers MarkerArray 发布
DebugPreprocess.hpp             预处理图层（仅 traditional）
DebugLight.hpp                  灯条图层（仅 traditional）
DebugArmorMatch.hpp             装甲板匹配图层（仅 traditional）
DebugClassification.hpp         数字分类图层（仅 traditional）
DebugYolo.hpp                   YOLO 检测图层（仅 yolo）
DebugPose.hpp                   位姿/坐标图层
DebugResult.hpp                 最终结果图层
DebugRoiRecorder.hpp            ROI 录制
```

新增功能使用 `Debug + 功能名`，例如：

```text
DebugPnpError.hpp
DebugRejectedArmor.hpp
```

## 新增 Observer 的步骤

1. 判断这个功能关心哪个阶段事件。
2. 若现有事件数据不够，在 `DetectionDebugData.hpp` 中补充对应阶段的数据结构。
3. 新增 `DebugXXX` 类，继承 `IDebugObserver`。
4. 只重写自己关心的事件，其他事件保持默认空实现。
5. 如果需要显示图像，把图像提交给 `DebugGUI`，不要直接调用 `cv::imshow`。
6. 如果需要发布 ROS topic（如 MarkerArray），在构造时创建 publisher。
7. 如果需要响应按键，实现 `onKey(const DebugKeyEvent & event)`。
8. 在 `DetectorNode::initDebug()` 中注册到 `DebugHub`。

## Observer 分类

### 非 GUI Observer（始终注册）

不受 `debug.show` 控制，即使无头模式也工作：

- **DebugTiming** — 阶段耗时统计，滑动窗口每 `debug.stats_interval` 帧打印平均用时日志
- **DebugPoseMarkerPublisher** — `/armor_markers` MarkerArray 发布，受 `debug.pose` 图层控制

### GUI Observer（仅 debug.show=true 时注册）

需要 OpenCV 窗口，无头模式下不注册：

- **Traditional 后端专用**：DebugPreprocessView, DebugLightView, DebugArmorMatchView, DebugClassificationView
- **YOLO 后端专用**：DebugYoloView
- **通用**：DebugResultView（两种后端都用）
- DebugLayerController（数字键切换）

注册逻辑：

```cpp
void DetectorNode::initDebug() {
    // 非 GUI observer：始终注册
    debug_hub_.addObserver(std::make_shared<debug::DebugTiming>(debug_config_.stats_interval));
    debug_hub_.addObserver(std::make_shared<debug::DebugPoseMarkerPublisher>(*this, layer_state_));

    // GUI observer：仅 debug.show=true 时注册
    if (!debug_config_.show) return;
    // ... 注册 GUI observers（传统 + YOLO + 通用）
}
```

## 事件使用规范

当前保留这些阶段事件：

```text
onFrameStart       帧开始（设置 frame context）
onDetection        统一检测结果（替代旧的 onPreprocess/onLights/onArmorMatch/onClassification）
onPoseSolved       位姿解算完成
onFrameEnd         帧结束（绘制帧耗时、刷新 GUI）
onKey              按键事件
```

`onDetection` 是主要的检测阶段事件，接收 `DetectionDebugData`，包含：
- `backend`：当前后端类型
- `final_armors`：最终检测结果
- `timings`：各阶段耗时
- `traditional`：传统后端专用 debug 数据
- `yolo`：YOLO 后端专用 debug 数据

事件粒度保持"阶段级"，不要在像素循环、轮廓循环等高频内层逻辑里发事件。

## 开关规范

Debug 开关控制 Observer 是否注册，而不是在主流程中堆 `if debug_xxx`。

配置结构：

```yaml
debug:
  show: true              # OpenCV GUI 开关（非 GUI observer 不受此控制）
  rosbag_control: true
  rosbag_player_node: "/rosbag2_player"
  detect_stage_1: false   # traditional: 预处理; yolo: letterbox 输入
  detect_stage_2: false   # traditional: 灯条; yolo: score 阈值候选
  detect_stage_3: false   # traditional: 装甲板匹配; yolo: NMS/过滤结果
  detect_stage_4: false   # traditional: 数字分类; yolo: 最终检测
  pose: false             # 控制 /armor_markers 发布
  result: true            # 最终结果显示
  stats_interval: 50      # 耗时统计打印间隔（帧数）

playback:
  mode: "realtime"        # "realtime" 或 "step"
  max_frames: 0           # 0=不限制，>0 时处理指定帧数后退出
  exit_on_complete: false
```

## 按键规范

`cv::waitKey` 的原始值由 `DebugKeyHandler` 转换为 `DebugKeyEvent`。

当前保留动作：

```text
EXIT          退出节点 / 关闭窗口
PAUSE_TOGGLE  rosbag 暂停/继续
STEP_FRAME    rosbag 单步
SAVE_ROI      保存 ROI（预留）
TOGGLE_LAYER  切换图层显示（数字键 1-6）
```

## 图层切换

数字键 1–6 切换对应 debug 图层的显示状态：

| 按键 | 图层 | Traditional 含义 | YOLO 含义 |
|---|---|---|---|
| 1 | DETECT_STAGE_1 | 预处理中间图 | letterbox 输入图 |
| 2 | DETECT_STAGE_2 | 灯条检测结果 | score 阈值后的候选框 |
| 3 | DETECT_STAGE_3 | 装甲板匹配 | NMS/类别/颜色过滤结果 |
| 4 | DETECT_STAGE_4 | 数字分类 | 最终检测结果 |
| 5 | POSE | /armor_markers 发布开关 | 同左 |
| 6 | RESULT | 最终识别装甲板 X 标记 | 同左 |

切换后输出日志，例如：`DEBUG layer DETECT_STAGE_1: ON`

## DebugYoloView 显示内容

YOLO 后端的 `DebugYoloView` 支持 4 个阶段：

| 阶段 | 窗口 | 内容 | 颜色 |
|---|---|---|---|
| Stage 1 | yolo_stage1_letterbox | source_roi + letterbox 拼接对比图 | — |
| Stage 2 | debug_show | score 阈值后的候选框 | 黄色 (0,255,255) |
| Stage 3 | debug_show | NMS kept + NMS rejected + filter rejected | 绿/灰/红/蓝/橙 |
| Stage 4 | debug_show | 最终检测结果框 + 名称/置信度 | 紫色 (255,0,255) |

拒绝原因颜色：
- 灰色 (128,128,128)：NMS 抑制
- 红色 (0,0,255)：不支持的类别
- 蓝色 (255,0,0)：颜色不匹配
- 橙色 (0,165,255)：低置信度

## Timing 行为

- `timing` 不是图层——始终注册，无法通过数字键关闭。
- 使用滑动窗口统计：每 `debug.stats_interval` 帧打印一次，然后清零重算。
- 检测耗时（`onDetection`）和帧耗时（`onFrameEnd`）分别统计，互不干扰。
- 日志格式：`[yolo] 最近50帧: [total:xxxms] [detect:xxxms [decode:xxxms, ...], pose:xxxms]`
- 仅当 `debug.show=true` 且 `display_bgr` 非空时，才在 GUI 上绘制帧耗时。
- `auto_test.launch.py` 支持 `timing_interval` 参数覆盖 `debug.stats_interval`。

## 播放控制

### realtime 模式

- rosbag 按时间正常播放
- 按键 Space 暂停/继续，n 单步

### step 模式

- 处理完一帧后再请求播放下一帧
- 通过 `/rosbag2_player/play_next` service 控制
- 使用 in-flight 状态机避免请求竞争
- `auto_test.launch.py` 默认使用此模式

## 数据结构规范

- 阶段 debug 数据统一放在 `DetectionDebugData.hpp`。
- 数据结构只表达"该阶段产生了什么"，不要混入窗口布局细节。
- 拒绝原因使用枚举 `DebugRejectReason`，补充细节放到 `detail` 字符串。
- 如果字段会影响算法结果，它不应该放在 debug 数据里。
