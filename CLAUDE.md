# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
colcon build --packages-select armor_detector
source install/setup.bash

# 交互模式（GUI + Foxglove）
ros2 launch armor_detector video1.launch.py

# 自动测试（无头 step 模式）
ros2 launch armor_detector auto_test.launch.py frame_count:=150 timing_interval:=1
```

ROS2 bag 测试数据在 `src/armor_detector/Test/video/video1` ~ `video5`。

## Code Style

项目使用 LLVM 风格（非 Google），配置在根目录 `.clang-format`：
- 缩进 4 格，行宽 120
- `else`/`catch` 前换行
- namespace 内缩进

静态分析用 `.clang-tidy`，启用了 `bugprone-*`、`modernize-*`、`performance-*`、`readability-*` 等检查。

## Architecture

单包 `armor_detector`，namespace `armor_detector`，用于 RoboMaster 装甲板检测。

### 数据流

```
rosbag → /image_raw → DetectorNode (订阅)
                         ↓
                    CameraProvider.receiveImage(msg)
                         ↓ Frame (cv::Mat + timestamp)
                    Detector.preprocess()
                         ↓ img_thre
                    LightDetector.findLights()
                         ↓ vector<LightBar>
                    ArmorDetector.match()
                         ↓ vector<ArmorCandidate>
                    NumberClassifier.classify()
                         ↓ vector<ClassifiedArmor>
                    PoseSolver.solve()
                         ↓ vector<SolvedArmor>
                    DebugHub.onPoseSolved()
```

### 核心组件

- **DetectorNode** — ROS2 Node，订阅 `/image_raw`，执行完整检测流水线。支持 realtime 和 step 两种播放模式。
- **CameraProvider** — 从 yaml 加载相机内参，`receiveImage(msg)` 用 `cv_bridge::toCvShare` 零拷贝转换为 `Frame`，强制输出 BGR8。
- **Detector** — 图像预处理（灰度 + 颜色阈值），输出二值图。
- **LightDetector** — 从二值图检测灯条，输出 `LightBar` 列表。
- **ArmorDetector** — 灯条配对匹配装甲板，输出 `ArmorCandidate` 列表。
- **NumberClassifier** — ONNX CNN 数字分类，输出 `ClassifiedArmor` 列表。
- **PoseSolver** — PnP 位姿解算（IPPE + iterative fallback），几何选择 + yaw 连续性修正 + yaw 重投影优化。输出 `SolvedArmor`。
- **YawSearch** — 两阶段 yaw 搜索算法（枚举粗搜 + 三分精搜），最小化重投影误差。

### 坐标系

- **Camera**: X右 Y下 Z前
- **Gimbal**: X前 Y左 Z上
- 转换矩阵 `R_GIMBAL_CAMERA` 定义在 `tools/transform.hpp`
- 装甲板固定 pitch=15°，roll=0°

### 工具模块 (tools/)

- `angle.hpp` — 角度归一化、度弧转换
- `transform.hpp` — R_GIMBAL_CAMERA、R_CAMERA_GIMBAL、calculateRWorldGimbal
- `geometry.hpp` — calculateYPR、calculateYPD、calculateXYZ
- `armor_geometry.hpp` — 装甲板尺寸常量、3D 角点

### Debug 系统 (Observer Pattern)

- **IDebugObserver** — 基接口，定义各阶段虚方法
- **DebugHub** — 事件分发器，维护 observer 列表并转发事件
- **DebugGUI** — OpenCV 窗口统一管理
- **DebugTiming** — 阶段耗时统计（无头模式也打印日志）
- **DebugPoseMarkerPublisher** — `/armor_markers` MarkerArray 发布（受 debug.pose 图层控制）
- **DebugLayerController** — 数字键切换图层
- 其他 view observer: DebugLightView, DebugArmorMatchView, DebugClassificationView, DebugResultView, DebugPreprocessView

### 播放模式

- **realtime** — rosbag 按时间正常播放（video1.launch.py 默认）
- **step** — 处理完一帧后再请求播放下一帧（auto_test.launch.py 默认）

参数：`playback.mode`、`playback.max_frames`、`playback.exit_on_complete`

## Dependencies

ROS2: `rclcpp`, `sensor_msgs`, `cv_bridge`, `ament_index_cpp`, `visualization_msgs`, `rosbag2_interfaces`
第三方: OpenCV, yaml-cpp, Eigen3
可选: foxglove_bridge

## 参考实现

源项目位于 `/home/minzhi/Desktop/Visual-Translationo`，有实现时先参考那里。

## 文档

- `docs/Conventions.md` — 单位、坐标系、cv::Mat 生命周期规则
- `docs/DebugGUI.md` — GUI 注册和显示标准（含 /armor_markers 文档）
- `docs/Debug.md` — Debug 系统扩展指南

## QoS 注意事项

订阅 `/image_raw` 用 `rclcpp::QoS(10)`（Reliable），不要用 `SensorDataQoS`（BestEffort），否则和 rosbag 发布的 QoS 不匹配会丢帧。
