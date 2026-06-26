# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
source /home/minzhi/intel/openvino_2026.1.0/setupvars.sh  # YOLO 后端需要
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

### 双后端架构

系统通过 `IArmorDetectorBackend` 接口支持两种检测后端，在 `config.yaml` 中通过 `detector.backend` 选择：

| 后端 | 说明 | 关键依赖 |
|---|---|---|
| `yolo`（默认） | RobotDetectionModel 端到端检测 | OpenVINO |
| `traditional` | 轮廓→灯条→装甲板匹配→CNN 分类 | OpenCV |

### 数据流

```
rosbag → /image_raw → DetectorNode (订阅)
                         ↓
                    CameraProvider.receiveImage(msg)
                         ↓ Frame (cv::Mat + timestamp)
                    backend_->detect(img)  ← IArmorDetectorBackend 接口
                         ↓ DetectionResult (armors + debug + timings)
                    PoseSolver.solve()
                         ↓ vector<SolvedArmor>
```

Traditional 后端内部流水线：
```
Detector.preprocess() → LightDetector.findLights() → ArmorDetector.match() → NumberClassifier.classify()
```

YOLO 后端内部流水线：
```
Letterbox → OpenVINO 推理 → 解码(22列) → NMS → 类别/颜色过滤
```

### 核心组件

- **DetectorNode** — ROS2 Node，订阅 `/image_raw`，通过 `backend_->detect()` 执行检测。支持 realtime 和 step 两种播放模式。
- **IArmorDetectorBackend** — 检测后端接口，统一输出 `DetectionResult`（`ClassifiedArmor` + `DetectionDebugData` + timings）。
- **TraditionalArmorDetectorBackend** — 传统后端实现，包住 Detector→LightDetector→ArmorDetector→NumberClassifier。
- **YoloArmorDetectorBackend** — YOLO 后端实现，OpenVINO 推理 + 22 列输出解析 + NMS。
- **CameraProvider** — 从 yaml 加载相机内参，`receiveImage(msg)` 用 `cv_bridge::toCvShare` 零拷贝转换为 `Frame`，强制输出 BGR8。
- **PoseSolver** — PnP 位姿解算（IPPE + iterative fallback），几何选择 + yaw 连续性修正 + yaw 重投影优化。输出 `SolvedArmor`。
- **YawSearch** — 两阶段 yaw 搜索算法（枚举粗搜 + 三分精搜），最小化重投影误差。

### 配置结构

```yaml
detector:
  backend: "yolo"     # yolo / traditional
  target_color: "RED"
  traditional: { ... }  # 传统后端参数
  yolo: { ... }       # YOLO 后端参数

debug:
  show: true
  detect_stage_1: false  # traditional: 预处理; yolo: letterbox
  detect_stage_2: false  # traditional: 灯条; yolo: score 候选
  detect_stage_3: false  # traditional: 装甲板匹配; yolo: NMS/过滤
  detect_stage_4: false  # traditional: 数字分类; yolo: 最终检测
  pose: false
  result: true
  stats_interval: 50
```

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

- **IDebugObserver** — 基接口，核心事件：`onDetection`（统一检测结果）
- **DebugHub** — 事件分发器，维护 observer 列表并转发事件
- **DebugGUI** — OpenCV 窗口统一管理
- **DebugTiming** — 滑动窗口耗时统计（无头模式也打印日志）
- **DebugPoseMarkerPublisher** — `/armor_markers` MarkerArray 发布（受 debug.pose 图层控制）
- **DebugLayerController** — 数字键切换图层
- Traditional 专用: DebugPreprocessView, DebugLightView, DebugArmorMatchView, DebugClassificationView
- YOLO 专用: DebugYoloView（letterbox/候选框/NMS/最终结果）
- 通用: DebugResultView

### 播放模式

- **realtime** — rosbag 按时间正常播放（video1.launch.py 默认）
- **step** — 处理完一帧后再请求播放下一帧（auto_test.launch.py 默认）

参数：`playback.mode`、`playback.max_frames`、`playback.exit_on_complete`

## Dependencies

ROS2: `rclcpp`, `sensor_msgs`, `cv_bridge`, `ament_index_cpp`, `visualization_msgs`, `rosbag2_interfaces`
第三方: OpenCV, OpenVINO, yaml-cpp, Eigen3
可选: foxglove_bridge

## 参考实现

传统检测参考：`/home/minzhi/Desktop/Visual-Translationo/src/armor_plate_identification`
YOLO 参考：`/home/minzhi/Desktop/参考/sp_vision_25/tasks/auto_aim/yolos/`

## 文档

- `docs/Conventions.md` — 单位、坐标系、cv::Mat 生命周期规则
- `docs/DebugGUI.md` — GUI 注册和显示标准（含 /armor_markers 文档）
- `docs/Debug.md` — Debug 系统扩展指南
- `docs/AlgorithmReference.md` — 核心算法参考（含 YOLO 输出布局、class mapping）

## QoS 注意事项

订阅 `/image_raw` 用 `rclcpp::QoS(10)`（Reliable），不要用 `SensorDataQoS`（BestEffort），否则和 rosbag 发布的 QoS 不匹配会丢帧。
