# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
colcon build --packages-select armor_detector
source install/setup.bash
ros2 launch armor_detector video1.launch.py   # 启动 rosbag 播放 + 检测节点
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
                    cv::imshow 显示
```

### 核心组件

- **DetectorNode** (`src/DetectorNode.cpp`) — ROS2 Node，订阅 `/image_raw`，callback 里调用 `CameraProvider` 转换图像并显示。ESC 退出。
- **CameraProvider** — 纯数据类，不管 ROS 通信。`init()` 从 yaml 加载相机内参，`receiveImage(msg)` 用 `cv_bridge::toCvShare` 零拷贝转换为 `Frame`。
- **Frame** — `cv::Mat image` + `builtin_interfaces::msg::Time timestamp`。
- **CameraInfo** — 自定义结构体，用 `cv::Matx` 固定大小矩阵存储内参、畸变、校正、投影矩阵。数据来源 `config/camera_info.yaml`。

### Debug 系统 (Observer Pattern)

- **IDebugObserver** — 基接口，定义各阶段虚方法（onFrameStart, onPreprocess, onLights, onArmorMatch, onClassification, onPoseSolved, onFrameEnd, onKey）
- **DebugHub** — 事件分发器，维护 observer 列表并转发事件
- **DebugGUI** — OpenCV 窗口统一管理，所有 imshow/waitKey 只在此处执行
- **专用观察者**: DebugLightView, DebugArmorMatchView, DebugClassificationView, DebugPoseView, DebugTiming, DebugRoiRecorder

### 待实现（当前为空 stub）

LightDetector → ArmorDetector → PoseSolver，构成完整检测流水线。

## Dependencies

ROS2: `rclcpp`, `sensor_msgs`, `cv_bridge`, `ament_index_cpp`
第三方: OpenCV, yaml-cpp, Eigen3

## 参考实现

源项目位于 `/home/minzhi/Desktop/Visual-Translationo`，有实现时先参考那里。

## 文档

- `docs/Conventions.md` — 单位、坐标系、cv::Mat 生命周期规则
- `docs/DebugGUI.md` — GUI 注册和显示标准
- `docs/Debug.md` — Debug 系统扩展指南

## QoS 注意事项

订阅 `/image_raw` 用 `rclcpp::QoS(10)`（Reliable），不要用 `SensorDataQoS`（BestEffort），否则和 rosbag 发布的 QoS 不匹配会丢帧。
