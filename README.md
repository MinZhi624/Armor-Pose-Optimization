# armor_detector

RoboMaster 装甲板检测系统，基于 ROS2 + OpenCV + Eigen3。

## 功能

从相机图像中检测 RoboMaster 机器人装甲板，执行 PnP 位姿解算，输出装甲板在相机/云台坐标系下的 3D 位置和姿态。支持 Foxglove/RViz 3D marker 可视化和无头自动测试。

## 依赖

- ROS2 (Jazzy)
- OpenCV
- Eigen3
- yaml-cpp
- visualization_msgs
- foxglove_bridge（可选，用于 3D 可视化）

## 构建

```bash
colcon build --packages-select armor_detector
source install/setup.bash
```

## 运行

### 交互模式（带 GUI + Foxglove）

```bash
ros2 launch armor_detector video1.launch.py
```

启动 rosbag 循环播放 + 检测节点 + Foxglove bridge。按 ESC 退出 OpenCV 窗口。

Foxglove 连接 `ws://localhost:8765`，查看 `/armor_markers` topic。

按键操作：`[1-6]` 切换图层，`[5]` 切换 pose marker，`[Space]` 暂停，`[n]` 单步。

### 自动测试（无头 step 模式）

```bash
ros2 launch armor_detector auto_test.launch.py frame_count:=150
```

逐帧播放 rosbag，处理 150 帧后自动退出。可加 `timing_interval:=1` 每帧打印用时。

## 检测流水线

```
/image_raw → CameraProvider → Frame
                                  ↓
                           Detector (预处理 + 灯条检测)
                                  ↓
                         LightDetector → ArmorDetector
                                  ↓
                         NumberClassifier (数字分类)
                                  ↓
                         PoseSolver (PnP + yaw 优化)
                                  ↓
                         SolvedArmor (位置 + 姿态)
```

## 目录结构

```
src/armor_detector/
  config/                    # 相机内参、运行参数配置
  include/armor_detector/
    debug/                   # Debug observer 体系
    detector/                # 灯条/装甲板检测
    tools/                   # 坐标变换、几何常量
    types/                   # 数据结构定义
    yaw/                     # yaw 搜索算法
  launch/
    video1.launch.py         # 交互模式 launch
    auto_test.launch.py      # 无头自动测试 launch
  src/
    debug/                   # Debug observer 实现
    detector/                # 检测算法实现
    yaw/                     # yaw 搜索实现
  Test/video/                # 测试用 rosbag 数据 (video1~video5)
docs/                        # 设计文档和约定
```
