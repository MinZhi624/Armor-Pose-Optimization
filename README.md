# armor_detector

RoboMaster 装甲板检测系统，基于 ROS2 + OpenCV。

## 功能

从相机图像中检测 RoboMaster 机器人装甲板，并进行位姿估计。当前已完成图像采集与显示，检测流水线开发中。

## 依赖

- ROS2 (Jazzy)
- OpenCV
- yaml-cpp

## 构建

```bash
colcon build --packages-select armor_detector
source install/setup.bash
```

## 运行

```bash
ros2 launch armor_detector video1.launch.py
```

启动后会自动播放 rosbag 并打开图像窗口，按 ESC 退出。

测试数据在 `src/armor_detector/Test/video/video1` ~ `video5`。

## 目录结构

```
src/armor_detector/
  config/                # 相机内参配置
  include/armor_detector/ # 头文件
  launch/                # ROS2 launch 文件
  src/                   # 源码
  Test/video/            # 测试用 rosbag 数据
```

## 检测流水线

```
image_raw → CameraProvider → Frame
                                ↓
                         LightDetector (待实现)
                                ↓
                         ArmorDetector (待实现)
                                ↓
                          PoseSolver (待实现)
```
