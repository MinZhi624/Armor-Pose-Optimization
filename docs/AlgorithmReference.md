# 核心算法参考文档

本文档是 `armor_detector` 重构阶段的核心算法参考入口。后续检测、分类、PnP、坐标转换、调试分析等核心算法实现，应先在这里登记参考依据，再进入代码实现。

## 使用规则

- 核心算法不要直接凭记忆写进代码，先在本文档登记来源、公式、输入输出和适用范围。
- 如果代码实现和参考依据不一致，必须在本文档说明原因。
- 如果一个算法有多个可选方案，先在本文档记录优缺点和当前选择，再实现代码。
- 算法代码中的关键公式注释，应能对应回本文档的小节。

## 当前算法登记表

## 图像预处理

### 参考来源

- 来源名称：OpenCV 阈值处理
- 参考内容：灰度转换 + 颜色阈值分割

### 输入

- 输入数据：BGR 图像
- 坐标系：image pixel
- 单位：`px`

### 输出

- 输出数据：二值图 (img_thre)
- 坐标系：image pixel

### 对应代码

- 头文件：`include/armor_detector/detector/Detector.hpp`
- 实现文件：`src/detector/Detector.cpp`

## 灯条检测

### 参考来源

- 来源名称：OpenCV 轮廓检测 + RotatedRect
- 参考内容：从二值图提取灯条轮廓，拟合旋转矩形

### 输入

- 输入数据：二值图、原始图像
- 坐标系：image pixel

### 输出

- 输出数据：`LightBar` 列表

### 对应代码

- 头文件：`include/armor_detector/detector/LightDetector.hpp`
- 实现文件：`src/detector/LightDetector.cpp`

## 装甲板匹配

### 参考来源

- 来源名称：RoboMaster 灯条配对规则
- 参考内容：左右灯条按 x 坐标排序，角度差/长度差/距离比例筛选

### 输入

- 输入数据：`LightBar` 列表

### 输出

- 输出数据：`ArmorCandidate` 列表

### 判断条件

- 角度差阈值：`max_angle_diff` (默认 15°)
- 长度比：`min_length_ratio` (默认 0.7)
- x 差异比：`min_x_diff_ratio` (默认 0.75)
- 距离比范围：`min_distance_ratio` ~ `max_distance_ratio`
- 大小装甲板判定：`kLargeArmorDistanceRatio = 2.8`

### 对应代码

- 头文件：`include/armor_detector/detector/ArmorDetector.hpp`
- 实现文件：`src/detector/ArmorDetector.cpp`

## 数字分类

### 参考来源

- 来源名称：ONNX CNN
- 参考内容：数字 ROI 裁剪后送入轻量 CNN 分类

### 输入

- 输入数据：装甲板候选、数字 ROI
- 坐标系：image pixel

### 输出

- 输出数据：`ClassifiedArmor` 列表（含 `ArmorName` + `confidence`）

### 对应代码

- 头文件：`include/armor_detector/NumberClassifier.hpp`
- 实现文件：`src/NumberClassifier.cpp`

## PnP 位姿解算

### 参考来源

- 来源名称：OpenCV solvePnP + 参考项目 PoseSolver
- 来源位置：`/home/minzhi/Desktop/Visual-Translationo/src/armor_plate_identification`
- 参考内容：IPPE 双解 + iterative fallback，几何选择 + yaw 连续性修正

### 输入

- 输入数据：装甲板 2D 角点、3D 模型点、相机内参、畸变参数
- 坐标系：image pixel / armor local / camera
- 单位：`px` / `m`

### 输出

- 输出数据：`SolvedArmor`（含 `ArmorPose`）
- 坐标系：camera / gimbal
- 单位：`m` / `rad`

### 3D 模型点

小装甲板 (0.135m × 0.055m)：

```text
(0, +0.0675, +0.0275)  left_top
(0, -0.0675, +0.0275)  right_top
(0, -0.0675, -0.0275)  right_bottom
(0, +0.0675, -0.0275)  left_bottom
```

大装甲板 (0.225m × 0.055m)：

```text
(0, +0.1125, +0.0275)  left_top
(0, -0.1125, +0.0275)  right_top
(0, -0.1125, -0.0275)  right_bottom
(0, +0.1125, -0.0275)  left_bottom
```

坐标系：X 法向量，Y 左，Z 上。

### PnP 流程

1. 优先 `cv::solvePnPGeneric(..., SOLVEPNP_IPPE)` 获取 2 个候选
2. IPPE 无候选时 fallback 到 `cv::solvePnP(..., SOLVEPNP_ITERATIVE)`
3. 每个候选计算 yaw、world_pitch、重投影误差
4. `selectByGeometry`: pitch 合法性 + 重投影误差最小
5. `selectBestCandidate`: 帧间 yaw 连续性修正突变解
6. yaw 重投影优化（YawSearch）

### 对应代码

- 头文件：`include/armor_detector/PoseSolver.hpp`
- 实现文件：`src/PoseSolver.cpp`

## yaw 优化 / 搜索

### 参考来源

- 来源名称：参考项目 YawSearch
- 来源位置：`/home/minzhi/Desktop/Visual-Translationo/src/armor_plate_identification/src/yaw/YawSearch.cpp`
- 参考内容：两阶段搜索（枚举粗搜 + 三分精搜）

### 输入

- 输入数据：PnP 初始 yaw、重投影误差函数
- 坐标系：gimbal
- 单位：`rad`

### 输出

- 输出数据：优化后的 yaw
- 单位：`rad`

### 搜索参数

| 参数 | 值 | 说明 |
|---|---|---|
| 搜索范围 | 30° | 枚举区间半径 |
| 枚举步长 | 4° | 粗搜采样间隔 |
| 局部范围 | 3° | 三分搜索区间半径 |
| 三分迭代 | 8 次 | 精搜迭代次数 |

### 流程

1. 枚举 `[center - 30°, center + 30°]`，每 4° 采样，找最小误差 yaw
2. 在粗搜结果 ±3° 范围内做 8 次三分搜索精修
3. 返回精修后的 yaw

### 对应代码

- 头文件：`include/armor_detector/yaw/YawSearch.hpp`
- 实现文件：`src/yaw/YawSearch.cpp`
- 主要函数：`runYawSearch(center_yaw, error_func)`

## 拒绝原因与调试数据

### 参考来源

- 来源名称：工程约定
- 参考内容：debug 数据只记录阶段结果，不改变算法输出

### 对应代码

- 头文件：`include/armor_detector/debug/DebugData.hpp`
- 实现文件：按具体 Observer 决定
