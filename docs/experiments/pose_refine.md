# Pose Refine 实验报告
## 实验目的

  比较 PnP 原始输出 (none) 和 PnP 初值 + yaw 搜索 (yaw_search) 对重投影误差和运
  行稳定性的影响。

  现在只是学习中
## 实验环境
  - 数据集：video1
  - 帧数：150
  - 检测后端：yolo
  - 角点修正：pca_gradient
  - 相机参数：config/camera_info.yaml
  - 日期：2026-06-30
## 方法说明

   方法          含义
   none          直接使用 PnP selected candidate
   yaw_search    固定 PnP xyz，只优化装甲板 yaw

  ## 指标

   指标          含义
   refine err    refine 后四角点重投影误差均值，单位 px
   pose ms       pose 阶段平均耗时，单位 ms
## 实验结果

### 1
视屏: video1
方法: none
帧率: 150

记录原始数据
[armor_detector_node-2] [INFO] [1782809703.342106915] [DebugTiming]: [yolo] 最近50帧: [total:10.11ms] [detect:9.94ms [corner_correction:0.52ms, decode:0.26ms, filter:0.00ms, infer:8.20ms, letterbox:0.94ms], pose:0.13ms, refine:none err:4.41px]
[armor_detector_node-2] [INFO] [1782809704.344033588] [DebugTiming]: [yolo] 最近50帧: [total:9.36ms] [detect:9.20ms [corner_correction:0.53ms, decode:0.26ms, filter:0.00ms, infer:7.46ms, letterbox:0.94ms], pose:0.14ms, refine:none err:4.57px]
[armor_detector_node-2] [INFO] [1782809705.425814611] [DebugTiming]: [yolo] 最近50帧: [total:9.47ms] [detect:9.33ms [corner_correction:0.54ms, decode:0.26ms, filter:0.00ms, infer:7.58ms, letterbox:0.93ms], pose:0.14ms, refine:none err:4.72px]
### 2
视屏: video1
方法: yaw_search
帧率: 150

记录原始数据
[armor_detector_node-2] [INFO] [1782809721.773009015] [DebugTiming]: [yolo] 最近50帧: [total:10.60ms] [detect:10.15ms [corner_correction:0.52ms, decode:0.27ms, filter:0.00ms, infer:8.41ms, letterbox:0.94ms], pose:0.41ms, refine:yaw_search err:4.15px]
[armor_detector_node-2] [INFO] [1782809722.952169520] [DebugTiming]: [yolo] 最近50帧: [total:9.82ms] [detect:9.37ms [corner_correction:0.57ms, decode:0.28ms, filter:0.00ms, infer:7.58ms, letterbox:0.92ms], pose:0.43ms, refine:yaw_search err:4.39px]
[armor_detector_node-2] [INFO] [1782809724.026726267] [DebugTiming]: [yolo] 最近50帧: [total:9.48ms] [detect:9.13ms [corner_correction:0.53ms, decode:0.24ms, filter:0.00ms, infer:7.34ms, letterbox:1.01ms], pose:0.37ms, refine:yaw_search err:4.60px]
