# Pose Refine 模块

PnP 位姿解算后的优化阶段。固定 xyz 不变（或联合优化），搜索使重投影误差最小的姿态参数。

## 架构

```
pose/
  IPoseRefiner.hpp          # 纯虚接口
  PoseRefineData.hpp/cpp    # 数据类型 + 枚举转换
  PoseRefineRunner.hpp/cpp  # 编译期方法分发
  NoneRefiner.hpp/cpp       # 不优化，直接返回 PnP 初始值
  YawSearchRefiner.hpp/cpp  # yaw 搜索优化
  PoseSearch.hpp/cpp        # 纯数学搜索工具（枚举 + 三分）
  PoseSolver.hpp/cpp        # PnP 求解 + 调用 refine
```

调用链：

```
DetectorNode::run()
  → PoseSolver::solve(armors, pose_refiner)
      → IPPE 求解 → 候选选择
      → PoseRefineRunner::refine(input, error_func)
          → switch(method_)
              NONE:      NoneRefiner      → 直接返回
              YAW_SEARCH: YawSearchRefiner → PoseSearch 搜索
      → 填充 PoseRefineDebugRecord
  → DebugHub::onPoseSolved()
      → DebugPoseRefineStats    (终端统计)
      → DebugPoseRefineCsvWriter (CSV 文件)
      → DebugPoseRefineTopicPublisher (ROS2 topic)
```

## 数据类型

定义在 `pose/PoseRefineData.hpp`：

```cpp
struct PoseRefineInput {
    std::array<cv::Point2f, 4> image_corners;   // 四角点图像坐标 (px)
    Eigen::Vector3d initial_xyz_gimbal;          // PnP 初始位置 (m, gimbal 系)
    double initial_yaw_rad;                      // PnP 初始 yaw (rad)
    ArmorType armor_type;                        // LARGE / SMALL
};

struct PoseRefineOutput {
    Eigen::Vector3d xyz_gimbal;      // 优化后位置 (m)
    double yaw_rad;                  // 优化后 yaw (rad)
    bool success;                    // 是否成功
    double reprojection_error_px;    // 优化后重投影误差 (px)
};
```

`PoseErrorFunction` 签名：

```cpp
using PoseErrorFunction = std::function<double(const Eigen::Vector3d &xyz_gimbal, double yaw_rad)>;
```

接受 gimbal 系 xyz + yaw，返回四角点重投影误差总和 (px)。由 `PoseSolver::calculatePoseRefineReprojectionError` 提供实现。

## 现有方法

### none — 不优化

`NoneRefiner`：直接返回 PnP 初始值，用于对比基准。

### yaw_search — yaw 搜索优化

`YawSearchRefiner`：固定 xyz，只优化 yaw。内部调用 `PoseSearch::refineYawFromPnp`。

两阶段搜索：

| 阶段 | 函数 | 范围 | 步长/迭代 | 说明 |
|------|------|------|-----------|------|
| 粗搜 | `enumerateYaw` | ±30° | 4° | 均匀采样，找大致最优 |
| 精搜 | `ternaryYaw` | ±3° | 8 轮 | 三分搜索，指数收敛 |

搜索参数硬编码在 `PoseSearch.cpp`，后续可参数化。

## PoseSearch 工具

`pose/PoseSearch.hpp` 提供纯数学搜索，不依赖 ROS/OpenCV/相机模型：

```cpp
double enumerateYaw(double center_yaw, const YawErrorFunction &calculate_error,
                    double search_range_rad, double step_rad);

double ternaryYaw(double initial_yaw, const YawErrorFunction &calculate_error,
                  double local_range_rad, int iterations);

double refineYawFromPnp(double center_yaw, const YawErrorFunction &calculate_error);
```

`YawErrorFunction = std::function<double(double)>`：接受 yaw (rad)，返回误差。

## Debug 基础设施

三个 Observer 通过 `DebugHub::onPoseSolved()` 事件触发，均继承 `IDebugObserver`。

### DebugPoseRefineStats

终端周期打印统计。每 `stats_interval` 帧输出：

```
最近50帧: method=yaw_search samples:87 success:87 fail:0
init_err:4.41px final_err:4.15px improv:0.26px
|yaw|:0.01rad |xyz|:0.00m
```

### DebugPoseRefineCsvWriter

每帧每装甲板写一行 CSV，路径格式：

```
debug/pose_refine/log/<video>/<method>/<timestamp>.csv
```

CSV 列：`frame_index, stamp_sec, stamp_nanosec, armor_index, armor_name, armor_type, confidence, center_x_px, center_y_px, method, success, initial/final/delta xyz (m), initial/final/delta yaw (rad), initial/final/delta error (px)`

### DebugPoseRefineTopicPublisher

发布 `/debug/pose_refine` topic（`armor_interfaces::msg::PoseRefineDebug`），受 `debug.pose` 图层控制。只发布第一个成功的 record。

消息定义：

```
std_msgs/Header header
float64 origin_yaw_rad
float64 final_yaw_rad
float64 error_delta_px
```

## 配置

`config/config.yaml`：

```yaml
detector:
  pose:
    refine_method: "yaw_search"   # "yaw_search" | "none"

debug:
  pose: true                      # 5键: /armor_markers + topic 发布
  pose_refine_csv:
    enabled: false                # 实验 launch 强制开启
    root_dir: ""                  # workspace root（launch 自动填入）
    video: "manual"               # 视频名（launch 自动填入）
  pose_refine_topic:
    enabled: true                 # /debug/pose_refine topic
  stats_interval: 50              # Stats 打印间隔（帧数）
```

ROS 参数覆盖（launch 文件常用）：

```python
'pose.refine_method': refine_method,
'debug.pose_refine_csv.enabled': True,
'debug.pose_refine_csv.root_dir': workspace_root,
'debug.pose_refine_csv.video': video,
'debug.pose_refine_topic.enabled': True,
```

## 实验流程

### 1. 运行实验

```bash
# 交互模式
ros2 launch armor_detector pose_refine_experiment.launch.py \
    video:=video1 refine_method:=yaw_search

# 自动测试
ros2 launch armor_detector auto_test.launch.py \
    video:=video1 frame_count:=150
```

CSV 自动输出到 `debug/pose_refine/log/<video>/<method>/`。

### 2. 选择 target.csv

将想要分析的 CSV 复制为：

```bash
cp debug/pose_refine/log/video1/yaw_search/<timestamp>.csv \
   debug/pose_refine/log/video1/yaw_search/target.csv
```

### 3. 分析 XY 分布

```bash
cd debug/pose_refine
python3 analyze_xy_distribution.py
```

输出：`analy/picture/<video>/<method>/xy_distribution.png`

## 添加新方法

1. 在 `PoseRefineData.hpp` 的 `PoseRefineMethod` 枚举中加新值
2. 在 `PoseRefineData.cpp` 的 `toString` / `poseRefineMethodFromString` 中加对应转换
3. 创建 `XxxRefiner.hpp/cpp`，继承 `IPoseRefiner`，实现 `refine()`
4. 在 `PoseRefineRunner.hpp` 中加成员，在 `PoseRefineRunner.cpp` 的 switch 中加 case
5. 在 `config.yaml` 中加默认值

接口签名：

```cpp
class IPoseRefiner {
public:
    virtual ~IPoseRefiner() = default;
    virtual PoseRefineOutput refine(const PoseRefineInput &input,
                                    const PoseErrorFunction &calculate_error) const = 0;
};
```

## 单位约定

遵循 `docs/Conventions.md`：

| 量 | 单位 | 后缀 |
|----|------|------|
| xyz_gimbal | m | `_m` / 无后缀 |
| yaw_rad | rad | `_rad` |
| 重投影误差 | px | `_px` |
| image_corners | px | 无后缀 |
