# Debug 使用指南

本文档基于当前工作区代码，仅说明 `debug/` 下交互式调试与离线分析脚本的实际行为与用法。

---

## 1. 环境与交互式 Debug 启动

从仓库根目录依次执行：

```bash
source /home/minzhi/intel/openvino_2026.1.0/setupvars.sh
colcon build --packages-select armor_detector
source install/setup.bash
ros2 launch armor_detector run.launch.py video:=video7
```

- `video:=video7` 可替换为 `video1` … `video7`。
- 若你以其他方式启动节点并希望启用 rosbag 控制，请保证 `debug.rosbag_control=true` 且 rosbag player 节点名为 `/rosbag2_player`。

---

## 2. `config.yaml` 中 GUI/图层/CSV 相关开关

`src/armor_detector/config/config.yaml` 的 `debug` 段与可视化、日志输出直接相关：

| 键 | 作用 | 默认值 |
|---|---|---|
| `debug.show` | OpenCV 窗口总开关；`false` 为无头模式 | `true` |
| `debug.rosbag_control` | 允许程序通过 ROS service 控制 rosbag 播放 | `true` |
| `debug.rosbag_player_node` | rosbag player 节点名 | `/rosbag2_player` |
| `debug.detect_stage_1` | `1` 键：传统=预处理 / YOLO=letterbox | `false` |
| `debug.detect_stage_2` | `2` 键：传统=灯条 / YOLO=score 过滤 | `false` |
| `debug.detect_stage_3` | `3` 键：传统=装甲匹配 / YOLO=NMS 结果 | `false` |
| `debug.detect_stage_4` | `4` 键：传统=数字分类 / YOLO=后端最终输出 | `false` |
| `debug.corner_correction` | `6` 键：角点修正对比（原始→修正） | `false` |
| `debug.pose` | `5` 键：启用 `/armor_markers` 发布 | `true` |
| `debug.pose_refine` | `7` 键：显示 Pose Refine 重投影角点 | `false` |
| `debug.result` | `0` 键：最终装甲板 X 标记 | `true` |
| `debug.pose_refine_csv.enabled` | 是否将 pose_refine 结果落盘为 CSV | `false` |
| `debug.pose_refine_csv.root_dir` | CSV 输出根目录；实验 launch 会填入 workspace 根目录 | `""` |
| `debug.pose_refine_csv.video` | 生成的 CSV 子目录对应的视频名 | `manual` |
| `debug.pose_refine_topic.enabled` | 是否发布 `/debug/armor_yaw` | `true` |
| `debug.stats_interval` | 统计信息打印间隔（帧数） | `50` |

其中 `pose_refine_csv.enabled` 是生成离线分析数据的核心开关。

---

## 3. 实际热键

以下热键由 `src/armor_detector/src/debug/DebugKeyHandler.cpp` 实现：

| 按键 | 动作 |
|---|---|
| `Esc` / `q` | 退出程序 |
| `Space` / `p` | 暂停/继续 |
| `n` / 右箭头 | 下一帧 |
| `+` / `=` | 播放倍率增加 |
| `-` / `_` | 播放倍率减少 |
| `0` | 切换最终结果层（装甲板 X 标记） |
| `1` | 切换检测阶段 1 图层 |
| `2` | 切换检测阶段 2 图层 |
| `3` | 切换检测阶段 3 图层 |
| `4` | 切换检测阶段 4 图层 |
| `5` | 切换 Pose 可视化层 |
| `6` | 切换角点修正对比层 |
| `7` | 切换 Pose Refine 重投影角点层 |

> 注意：`n`、`Space/p`、`+/-` 依赖 `debug.rosbag_control=true` 和 `/rosbag2_player` 提供的控制 service。`n` 会请求播放下一帧，只有 rosbag 已暂停时才有意义；`pose_refine_experiment.launch.py` 会以暂停状态启动 rosbag。

---

## 4. Pose Refine CSV 的生成与 target.csv

### 生成路径

当 `debug.pose_refine_csv.enabled=true` 时，CSV 输出到：

```
debug/pose_refine/log/<video>/<corner_method>/<refine_method>/<timestamp>.csv
```

`<timestamp>` 是启动时刻生成的时间戳文件名。CSV 表头包含但不限于：

- `frame_index`
- `armor_name`
- `success`
- `final_x_m`
- `final_y_m`
- `final_yaw_rad`
- `final_error_px`

### 生成一轮实验 CSV

在完成环境初始化并 source `install/setup.bash` 后，可运行无头实验：

```bash
ros2 launch armor_detector pose_refine_experiment.launch.py \
  video:=video7 \
  frame_count:=150 \
  use_foxglove:=false
```

该 launch 会开启 CSV 写入，并将输出根目录设为 workspace。当前要切换位姿优化方法时，先修改 `src/armor_detector/config/config.yaml` 中的 `pose.refine_method`，再分别运行实验；详见本文的限制说明。

### 准备 target.csv

离线分析脚本只读取名为 `target.csv` 的文件。将某一时刻的 CSV 目录准备为分析输入的操作：

1. 在 `debug/pose_refine/log/<video>/<corner_method>/<refine_method>/` 下找到你关心的 `<timestamp>.csv`。
2. 复制一份并命名为 `target.csv`，放在同一目录下：

```bash
cp debug/pose_refine/log/video7/pca_gradient/pose_only_ba_6dof/20250616_120000.csv \
   debug/pose_refine/log/video7/pca_gradient/pose_only_ba_6dof/target.csv
```

如果你只有 legacy（不含 corner_method）路径，复制到 `debug/pose_refine/log/<video>/<refine_method>/target.csv` 即可。

---

## 5. XY 脚本与 X 对比脚本

### XY 分布分析

```bash
# 交互选择单个 target.csv 并绘制 final_x_m vs final_y_m 散点
python3 debug/pose_refine/analyze_xy_distribution.py

# 自动处理当前 log 下所有 target.csv
python3 debug/pose_refine/analyze_xy_distribution.py --all --max-frames 300
```

新版目录的输出路径为 `debug/pose_refine/analy/picture/<video>/<corner_method>/<refine_method>/`；legacy 路径则为 `debug/pose_refine/analy/picture/<video>/<refine_method>/`。

### X 多方法对比

```bash
# 1) 先列出当前 video+corner_method 下各 armor_name 的可用帧数
python3 debug/pose_refine/analyze_x_comparison.py \
  --video video7 \
  --corner-method pca_gradient \
  --list-armors

# 2) 选定 armor_name 后，绘制所有发现 refine_method 的 X(mm) 曲线
python3 debug/pose_refine/analyze_x_comparison.py \
  --video video7 \
  --corner-method pca_gradient \
  --armor-name 1 \
  --max-frames 200

# 3) 只比较指定方法（空格分隔）
python3 debug/pose_refine/analyze_x_comparison.py \
  --video video7 \
  --corner-method pca_gradient \
  --armor-name 1 \
  --methods pose_only_ba_6dof pose_only_ba_4dof
```

输出图片写入 `debug/pose_refine/analy/picture/cmp/`，文件名形如：

```
video7_pca_gradient_1_x_comparison.png
```

若输入使用 legacy 目录结构，请传入 `--corner-method legacy`。

---

## 6. 自定义方法 / 外部 CSV 的最小约定

若你要添加自定义 `refine_method` 或外部 CSV，只需保证：

1. 目录结构满足：
   ```
   debug/pose_refine/log/<video>/<corner_method>/<refine_method>/target.csv
   ```
   或 legacy：
   ```
   debug/pose_refine/log/<video>/<refine_method>/target.csv
   ```

2. CSV 至少包含以下列（大小写敏感）：

   - `frame_index` — 整数帧序号
   - `armor_name` — 装甲板名称
   - `success` — 布尔值字符串，仅接受 `true`（不区分大小写）的条目会被读入
   - `final_x_m` — X 坐标，单位米

3. 同一 `frame_index` 下出现多条同名 `armor_name` 且 `success=true` 的记录时，该帧会被跳过并打印 Warning，曲线会出现断线。

---

## 7. 重要限制与注意事项

- **X 轴定义**：这里的 `X` 指云台前向轴（gimbal forward），不是图像坐标系水平轴。
- **单位**：CSV 中 `final_x_m` 为**米**；绘图脚本会自动乘以 `1000`，图中单位为 **毫米**。
- **无真值**：当前离线分析脚本不依赖真值，只关注同一 armor_name 在不同方法下的 X 曲线稳定性与一致性。
- **launch 不转发 `refine_method`**：`src/armor_detector/launch/pose_refine_experiment.launch.py` 并没有把启动参数 `refine_method` 设置到节点参数里。因此：
  - 不要在命令行中声称 `refine_method:=...` 可以切换方法；
  - 手动实验切换方法时，应先编辑 `src/armor_detector/config/config.yaml` 的 `pose.refine_method` 字段。
- **`batch_run.py` 的限制**：`debug/pose_refine/batch_run.py` 通过循环修改 `config.yaml` 的 `detector.corner_correction.method` 并重新 launch 来批量跑不同角点修正方法；但它同样受上述限制影响，不会真正切换 `pose.refine_method`，因此**不应作为严格 BA 方法比较的依据**。
