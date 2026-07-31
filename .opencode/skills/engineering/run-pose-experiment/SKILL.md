---
name: run-pose-experiment
disable-model-invocation: true
---

按视频跑完整的 pose_refine 实验：播完整个 rosbag，生成 CSV 日志，跑 XY 散点分析，输出测试文档。

## 流程

### 1. 构建

```bash
source /home/minzhi/intel/openvino_2026.1.0/setupvars.sh
colcon build --packages-select armor_detector --symlink-install
source install/setup.bash
```

完成后确认无编译错误。

### 2. 跑实验

对每个视频执行（`video7`、`video8` 等）：

```bash
ros2 launch armor_detector pose_refine_experiment.launch.py \
  video:=<name> frame_count:=0 use_foxglove:=false
```

- `frame_count:=0` 播完整个 bag，不限制帧数
- CSV 输出位置：`debug/pose_refine/log/<video>/<corner_method>/<refine_method>/<timestamp>.csv`

完成后确认日志出现 `auto test completed`，且对应 CSV 文件已生成。

### 3. 重命名为 target.csv

每个视频跑完后，把生成的 CSV 重命名为 `target.csv`：

```bash
python3 -c "
import os
log_root = 'debug/pose_refine/log'
for video in ['<video1>', '<video2>', ...]:
    video_dir = os.path.join(log_root, video)
    if not os.path.exists(video_dir): continue
    for root, dirs, files in os.walk(video_dir):
        for f in files:
            if not f.endswith('.csv') or f == 'target.csv': continue
            fp = os.path.join(root, f)
            dst = os.path.join(root, 'target.csv')
            os.rename(fp, dst)
            print(f'{fp} -> {dst}')
"
```

完成后确认每个视频目录下的 CSV 已改为 `target.csv`。

### 4. 跑 XY 散点分布

```bash
python3 debug/pose_refine/analyze_xy_distribution.py --all
```

脚本自动遍历 `debug/pose_refine/log/` 下所有 CSV，生成散点图到 `debug/pose_refine/analy/picture/`。

完成后确认每个 CSV 都生成了对应的 `*_xy_distribution.png`。

### 5. 统计位姿指标

```bash
python3 -c "
import csv, os
for root, dirs, files in os.walk('debug/pose_refine/log'):
    for f in files:
        if not f.endswith('.csv'): continue
        fp = os.path.join(root, f)
        with open(fp) as fh:
            rows = list(csv.DictReader(fh))
        if not rows: continue
        n = len(rows)
        succ = sum(1 for r in rows if r['success'] == 'true')
        rel = os.path.relpath(fp, 'debug/pose_refine/log')
        print(f'=== {rel} ===')
        print(f'samples: {n}, success: {succ}/{n}')
        for k in ['final_x_m','final_y_m','final_z_m','final_distance_m','final_pose_yaw_rad','final_reproj_mean_px']:
            v = [float(r[k]) for r in rows]
            print(f'  {k}: mean={sum(v)/len(v):.4f}  [{min(v):.4f}, {max(v):.4f}]')
        print()
"
```

### 6. 生成测试文档

写入 `docs/test-report-<experiment>.md`，包含：

- 测试配置表（方法、角点修正、后端、目标颜色）
- 每个视频的统计（样本数、成功率）
- 每个视频的位姿指标均值与范围
- 分析图路径
- 关键观察

完成后确认文档完整可读。
