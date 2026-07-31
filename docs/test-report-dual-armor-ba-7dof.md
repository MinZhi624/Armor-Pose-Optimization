# 双装甲板 7DoF 联合 BA 测试报告

**日期**: 2026-07-31
**实验**: 最新 7DoF 联合优化回归（`dual_armor_ba_7dof_xyz`），单板使用 `yaw_search_then_distance`

---

## 测试配置

| 配置项 | 值 |
|---|---|
| 单板策略 | `yaw_search_then_distance` |
| 双板策略 | `dual_armor_ba_7dof_xyz`（最新 7DoF 联合优化） |
| 角点修正 | `pca_gradient` |
| 检测后端 | `yolo`（`robot_0526.onnx`, CPU） |
| 目标颜色 | RED |
| 播放模式 | step，播完整个 bag（`frame_count:=0`） |
| GUI/Foxglove | 关闭（`use_foxglove:=false`） |
| 输出日志 | `debug/pose_refine/log/<video>/pca_gradient/single_yaw_search_then_distance__dual_dual_armor_ba_7dof_xyz/target.csv` |

## 测试视频

| 视频 | bag 帧数 | 有效记录数 | 成功率 |
|---|---|---|---|
| video7 | 1061 | 1046 | 100% |
| video8 | 744 | 754 | 100% |

> video7 有效记录少于 bag 帧数，是因为部分帧未检测到装甲板，属正常行为；video8 单帧可产生多条 armor 记录，因此记录数（754）略多于帧数（744）。

## 位姿统计

### video7 — RED 哨兵

| 指标 | 均值 | 范围 |
|---|---|---|
| x (m) | 5.6648 | [4.2235, 6.9997] |
| y (m) | 0.0053 | [-0.2510, 0.2912] |
| z (m) | 0.1431 | [0.0993, 0.1912] |
| distance (m) | 5.6687 | [4.2280, 7.0057] |
| pose_yaw (rad) | 0.0839 | [-1.0598, 1.1180] |
| 重投影误差 (px) | 1.0913 | [0.1108, 4.0659] |

### video8 — RED 哨兵

| 指标 | 均值 | 范围 |
|---|---|---|
| x (m) | 5.8927 | [4.4478, 7.6443] |
| y (m) | 0.4672 | [0.2681, 0.8050] |
| z (m) | 0.0146 | [-0.0190, 0.0454] |
| distance (m) | 5.9128 | [4.4757, 7.6524] |
| pose_yaw (rad) | 0.1400 | [-0.8464, 1.1260] |
| 重投影误差 (px) | 1.0659 | [0.1727, 4.2351] |

### 7DoF vs 3DoF 对比（同视频、同单板方法）

| 视频 | 方法 | 样本 | 成功率 | x mean | y mean | dist mean | yaw mean | reproj mean |
|---|---|---|---|---|---|---|---|---|
| video7 | 3dof_ypd | 1046 | 100% | 5.6649 | 0.0053 | 5.6688 | 0.0840 | 1.0918 |
| video7 | **7dof_xyz** | 1046 | 100% | 5.6648 | 0.0053 | 5.6687 | 0.0839 | **1.0913** |
| video8 | 3dof_ypd | 754 | 100% | 5.8932 | 0.4672 | 5.9134 | 0.1400 | 1.0663 |
| video8 | **7dof_xyz** | 754 | 100% | 5.8927 | 0.4672 | 5.9128 | 0.1400 | **1.0659** |

## XY 分布分析

分析脚本 `debug/pose_refine/analyze_xy_distribution.py --all` 生成散点图（按 final yaw 着色）：

- **video7** (1046 点) → `debug/pose_refine/analy/picture/video7/pca_gradient/single_yaw_search_then_distance__dual_dual_armor_ba_7dof_xyz/pca_gradient_single_yaw_search_then_distance__dual_dual_armor_ba_7dof_xyz_1046_xy_distribution.png`
- **video8** (754 点) → `debug/pose_refine/analy/picture/video8/pca_gradient/single_yaw_search_then_distance__dual_dual_armor_ba_7dof_xyz/pca_gradient_single_yaw_search_then_distance__dual_dual_armor_ba_7dof_xyz_754_xy_distribution.png`

## 关键观察

1. **7DoF 联合优化工作正常**：两个视频均 100% 成功率，无 refine 失败帧，未触发异常退化路径。
2. **与 3DoF 结果几乎一致**：7dof_xyz 在 x/y/z/distance/yaw 均值上与 3dof_ypd 的差异均在 1e-4~1e-3 量级；重投影误差略优（video7: 1.0913 vs 1.0918，video8: 1.0659 vs 1.0663）。说明 7DoF 在保持精度的同时提供了完整的 6DoF 平移+绕 z 旋转联合自由度。
3. **video7 y 分布以 0 为中心**（侧面视角，范围 [-0.25, 0.29] m），**video8 y 显著偏正**（~0.47 m，偏上视角），与历史观察一致。
4. **重投影误差均值约 1.07-1.09 px**，最大不超过 4.3 px，位姿配准稳定。
5. **位姿范围合理**：distance 覆盖 4.2~7.7 m，yaw 覆盖 ±1.1 rad，与 bag 中机器人远近和朝向变化吻合。
