# Pose Refine 实验测试报告 — video8

**日期**: 2026-07-29
**测试视频**: video8 (744 frames, 43.2s 时长, YOLO 后端, RED 目标)

---

## 测试配置

| 配置 | corner_method | single_refine_method | dual_refine_method |
|------|--------------|---------------------|-------------------|
| 1 | pca_gradient | none | none |
| 2 | pca_gradient | yaw_search | none |
| 3 | pca_gradient | yaw_search_then_distance | none |
| 4 | pca_gradient | pose_only_ba_6dof | none |
| 5 | pca_gradient | pose_only_ba_4dof_xyz | none |
| 6 | pca_gradient | pose_only_ba_4dof_ypd | none |
| 7 | pca_gradient | yaw_search_then_distance | dual_armor_ba_3dof_ypd |
| 8 | none | none | none |

所有实验使用 `pose_refine_experiment.launch.py`，`frame_count:=0`（播完整 bag），`use_foxglove:=false`。

---

## 样本统计

| 配置 | 样本数 | 成功 | 成功率 |
|------|--------|------|--------|
| none + none + none | 831 | 831 | 100.0% |
| pca + none + none | 754 | 754 | 100.0% |
| pca + yaw_search + none | 754 | 754 | 100.0% |
| pca + yaw_search_then_distance + none | 754 | 754 | 100.0% |
| pca + ba_6dof + none | 754 | 754 | 100.0% |
| pca + ba_4dof_xyz + none | 754 | 754 | 100.0% |
| pca + ba_4dof_ypd + none | 754 | 754 | 100.0% |
| pca + yaw_search_then_distance + dual_ba | 754 | 754 | 100.0% |

> **注意**: none+none+none 样本数 831 > 其他 754，因为无角点修正时检测到的装甲板更多。

---

## 位姿指标

### final_x_m

| 配置 | mean | [min, max] |
|------|------|------------|
| none + none + none | 5.3583 | [4.1225, 6.5918] |
| pca + none + none | 6.0662 | [4.5843, 7.6544] |
| pca + yaw_search | 6.0662 | [4.5843, 7.6544] |
| pca + yaw_search_then_distance | 6.0857 | [4.5707, 7.6446] |
| pca + ba_6dof | 6.0678 | [4.5831, 7.6871] |
| pca + ba_4dof_xyz | 6.1085 | [4.5500, 7.6606] |
| pca + ba_4dof_ypd | 6.1080 | [4.5499, 7.6606] |
| pca + yaw_search_then_distance + dual | 5.8932 | [4.4479, 7.6443] |

### final_y_m

| 配置 | mean | [min, max] |
|------|------|------------|
| none + none + none | 0.4276 | [0.2408, 0.6899] |
| pca + none + none | 0.4818 | [0.2722, 0.8830] |
| pca + all single methods | 0.48-0.48 | similar |
| pca + yaw_search_then_distance + dual | 0.4672 | [0.2716, 0.8050] |

### final_z_m

| 配置 | mean | [min, max] |
|------|------|------------|
| none + none + none | 0.0127 | [-0.0160, 0.0404] |
| pca + all methods | 0.015 | [-0.019, 0.048] (approx) |

### final_distance_m

| 配置 | mean | [min, max] |
|------|------|------------|
| none + none + none | 5.3769 | [4.1489, 6.5992] |
| pca + none + none | 6.0871 | [4.6130, 7.6617] |
| pca + ba_4dof_xyz | 6.1295 | [4.5785, 7.6679] |
| pca + yaw_search_then_distance + dual | 5.9134 | [4.4757, 7.6524] |

### final_pose_yaw_rad

| 配置 | mean | [min, max] |
|------|------|------------|
| none + none + none | 0.1492 | [-0.9654, 1.1709] |
| pca + none + none | 0.1693 | [-0.7535, 1.0713] |
| pca + yaw_search | 0.1477 | [-0.7784, 1.0734] |
| pca + ba_4dof_xyz | 0.1393 | [-0.7712, 1.0763] |
| pca + yaw_search_then_distance + dual | 0.1400 | [-0.8464, 1.1261] |

### final_reproj_mean_px

| 配置 | mean | [min, max] |
|------|------|------------|
| none + none + none | **0.0805** | [0.0028, 0.2788] |
| pca + none + none | 0.3318 | [0.0062, 1.2331] |
| pca + ba_6dof | 0.3313 | [0.0063, 1.2291] |
| pca + yaw_search | 0.9484 | [0.1677, 4.7475] |
| pca + yaw_search_then_distance | 0.9450 | [0.1693, 4.7207] |
| pca + ba_4dof_xyz | 0.9340 | [0.1694, 4.1531] |
| pca + ba_4dof_ypd | 0.9339 | [0.1694, 4.1531] |
| pca + yaw_search_then_distance + dual | 1.0663 | [0.1727, 4.2590] |

---

## 分析图路径

暂未生成 XY 散点图。如需生成，请运行：

```bash
python3 debug/pose_refine/analyze_xy_distribution.py --all
```

图输出路径: `debug/pose_refine/analy/picture/video8/<corner>/<method>/`

---

## 关键观察

1. **none+none+none 基准**: 无角点修正时重投影误差最低（0.08px 均值），样本数最多（831），距离均值 5.38m。说明原始 YOLO 角点在无干预时已经很稳定。

2. **pca 角点修正的影响**: PCA 修正后重投影误差从 0.08px 升至 0.33px，距离均值从 5.38m 变为 6.09m。说明 PCA 修正改变了角点位置分布，产生系统性偏移。

3. **单装甲板精化方法比较**:
   - **none / ba_6dof**: 重投影误差 ~0.33px，表现相同（ba_6dof 未做实质性修正）
   - **yaw_search / yaw_search_then_distance**: 重投影误差恶化到 ~0.95px，说明 yaw_search 在当前场景下并未改善角点重投影
   - **ba_4dof_xyz / ba_4dof_ypd**: 误差 ~0.93px，表现相似

4. **双装甲板联合优化**: dual_armor_ba_3dof_ypd 重投影误差最高（1.07px），且距离和 yaw 偏移更大。双装甲板联合优化在 video8 上效果不佳。

5. **所有配置 100% 成功率**，没有 fail 样本。

6. **yaw 范围**: none+none+none 的 yaw 范围 [-0.97, 1.17] rad，PCA 方法范围 [-0.75, 1.07] rad，PCA 略收窄。
