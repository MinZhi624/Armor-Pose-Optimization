# 双装甲板联合 BA 测试报告

## 测试配置

| 配置项 | 值 |
|---|---|
| 单板策略 | `yaw_search_then_distance` |
| 双板策略 | `dual_armor_ba_3dof_ypd` |
| 角点修正 | `pca_gradient` |
| 检测后端 | `yolo` |
| 目标颜色 | RED |
| 帧数 | 500 / video |
| 输出日志 | `debug/pose_refine/log/target/` |

## 测试视频

| 视频 | 帧数 | 有效记录数 | 成功率 |
|---|---|---|---|
| video7 | 500 | 488 | 100% |
| video8 | 500 | 510 | 100% |

> video7 产生的记录少于 500 帧是由于部分帧未检测到装甲板，属正常行为。

## 位姿统计

### video7 — RED 哨兵

| 指标 | 均值 | 范围 |
|---|---|---|
| x (m) | 5.66 | [4.34, 7.00] |
| y (m) | 0.01 | [-0.25, 0.29] |
| z (m) | 0.14 | [0.10, 0.19] |
| distance (m) | 5.66 | [4.35, 7.00] |
| pose_yaw (rad) | 0.08 | [-1.06, 1.11] |
| 重投影误差 (px) | 1.10 | [0.11, 3.37] |

### video8 — RED 哨兵

| 指标 | 均值 | 范围 |
|---|---|---|
| x (m) | 5.88 | [4.45, 7.64] |
| y (m) | 0.47 | [0.27, 0.81] |
| z (m) | 0.01 | [-0.02, 0.05] |
| distance (m) | 5.90 | [4.48, 7.65] |
| pose_yaw (rad) | 0.13 | [-0.85, 1.13] |
| 重投影误差 (px) | 1.08 | [0.24, 4.16] |

## XY 分布分析

分析脚本 `analyze_xy_distribution.py` 分别对两个 CSV 生成散点图，按 yaw 着色：

- **video7** (510 点) → `*_510_xy_distribution.png`
- **video8** (488 点) → `*_488_xy_distribution.png`

输出位置：

```
debug/pose_refine/analy/picture/target/pca_gradient/
  single_yaw_search_then_distance__dual_dual_armor_ba_3dof_ypd/
    pca_gradient_single_yaw_search_then_distance__dual_dual_armor_ba_3dof_ypd_488_xy_distribution.png
    pca_gradient_single_yaw_search_then_distance__dual_dual_armor_ba_3dof_ypd_510_xy_distribution.png
```

## 观察结论

1. 两视频均 100% 通过，未出现 refine 失败帧。
2. video7 y 方向分布以 0 为中心（侧面视角），video8 y 显著偏正（~0.47m，偏上视角）。
3. 重投影误差均值约 1.1 px，最大不超过 4.2 px，说明位姿估计在配准精度上表现稳定。
4. 双板联合 BA 在此批处理中正常工作，未出现异常退化或失败路径触发。
