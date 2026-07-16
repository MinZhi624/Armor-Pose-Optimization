#!/usr/bin/env python3
"""
散点图: |Δdistance| vs |θ_BA − θ_yaw_search|

教学目的: 直观验证 distance 被允许移动得越多时，pose_yaw 的解是否也越偏离固定 distance 时的解。

读取两个详细 CSV 日志（BA 和 yaw_search），按 (frame_index, armor_index) 合并后绘图。

用法:
  # 标准: video 目录名相同
  python3 distance_yaw_relative.py --video video7

  # 验证数据集（不同 video 名）
  python3 distance_yaw_relative.py \\
      --ba-video  validate_bamodel_ba_20260716 \\
      --yaw-video validate_bamodel_20260716

  # 限制帧数，不弹窗
  python3 distance_yaw_relative.py --video video8 --frames 200 --no-show

输出: cmp/<video>/distance-yaw-relative.png
"""

import argparse
import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt

COL_YAW = "final_pose_yaw_rad"
COL_YAW_ALT = "final_yaw_rad"
COL_DIST = "delta_distance_m"


def find_csv(log_dir, video, corner_method, refine_method):
    method_dir = os.path.join(log_dir, video, corner_method, refine_method)
    if not os.path.isdir(method_dir):
        return None

    files = [f for f in os.listdir(method_dir) if f.endswith(".csv")]
    if not files:
        return None

    detailed = sorted([f for f in files if f != "target.csv"], reverse=True)
    has_target = "target.csv" in files

    if detailed:
        return os.path.join(method_dir, detailed[0])
    if has_target:
        return os.path.join(method_dir, "target.csv")
    return os.path.join(method_dir, files[0]) if files else None


def pick_col(headers, *preferred):
    for name in preferred:
        if name in headers:
            return name
    return preferred[-1]


def read_rows(csv_path):
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []

        yaw_col = pick_col(headers, COL_YAW, COL_YAW_ALT)
        dist_col = pick_col(headers, COL_DIST)

        for row in reader:
            if row.get("success", "").strip().lower() != "true":
                continue

            frame = int(row["frame_index"])
            armor = int(row["armor_index"])
            yaw = float(row[yaw_col])

            if dist_col in headers:
                dist_delta = float(row[dist_col])
            else:
                dx = float(row["delta_x_m"])
                dy = float(row["delta_y_m"])
                dz = float(row["delta_z_m"])
                dist_delta = np.sqrt(dx * dx + dy * dy + dz * dz)

            rows.append(dict(_frame=frame, _armor=armor, _yaw=yaw, _dist=dist_delta))
    return rows


def main():
    parser = argparse.ArgumentParser(description="Distance-Yaw 耦合散点图")
    parser.add_argument("--video", default="video7",
                        help="视频目录名 (默认: video7; 被 --ba-video/--yaw-video 覆盖)")
    parser.add_argument("--corner-method", default="pca_gradient",
                        help="角点修正方法 (默认: pca_gradient)")
    parser.add_argument("--ba-method", default="pose_only_ba_4dof_xyz",
                        help="BA 方法名 (默认: pose_only_ba_4dof_xyz)")
    parser.add_argument("--ba-video",
                        help="BA 的视频目录名 (覆盖 --video)")
    parser.add_argument("--yaw-video",
                        help="yaw_search 的视频目录名 (覆盖 --video)")
    parser.add_argument("--log-dir",
                        help="log 根目录 (默认: debug/pose_refine/log)")
    parser.add_argument("--output-dir",
                        help="输出根目录 (默认: debug/pose_refine/cmp)")
    parser.add_argument("--frames", type=int, default=0,
                        help="最大帧数 (0=全部)")
    parser.add_argument("--no-show", action="store_true",
                        help="不弹窗，只保存")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = args.log_dir or os.path.join(script_dir, "log")
    output_dir = args.output_dir or os.path.join(script_dir, "cmp")

    ba_video = args.ba_video or args.video
    yaw_video = args.yaw_video or args.video
    out_video = args.video  # 用 --video 作为输出目录名

    ba_path = find_csv(log_dir, ba_video, args.corner_method, args.ba_method)
    yaw_path = find_csv(log_dir, yaw_video, args.corner_method, "yaw_search")

    if not ba_path:
        print(f"错误: 找不到 BA CSV ({ba_video}/{args.corner_method}/{args.ba_method})",
              file=sys.stderr)
        return 1
    if not yaw_path:
        print(f"错误: 找不到 yaw_search CSV ({yaw_video}/{args.corner_method}/yaw_search)",
              file=sys.stderr)
        return 1

    print(f"BA:         {ba_path}")
    print(f"Yaw search: {yaw_path}")

    ba_rows = read_rows(ba_path)
    yaw_rows = read_rows(yaw_path)
    print(f"BA 成功行:          {len(ba_rows)}")
    print(f"Yaw search 成功行:  {len(yaw_rows)}")

    if args.frames > 0:
        ba_rows = [r for r in ba_rows if r["_frame"] < args.frames]
        yaw_rows = [r for r in yaw_rows if r["_frame"] < args.frames]
        print(f"帧数限制 < {args.frames}: BA={len(ba_rows)}, Yaw={len(yaw_rows)}")

    ba_by_key = {(r["_frame"], r["_armor"]): r for r in ba_rows}
    yaw_by_key = {(r["_frame"], r["_armor"]): r for r in yaw_rows}
    all_keys = set(ba_by_key.keys()) | set(yaw_by_key.keys())

    x_vals, y_vals_deg = [], []
    matched = ba_only = yaw_only = 0

    for key in sorted(all_keys):
        br = ba_by_key.get(key)
        yr = yaw_by_key.get(key)
        if br is None:
            yaw_only += 1
            continue
        if yr is None:
            ba_only += 1
            continue
        matched += 1

        x = abs(br["_dist"])
        y = abs(br["_yaw"] - yr["_yaw"]) * 180.0 / np.pi
        x_vals.append(x)
        y_vals_deg.append(y)

    print(f"匹配: {matched}  仅 BA: {ba_only}  仅 yaw: {yaw_only}")

    if len(x_vals) < 2:
        print("错误: 匹配点数不足，无法绘图", file=sys.stderr)
        return 1

    # IQR 过滤 y 方向极端离群点
    x_vals, y_vals_deg = np.array(x_vals), np.array(y_vals_deg)
    q1, q3 = np.percentile(y_vals_deg, [25, 75])
    iqr = q3 - q1
    upper = q3 + 1.5 * iqr
    mask = y_vals_deg <= upper
    x_f, y_f = x_vals[mask], y_vals_deg[mask]
    n_removed = int(mask.size - mask.sum())
    if n_removed:
        print(f"过滤极端离群点: 移除 {n_removed} 个 (y > {upper:.1f} deg, IQR×1.5)")
    print(f"绘图点数: {len(x_f)}")

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.scatter(x_f, y_f, s=15, c="#2196F3", alpha=0.5,
               edgecolors="none", zorder=3)

    # 线性回归趋势线 + Spearman 秩相关系数
    if len(x_f) > 2:
        slope, intercept = np.polyfit(x_f, y_f, 1)
        x_line = np.linspace(x_f.min(), x_f.max(), 100)
        ax.plot(x_line, slope * x_line + intercept, "--", color="#E91E63",
                linewidth=1.2, zorder=4,
                label=f"trend: y = {slope:.2f}x + {intercept:.2f}")

        from scipy.stats import spearmanr
        rho, _ = spearmanr(x_f, y_f)
        ax.text(0.97, 0.95, f"Spearman ρ = {rho:.3f}",
                transform=ax.transAxes, ha="right", va="top",
                fontsize=11, color="#333",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#ccc", alpha=0.8))
        ax.legend(fontsize=9)

    ax.set_xlabel("|Δdistance| (m)")
    ax.set_ylabel("|θ_BA − θ_yaw_search| (deg)")
    ax.set_title(
        f"Distance-Yaw Coupling\n"
        f"{args.corner_method} / {out_video}  ({len(x_f)} pts, {n_removed} outliers removed)",
        fontsize=12)
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(output_dir, out_video, "distance-yaw-relative.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"保存: {out_path}")

    if not args.no_show:
        plt.show()
    else:
        plt.close(fig)

    return 0


if __name__ == "__main__":
    sys.exit(main())
