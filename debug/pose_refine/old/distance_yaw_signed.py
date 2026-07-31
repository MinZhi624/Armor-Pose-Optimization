#!/usr/bin/env python3
"""
散点图: Δdistance vs (|θ_BA| − |θ_yaw_search|)

教学目的: distance 被允许移动时，BA 认为装甲板是更侧了还是更正了。
  x = Δdistance
  y = |θ_BA| − |θ_yaw_search|
  y > 0  → BA 认为装甲板更侧（|yaw| 增大）
  y < 0  → BA 认为装甲板更正（|yaw| 减小）

用法:
  python3 distance_yaw_signed.py --video video8 --ba-method pose_only_ba_4dof_ypd

输出: cmp/<video>/distance-yaw-signed.png
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
    if detailed:
        return os.path.join(method_dir, detailed[0])
    if "target.csv" in files:
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
    parser = argparse.ArgumentParser(description="Δdistance vs Δ|θ| 散点图")
    parser.add_argument("--video", default="video7")
    parser.add_argument("--corner-method", default="pca_gradient")
    parser.add_argument("--ba-method", default="pose_only_ba_4dof_xyz")
    parser.add_argument("--ba-video")
    parser.add_argument("--yaw-video")
    parser.add_argument("--log-dir")
    parser.add_argument("--output-dir")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = args.log_dir or os.path.join(script_dir, "log")
    output_dir = args.output_dir or os.path.join(script_dir, "cmp")

    ba_video = args.ba_video or args.video
    yaw_video = args.yaw_video or args.video
    out_video = args.video

    ba_path = find_csv(log_dir, ba_video, args.corner_method, args.ba_method)
    yaw_path = find_csv(log_dir, yaw_video, args.corner_method, "yaw_search")
    if not ba_path or not yaw_path:
        print("错误: 找不到 CSV", file=sys.stderr)
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

    ba_by_key = {(r["_frame"], r["_armor"]): r for r in ba_rows}
    yaw_by_key = {(r["_frame"], r["_armor"]): r for r in yaw_rows}
    all_keys = set(ba_by_key.keys()) | set(yaw_by_key.keys())

    x_vals, y_vals = [], []
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
        x_vals.append(br["_dist"])                                          # signed Δdistance
        y_vals.append((abs(br["_yaw"]) - abs(yr["_yaw"])) * 180.0 / np.pi)  # Δ|θ| in deg

    print(f"匹配: {matched}  仅 BA: {ba_only}  仅 yaw: {yaw_only}")
    if len(x_vals) < 2:
        print("错误: 点数不足", file=sys.stderr)
        return 1

    x_arr, y_arr = np.array(x_vals), np.array(y_vals)

    # Mahalanobis 距离过滤二维离群点
    cov = np.cov(x_arr, y_arr)
    try:
        inv_cov = np.linalg.inv(cov)
    except np.linalg.LinAlgError:
        inv_cov = np.linalg.pinv(cov)
    center = np.array([np.median(x_arr), np.median(y_arr)])
    d = np.array([np.sqrt((np.array([x, y]) - center).T @ inv_cov @ (np.array([x, y]) - center))
                  for x, y in zip(x_arr, y_arr)])
    threshold = np.percentile(d, 95)
    mask = d <= threshold
    x_f, y_f = x_arr[mask], y_arr[mask]
    n_out = int(mask.size - mask.sum())
    if n_out:
        print(f"过滤二维离群点: 移除 {n_out} 个 (Mahalanobis P95)")

    print(f"绘图点数: {len(x_f)}")

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.axhline(0, color="#888", linewidth=0.6, zorder=1)
    ax.axvline(0, color="#888", linewidth=0.6, zorder=1)
    ax.scatter(x_f, y_f, s=15, c="#2196F3", alpha=0.5, edgecolors="none", zorder=3)

    # 象限统计
    q = [(x_f > 0) & (y_f > 0), (x_f < 0) & (y_f > 0),
         (x_f < 0) & (y_f < 0), (x_f > 0) & (y_f < 0)]
    q_labels = ["Q1 (farther, more side)", "Q2 (closer, more side)",
                 "Q3 (closer, more frontal)", "Q4 (farther, more frontal)"]
    q_colors = ["#E91E63", "#FF9800", "#4CAF50", "#9C27B0"]
    for i, (qmask, qlabel, qcolor) in enumerate(zip(q, q_labels, q_colors)):
        nq = int(qmask.sum())
        if nq:
            ax.scatter(x_f[qmask], y_f[qmask], s=15, c=qcolor, alpha=0.5,
                       edgecolors="none", zorder=3)
            # 象限标签放在角落
            if i == 0:
                ax.text(0.94, 0.90, f"{qlabel}: {nq}", transform=ax.transAxes,
                        ha="right", va="top", fontsize=9, color=qcolor)
            elif i == 1:
                ax.text(0.06, 0.90, f"{qlabel}: {nq}", transform=ax.transAxes,
                        ha="left", va="top", fontsize=9, color=qcolor)
            elif i == 2:
                ax.text(0.06, 0.10, f"{qlabel}: {nq}", transform=ax.transAxes,
                        ha="left", va="bottom", fontsize=9, color=qcolor)
            else:
                ax.text(0.94, 0.10, f"{qlabel}: {nq}", transform=ax.transAxes,
                        ha="right", va="bottom", fontsize=9, color=qcolor)

    # Spearman
    if len(x_f) > 2:
        from scipy.stats import spearmanr
        rho, p = spearmanr(x_f, y_f)
        ax.text(0.97, 0.97, f"Spearman ρ = {rho:.3f}  (p={p:.2g})",
                transform=ax.transAxes, ha="right", va="top",
                fontsize=11, color="#333",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#ccc", alpha=0.8))

    ax.set_xlabel("Δdistance (m)")
    ax.set_ylabel("|θ_BA| − |θ_yaw_search| (deg)")
    ax.set_title(
        f"Distance vs |Yaw| Change\n"
        f"{args.corner_method} / {out_video}  ({len(x_f)} pts, {n_out} removed)",
        fontsize=12)
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(output_dir, out_video, "distance-yaw-signed.png")
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
