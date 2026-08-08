#!/usr/bin/env python3
"""
PoseRefine final (x,y) 散点分布分析脚本:
- 搜索 log/ 下所有 *.csv → 读取 final_x_m, final_y_m
- 按 yaw 颜色绘制散点图
"""

import argparse
import csv
import os
import sys

import numpy as np
from matplotlib import pyplot as plt


def find_target_csv(log_root):
    results = []
    for root, _dirs, files in os.walk(log_root):
        for f in files:
            if f.endswith(".csv") and f != "index.csv":
                rel = os.path.relpath(root, log_root)
                parts = rel.split(os.sep)
                n = len(parts)
                if n == 2:
                    video, method = parts[0], parts[1]
                    results.append((video, "legacy", method, os.path.join(root, f), True))
                elif n >= 3:
                    video, corner_method, method = parts[0], parts[1], parts[2]
                    results.append((video, corner_method, method, os.path.join(root, f), False))
                else:
                    print(f"Skipping unexpected path depth ({n}): {rel}")
    return results


def read_xy(csv_path, max_frames=0):
    xs, ys, yaws, errors, ts = [], [], [], [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row["final_x_m"]))
            ys.append(float(row["final_y_m"]))
            yaws.append(float(row.get("final_pose_yaw_rad", 0)))
            errors.append(float(row.get("final_reproj_mean_px", 0)))
            ts.append(int(row["frame_index"]))
            if 0 < max_frames <= len(xs):
                break
    points = np.column_stack([np.array(xs), np.array(ys)])
    extras = {
        "yaw": np.array(yaws),
        "error": np.array(errors),
        "frame": np.array(ts, dtype=float),
    }
    return points, extras


def plot_scatter(points, extras, n_total, save_dir, prefix, label=""):
    x, y = points[:, 0], points[:, 1]
    yaw = extras["yaw"]

    os.makedirs(save_dir, exist_ok=True)
    prefix = prefix if prefix else str(n_total)

    fig, ax = plt.subplots(figsize=(10, 10))

    scatter = ax.scatter(x, y, s=10, c=yaw, cmap="hsv",
                         alpha=0.7, label=f"{n_total} points", zorder=2)
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
    cbar.set_label("final yaw (rad)")

    span = max(np.ptp(x), np.ptp(y)) * 0.55
    x_c, y_c = x.mean(), y.mean()
    ax.set_xlim(x_c - span, x_c + span)
    ax.set_ylim(y_c - span, y_c + span)
    ax.set_aspect("equal")

    title = f"XY Distribution — {n_total} points"
    if label:
        title += f"\n{label}"
    ax.set_title(title, fontsize=13)
    ax.legend(fontsize=10, loc="upper right")
    ax.grid(True, alpha=0.3)

    fname = f"{prefix}_xy_distribution.png"
    fig.savefig(os.path.join(save_dir, fname), dpi=150, bbox_inches="tight")
    print(f"Saved: {os.path.join(save_dir, fname)}")
    plt.close(fig)


def process_one(video, corner_method, refine_method, csv_path, legacy, max_frames, prefix):
    if legacy:
        print(f"\nProcessing: {video}/{refine_method} (legacy corner_method=legacy)")
        save_rel = os.path.join(video, refine_method)
        cm_display = "legacy"
    else:
        print(f"\nProcessing: {video}/{corner_method}/{refine_method}")
        save_rel = os.path.join(video, corner_method, refine_method)
        cm_display = corner_method
    points, extras = read_xy(csv_path, max_frames)
    print(f"  Loaded {len(points)} points")

    save_dir = os.path.join(os.path.dirname(__file__), "analy", "picture", save_rel)
    label = f"corner={cm_display}  refine={refine_method}"
    filename_parts = [prefix, cm_display, refine_method, str(len(points))]
    output_prefix = "_".join(part for part in filename_parts if part)
    plot_scatter(points, extras, len(points), save_dir, output_prefix, label=label)


def main():
    parser = argparse.ArgumentParser(description="PoseRefine XY scatter plot")
    parser.add_argument("--prefix", default="",
                        help="Output filename prefix")
    parser.add_argument("--max-frames", "-n", type=int, default=0,
                        help="Max frames to read (0=all)")
    parser.add_argument("--all", "-a", action="store_true",
                        help="Process all CSV files automatically")
    args = parser.parse_args()

    log_root = os.path.join(os.path.dirname(__file__), "log")
    targets = find_target_csv(log_root)

    if not targets:
        print("No CSV found under", log_root)
        sys.exit(1)

    if args.all:
        for video, corner_method, refine_method, csv_path, legacy in targets:
            process_one(video, corner_method, refine_method, csv_path, legacy,
                        args.max_frames, args.prefix)
        print("\nDone.")
        return

    if len(targets) == 1:
        chosen = targets[0]
    else:
        print("Found CSV:")
        for i, (v, cm, m, _p, leg) in enumerate(targets):
            label = f"{v}/{cm}/{m}" if not leg else f"{v}/{m} (legacy)"
            print(f"  [{i}] {label}")
        try:
            idx = int(input("Select index: "))
            chosen = targets[idx]
        except (ValueError, IndexError):
            print("Invalid")
            sys.exit(1)

    video, corner_method, refine_method, csv_path, legacy = chosen
    max_frames = args.max_frames
    if max_frames == 0:
        try:
            inp = input("Frames to read (0=all, Enter=all): ").strip()
            if inp:
                max_frames = int(inp)
        except (ValueError, EOFError):
            max_frames = 0
    process_one(video, corner_method, refine_method, csv_path, legacy,
                max_frames, args.prefix)


if __name__ == "__main__":
    main()
