#!/usr/bin/env python3
"""
PoseRefine final (x,y) 分布分析脚本:
- 搜索 target.csv → 读取 final_x_m, final_y_m
- 最小二乘圆拟合，过滤极端离群点
- 以圆心为中心出图
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
            if f == "target.csv":
                rel = os.path.relpath(root, log_root)
                parts = rel.split(os.sep)
                n = len(parts)
                if n == 2:
                    # legacy: log/<video>/<method>/target.csv
                    video, method = parts[0], parts[1]
                    results.append((video, "legacy", method, os.path.join(root, f), True))
                elif n >= 3:
                    # new: log/<video>/<corner_method>/<method>/target.csv
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
            yaws.append(float(row.get("final_yaw_rad", 0)))
            errors.append(float(row.get("final_error_px", 0)))
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


def fit_circle_ls(points):
    x, y = points[:, 0], points[:, 1]
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x ** 2 + y ** 2)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.sqrt(cx ** 2 + cy ** 2 - c)
    return cx, cy, r


def filter_outliers(points, extras, cx, cy, r, sigma=2.0):
    dists = np.abs(np.linalg.norm(points - np.array([[cx, cy]]), axis=1) - r)
    threshold = np.mean(dists) + sigma * np.std(dists)
    mask = dists < threshold
    return points[mask], {k: v[mask] for k, v in extras.items()}, mask


def plot_ls(points, filtered, extras, cx, cy, r, n_total, save_dir, prefix, label=""):
    x, y = filtered[:, 0], filtered[:, 1]
    yaw = extras["yaw"]

    os.makedirs(save_dir, exist_ok=True)
    prefix = prefix if prefix else str(n_total)

    fig, ax = plt.subplots(figsize=(10, 10))

    theta = np.linspace(0, 2 * np.pi, 400)
    ax.plot(cx + r * np.cos(theta), cy + r * np.sin(theta), "r-", lw=2.5, zorder=3)

    scatter = ax.scatter(x, y, s=10, c=yaw, cmap="hsv",
                         alpha=0.7, label=f"inliers ({len(filtered)}/{n_total})", zorder=2)
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.8)
    cbar.set_label("final yaw (rad)")

    ax.plot(cx, cy, "rx", markersize=15, mew=3, zorder=4)
    ax.annotate(f"center=({cx:.4f}, {cy:.4f})", xy=(cx, cy),
                fontsize=12, xytext=(10, -20), textcoords="offset points",
                arrowprops=dict(arrowstyle="->", color="gray"))

    ax.plot([cx, cx + r], [cy, cy], "k--", lw=1, alpha=0.5)
    ax.annotate(f"r={r:.4f}m", xy=(cx + r / 2, cy), fontsize=11,
                xytext=(0, 12), textcoords="offset points", ha="center")

    span = max(x.ptp(), y.ptp()) * 0.6
    x_c, y_c = x.mean(), y.mean()
    ax.set_xlim(x_c - span, x_c + span)
    ax.set_ylim(y_c - span, y_c + span)
    ax.set_aspect("equal")

    title = f"Least Squares Circle Fit — {n_total} points"
    if label:
        title += f"\n{label}"
    ax.set_title(title, fontsize=13)
    ax.legend(fontsize=10, loc="upper right")
    ax.grid(True, alpha=0.3)

    fname = f"{prefix}_xy_distribution.png"
    fig.savefig(os.path.join(save_dir, fname), dpi=150, bbox_inches="tight")
    print(f"Saved: {os.path.join(save_dir, fname)}")
    plt.close(fig)


def process_one(video, corner_method, refine_method, csv_path, legacy, max_frames, sigma, prefix):
    """处理单个 target.csv"""
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

    cx, cy, r = fit_circle_ls(points)
    print(f"  LS: center=({cx:.4f}, {cy:.4f}), radius={r:.4f}")

    filtered, filtered_extras, mask = filter_outliers(points, extras, cx, cy, r, sigma)
    n_out = int((~mask).sum())
    if n_out:
        print(f"  Filtered {n_out} outliers (sigma={sigma})")

    save_dir = os.path.join(os.path.dirname(__file__), "analy", "picture", save_rel)
    label = f"corner={cm_display}  refine={refine_method}"
    plot_ls(points, filtered, filtered_extras, cx, cy, r, len(points), save_dir, prefix, label=label)


def main():
    parser = argparse.ArgumentParser(description="PoseRefine XY circle fit")
    parser.add_argument("--prefix", default="",
                        help="Output filename prefix")
    parser.add_argument("--sigma", type=float, default=2.0,
                        help="Outlier threshold in sigma (default=2.0)")
    parser.add_argument("--max-frames", "-n", type=int, default=0,
                        help="Max frames to read (0=all)")
    parser.add_argument("--all", "-a", action="store_true",
                        help="Process all target.csv automatically")
    args = parser.parse_args()

    log_root = os.path.join(os.path.dirname(__file__), "log")
    targets = find_target_csv(log_root)

    if not targets:
        print("No target.csv found under", log_root)
        sys.exit(1)

    if args.all:
        for video, corner_method, refine_method, csv_path, legacy in targets:
            process_one(video, corner_method, refine_method, csv_path, legacy,
                        args.max_frames, args.sigma, args.prefix)
        print("\nDone.")
        return

    if len(targets) == 1:
        chosen = targets[0]
    else:
        print("Found target.csv:")
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
                max_frames, args.sigma, args.prefix)


if __name__ == "__main__":
    main()
