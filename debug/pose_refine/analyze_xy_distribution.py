#!/usr/bin/env python3
"""
PoseRefine XY 分布分析脚本:
- 搜索 target.csv → 读取 center_x_px, center_y_px
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
                video = parts[0] if len(parts) > 0 else "unknown"
                method = parts[1] if len(parts) > 1 else "unknown"
                results.append((video, method, os.path.join(root, f)))
    return results


def read_xy(csv_path):
    xs, ys, ts = [], [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row["center_x_px"]))
            ys.append(float(row["center_y_px"]))
            ts.append(int(row["frame_index"]))
    points = np.column_stack([np.array(xs), np.array(ys)])
    times = np.array(ts, dtype=float)
    return points, times


def fit_circle_ls(points):
    x, y = points[:, 0], points[:, 1]
    A = np.column_stack([x, y, np.ones_like(x)])
    b = -(x ** 2 + y ** 2)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.sqrt(cx ** 2 + cy ** 2 - c)
    return cx, cy, r


def filter_outliers(points, times, cx, cy, r, sigma=2.0):
    dists = np.abs(np.linalg.norm(points - np.array([[cx, cy]]), axis=1) - r)
    threshold = np.mean(dists) + sigma * np.std(dists)
    mask = dists < threshold
    return points[mask], times[mask], mask


def plot_ls(points, filtered, times, cx, cy, r, n_total, save_path):
    fig, ax = plt.subplots(figsize=(10, 10))

    # Circle
    theta = np.linspace(0, 2 * np.pi, 400)
    ax.plot(cx + r * np.cos(theta), cy + r * np.sin(theta), "r-", lw=2.5, zorder=3)

    # Points colored by time
    sc = ax.scatter(filtered[:, 0], filtered[:, 1], s=10, c=times, cmap="plasma",
                    alpha=0.7, label=f"inliers ({len(filtered)}/{n_total})", zorder=2)
    cbar = plt.colorbar(sc, ax=ax, label="frame index")
    cbar.ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f"{x:.0f}"))

    # Center
    ax.plot(cx, cy, "rx", markersize=15, mew=3, zorder=4)
    ax.annotate(f"center=({cx:.1f}, {cy:.1f})", xy=(cx, cy),
                fontsize=12, xytext=(10, -20), textcoords="offset points",
                arrowprops=dict(arrowstyle="->", color="gray"))

    # Radius annotation (horizontal line from center to edge)
    ax.plot([cx, cx + r], [cy, cy], "k--", lw=1, alpha=0.5)
    ax.annotate(f"r={r:.1f}px", xy=(cx + r / 2, cy), fontsize=11,
                xytext=(0, 12), textcoords="offset points", ha="center")

    # Center viewport on data points
    span = max(filtered[:, 0].ptp(), filtered[:, 1].ptp()) * 0.6
    x_c, y_c = filtered[:, 0].mean(), filtered[:, 1].mean()
    ax.set_xlim(x_c - span, x_c + span)
    ax.set_ylim(y_c - span, y_c + span)
    ax.set_aspect("equal")

    ax.set_title(f"Least Squares Circle Fit — {n_total} points", fontsize=13)
    ax.legend(fontsize=10, loc="upper right")
    ax.grid(True, alpha=0.3)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {save_path}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="PoseRefine XY circle fit")
    parser.add_argument("--prefix", default="",
                        help="Output filename prefix")
    parser.add_argument("--sigma", type=float, default=2.0,
                        help="Outlier threshold in sigma (default=2.0)")
    args = parser.parse_args()

    log_root = os.path.join(os.path.dirname(__file__), "log")
    targets = find_target_csv(log_root)

    if not targets:
        print("No target.csv found under", log_root)
        sys.exit(1)

    if len(targets) == 1:
        chosen = targets[0]
    else:
        print("Found target.csv:")
        for i, (v, m, p) in enumerate(targets):
            print(f"  [{i}] {v}/{m}")
        try:
            idx = int(input("Select index: "))
            chosen = targets[idx]
        except (ValueError, IndexError):
            print("Invalid")
            sys.exit(1)

    video, method, csv_path = chosen
    print(f"Processing: {video}/{method}")
    points, times = read_xy(csv_path)
    print(f"Loaded {len(points)} points")

    cx, cy, r = fit_circle_ls(points)
    print(f"LS: center=({cx:.2f}, {cy:.2f}), radius={r:.2f}")

    filtered, ftimes, mask = filter_outliers(points, times, cx, cy, r, args.sigma)
    n_out = int((~mask).sum())
    if n_out:
        print(f"Filtered {n_out} outliers (sigma={args.sigma})")

    save_dir = os.path.join(os.path.dirname(__file__), "analy", "picture", video, method)
    prefix = args.prefix if args.prefix else str(len(points))
    fname = f"{prefix}_xy_distribution.png"
    save_path = os.path.join(save_dir, fname)
    plot_ls(points, filtered, ftimes, cx, cy, r, len(points), save_path)


if __name__ == "__main__":
    main()
