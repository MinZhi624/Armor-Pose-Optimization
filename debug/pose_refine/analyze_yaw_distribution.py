#!/usr/bin/env python3
"""
PoseRefine pose-yaw 分布总览:

- 每个 video 生成一张方法对比图
- 每种方法读取自己的 target.csv，并纳入全部 success=true 的 pose yaw observation
- 用观测点、中央 50%/90% 区间和中位数直观展示分布
- CSV 中的 yaw 使用 rad，图形和统计输出统一使用 deg
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator


RAD2DEG = 180.0 / np.pi
YAW_COLUMNS = ("final_pose_yaw_rad", "final_yaw_rad")
UNIFORMITY_MIN_DEG = -70.0
UNIFORMITY_MAX_DEG = 70.0
UNIFORMITY_BIN_WIDTH_DEG = 5.0
UNIFORMITY_BIN_EDGES_DEG = np.arange(
    UNIFORMITY_MIN_DEG,
    UNIFORMITY_MAX_DEG + UNIFORMITY_BIN_WIDTH_DEG,
    UNIFORMITY_BIN_WIDTH_DEG,
)
UNIFORMITY_BIN_COUNT = len(UNIFORMITY_BIN_EDGES_DEG) - 1


@dataclass(frozen=True)
class Target:
    """一个待读取的 target.csv 及其方法标签。"""

    video: str
    corner_method: str
    refine_method: str
    csv_path: Path


@dataclass(frozen=True)
class ReadResult:
    """一个 target.csv 的有效观测和读取统计。"""

    yaws_deg: np.ndarray
    total_rows: int
    non_success_rows: int
    invalid_rows: int
    frame_filtered_rows: int
    duplicate_keys: int
    armor_index_available: bool


@dataclass(frozen=True)
class YawStats:
    """一组 pose yaw observation 的可读摘要。"""

    minimum_deg: float
    p05_deg: float
    p25_deg: float
    median_deg: float
    p75_deg: float
    p95_deg: float
    maximum_deg: float


@dataclass(frozen=True)
class MethodDistribution:
    """一行方法分布图所需的全部数据。"""

    target: Target
    yaws_deg: np.ndarray
    stats: YawStats
    read_result: ReadResult


@dataclass(frozen=True)
class UniformityStats:
    """固定 yaw 范围内一组 observation 的覆盖均匀性摘要。"""

    counts: np.ndarray
    percentages: np.ndarray
    ratios: np.ndarray
    in_range_count: int
    outside_count: int
    expected_per_bin: float
    uniformity_score: float | None
    peak_bin_index: int | None
    empty_bins: int


def discover_videos(log_root: Path) -> list[str]:
    """返回 log 根目录下的 video 目录名。"""
    if not log_root.is_dir():
        return []
    return sorted(path.name for path in log_root.iterdir() if path.is_dir())


def find_target_csv(log_root: Path) -> list[Target]:
    """
    只发现 target.csv，不回退到同目录中的时间戳 CSV。

    支持两种布局:
      log/<video>/<refine_method>/target.csv
      log/<video>/<corner_method>/<refine_method>/target.csv
    """
    targets: list[Target] = []
    if not log_root.is_dir():
        return targets

    for root_string, dirs, files in os.walk(log_root):
        dirs.sort()
        files.sort()
        csv_files = {name for name in files if name.endswith(".csv") and name != "index.csv"}
        if not csv_files:
            continue

        root = Path(root_string)
        if "target.csv" not in csv_files:
            rel = root.relative_to(log_root)
            print(f"Skipping {rel}: target.csv not found")
            continue

        parts = root.relative_to(log_root).parts
        if len(parts) == 2:
            video, refine_method = parts
            corner_method = "legacy"
        elif len(parts) == 3:
            video, corner_method, refine_method = parts
        else:
            print(
                f"Skipping {root.relative_to(log_root)}: "
                f"unexpected path depth ({len(parts)})"
            )
            continue

        targets.append(
            Target(
                video=video,
                corner_method=corner_method,
                refine_method=refine_method,
                csv_path=root / "target.csv",
            )
        )

    return sorted(targets, key=lambda target: (
        target.video,
        target.corner_method,
        target.refine_method,
        str(target.csv_path),
    ))


def pick_yaw_column(headers: list[str]) -> str:
    """按优先级选择当前和旧格式的 pose yaw 列。"""
    for name in YAW_COLUMNS:
        if name in headers:
            return name
    expected = ", ".join(YAW_COLUMNS)
    raise ValueError(f"missing yaw column; expected one of: {expected}")


def _is_success(row: dict[str, str]) -> bool:
    return (row.get("success") or "").strip().lower() == "true"


def _parse_frame_index(raw: str | None) -> int:
    value = int((raw or "").strip())
    if value < 0:
        raise ValueError("frame_index must be non-negative")
    return value


def _parse_yaw_deg(raw: str | None) -> float:
    value = float((raw or "").strip()) * RAD2DEG
    if not math.isfinite(value):
        raise ValueError("yaw is not finite")
    return value


def read_yaw(csv_path: Path, max_frames: int = 0) -> ReadResult:
    """
    读取 target.csv 中有效的 pose yaw observation。

    `max_frames` 表示 frame_index < max_frames；0 表示不限制。
    非 success 行不算 invalid observation，成功但字段错误的行会被计入 invalid_rows。
    """
    yaws_deg: list[float] = []
    total_rows = 0
    non_success_rows = 0
    invalid_rows = 0
    frame_filtered_rows = 0
    duplicate_keys = 0
    seen_keys: set[tuple[int, str]] = set()

    with csv_path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        headers = reader.fieldnames or []
        required = {"success", "frame_index"}
        missing = sorted(required - set(headers))
        if missing:
            raise ValueError(f"missing required column(s): {', '.join(missing)}")
        yaw_column = pick_yaw_column(headers)
        armor_column = "armor_index" if "armor_index" in headers else None

        for row in reader:
            total_rows += 1
            if not _is_success(row):
                non_success_rows += 1
                continue

            try:
                frame_index = _parse_frame_index(row.get("frame_index", ""))
                if max_frames > 0 and frame_index >= max_frames:
                    frame_filtered_rows += 1
                    continue
                yaw_deg = _parse_yaw_deg(row.get(yaw_column, ""))
            except (TypeError, ValueError):
                invalid_rows += 1
                continue

            if armor_column is not None:
                armor_index = (row.get(armor_column) or "").strip()
                if armor_index:
                    key = (frame_index, armor_index)
                    if key in seen_keys:
                        duplicate_keys += 1
                    seen_keys.add(key)

            yaws_deg.append(yaw_deg)

    return ReadResult(
        yaws_deg=np.asarray(yaws_deg, dtype=float),
        total_rows=total_rows,
        non_success_rows=non_success_rows,
        invalid_rows=invalid_rows,
        frame_filtered_rows=frame_filtered_rows,
        duplicate_keys=duplicate_keys,
        armor_index_available=armor_column is not None,
    )


def calculate_stats(yaws_deg: np.ndarray) -> YawStats:
    """计算分布总览所需的 min/max、中央 50% 和中央 90%。"""
    if yaws_deg.size == 0:
        raise ValueError("cannot calculate statistics for an empty distribution")
    minimum, p05, p25, median, p75, p95, maximum = np.percentile(
        yaws_deg, [0, 5, 25, 50, 75, 95, 100]
    )
    return YawStats(
        minimum_deg=float(minimum),
        p05_deg=float(p05),
        p25_deg=float(p25),
        median_deg=float(median),
        p75_deg=float(p75),
        p95_deg=float(p95),
        maximum_deg=float(maximum),
    )


def calculate_uniformity(yaws_deg: np.ndarray) -> UniformityStats:
    """按固定 [-70°, 70°] / 5° 区间计算 yaw 覆盖均匀性。"""
    yaws_deg = np.asarray(yaws_deg, dtype=float)
    in_range = (yaws_deg >= UNIFORMITY_MIN_DEG) & (yaws_deg <= UNIFORMITY_MAX_DEG)
    in_range_yaws = yaws_deg[in_range]
    counts, _ = np.histogram(in_range_yaws, bins=UNIFORMITY_BIN_EDGES_DEG)
    counts = counts.astype(int)

    total_count = int(yaws_deg.size)
    in_range_count = int(counts.sum())
    outside_count = total_count - in_range_count
    percentages = counts.astype(float) / total_count * 100.0 if total_count else np.zeros(
        UNIFORMITY_BIN_COUNT, dtype=float
    )
    expected_per_bin = in_range_count / UNIFORMITY_BIN_COUNT

    if in_range_count:
        ratios = counts.astype(float) / expected_per_bin
        probabilities = counts.astype(float) / in_range_count
        nonzero_probabilities = probabilities[probabilities > 0.0]
        entropy = -float(np.sum(nonzero_probabilities * np.log(nonzero_probabilities)))
        uniformity_score = entropy / math.log(UNIFORMITY_BIN_COUNT) * 100.0
        peak_bin_index = int(np.argmax(counts))
    else:
        ratios = np.full(UNIFORMITY_BIN_COUNT, np.nan, dtype=float)
        uniformity_score = None
        peak_bin_index = None

    return UniformityStats(
        counts=counts,
        percentages=percentages,
        ratios=ratios,
        in_range_count=in_range_count,
        outside_count=outside_count,
        expected_per_bin=expected_per_bin,
        uniformity_score=uniformity_score,
        peak_bin_index=peak_bin_index,
        empty_bins=int(np.count_nonzero(counts == 0)),
    )


def load_method_distribution(target: Target, max_frames: int) -> MethodDistribution | None:
    """读取一个 target.csv；没有有效观测时返回 None。"""
    try:
        result = read_yaw(target.csv_path, max_frames)
    except (OSError, ValueError) as error:
        print(f"Skipping {target.csv_path}: {error}")
        return None

    if result.yaws_deg.size == 0:
        print(
            f"Skipping {target.csv_path}: no valid success=true pose yaw observations "
            f"(rows={result.total_rows}, non_success={result.non_success_rows}, "
            f"invalid={result.invalid_rows}, frame_filtered={result.frame_filtered_rows})"
        )
        return None

    stats = calculate_stats(result.yaws_deg)
    distribution = MethodDistribution(
        target=target,
        yaws_deg=result.yaws_deg,
        stats=stats,
        read_result=result,
    )
    print_method_summary(distribution)
    if result.duplicate_keys:
        print(
            f"  Warning: {target.csv_path} contains "
            f"{result.duplicate_keys} duplicate (frame_index, armor_index) keys; "
            "all valid rows are retained."
        )
    elif not result.armor_index_available:
        print(
            f"  Warning: {target.csv_path} has no armor_index column; "
            "duplicate (frame_index, armor_index) checks are unavailable."
        )
    return distribution


def print_method_summary(distribution: MethodDistribution) -> None:
    """打印单个方法的读取和分布摘要。"""
    result = distribution.read_result
    stats = distribution.stats
    print(
        f"  {distribution.target.corner_method}/{distribution.target.refine_method}: "
        f"N={distribution.yaws_deg.size}, median={stats.median_deg:.2f} deg, "
        f"range=[{stats.minimum_deg:.2f}, {stats.maximum_deg:.2f}] deg, "
        f"non_success={result.non_success_rows}, invalid={result.invalid_rows}, "
        f"frame_filtered={result.frame_filtered_rows}"
    )


def rounded_axis_limits(yaws_deg: np.ndarray) -> tuple[float, float]:
    """将联合 yaw 范围向外取整到 5°。"""
    minimum = float(np.min(yaws_deg))
    maximum = float(np.max(yaws_deg))
    lower = 5.0 * math.floor(minimum / 5.0)
    upper = 5.0 * math.ceil(maximum / 5.0)
    if lower == upper:
        lower -= 5.0
        upper += 5.0
    return lower, upper


def _point_alpha(sample_count: int) -> float:
    """样本越多点越透明，让重叠位置自然变深。"""
    return min(0.75, max(0.50, 120.0 / sample_count))


def _method_label(distribution: MethodDistribution) -> str:
    stats = distribution.stats
    return (
        f"{distribution.target.corner_method} / {distribution.target.refine_method}  "
        f"N={distribution.yaws_deg.size}  median={stats.median_deg:.1f}°"
    )


def plot_overview(
    video: str,
    distributions: list[MethodDistribution],
    output_root: Path,
    prefix: str = "",
) -> Path:
    """为一个 video 绘制共享 degree 横轴的方法分布总览。"""
    if not distributions:
        raise ValueError("cannot plot an empty method list")

    all_yaws = np.concatenate([distribution.yaws_deg for distribution in distributions])
    x_min, x_max = rounded_axis_limits(all_yaws)

    row_positions: list[float] = []
    current_position = 0.0
    previous_corner = None
    for distribution in distributions:
        corner_method = distribution.target.corner_method
        if previous_corner is not None and corner_method != previous_corner:
            current_position += 0.4
        row_positions.append(current_position)
        current_position += 1.0
        previous_corner = corner_method

    figure_height = max(3.8, 1.0 + 0.9 * len(distributions) + 0.45 * len(
        {distribution.target.corner_method for distribution in distributions}
    ))
    fig, ax = plt.subplots(figsize=(15, figure_height))
    colors = plt.get_cmap("tab20").colors
    rng = np.random.default_rng(20260728)

    for index, (distribution, row_position) in enumerate(zip(distributions, row_positions)):
        color = colors[index % len(colors)]
        yaws = distribution.yaws_deg
        stats = distribution.stats
        jitter = rng.uniform(-0.16, 0.16, size=yaws.size)

        # 所有观测点先画出；rasterized 保持大 CSV 的 PNG/矢量中间结果可控。
        ax.scatter(
            yaws,
            row_position + jitter,
            s=11,
            color=color,
            alpha=_point_alpha(yaws.size),
            edgecolors="none",
            rasterized=True,
            zorder=2,
        )

        # 中央 90% 和中央 50% 区间，以及中位数标记。
        ax.plot(
            [stats.p05_deg, stats.p95_deg],
            [row_position, row_position],
            color=color,
            linewidth=2.2,
            solid_capstyle="round",
            zorder=4,
        )
        ax.plot(
            [stats.p25_deg, stats.p75_deg],
            [row_position, row_position],
            color=color,
            linewidth=9.0,
            solid_capstyle="round",
            zorder=5,
        )
        ax.plot(
            [stats.median_deg, stats.median_deg],
            [row_position - 0.27, row_position + 0.27],
            color="#202020",
            linewidth=2.5,
            solid_capstyle="round",
            zorder=6,
        )

    ax.axvline(0.0, color="#555555", linestyle="--", linewidth=1.0, zorder=1)
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-0.55, row_positions[-1] + 0.55)
    ax.invert_yaxis()

    ax.set_yticks(row_positions)
    ax.set_yticklabels([_method_label(distribution) for distribution in distributions])
    ax.set_xlabel("final pose yaw (deg)", fontsize=12)
    ax.set_ylabel("method / observation summary", fontsize=11)
    ax.set_title(
        f"Pose Yaw Distribution Overview — {video}\n"
        f"{all_yaws.size} valid observations across {len(distributions)} methods",
        fontsize=14,
    )

    ax.xaxis.set_major_locator(MultipleLocator(10))
    ax.xaxis.set_minor_locator(MultipleLocator(5))
    ax.grid(axis="x", which="major", alpha=0.28)
    ax.grid(axis="x", which="minor", alpha=0.12)
    ax.tick_params(axis="y", length=0, pad=8)
    ax.tick_params(axis="x", which="minor", length=3)

    legend_handles = [
        Line2D(
            [0], [0], marker="o", linestyle="none", markerfacecolor="#607D8B",
            markeredgecolor="none", alpha=0.55, markersize=6,
            label="each pose-yaw observation",
        ),
        Line2D([0], [0], color="#607D8B", linewidth=2.2, label="central 90%"),
        Line2D([0], [0], color="#607D8B", linewidth=9.0, label="central 50%"),
        Line2D([0], [0], color="#202020", linewidth=2.5, label="median"),
        Line2D([0], [0], color="#555555", linestyle="--", linewidth=1.0, label="0° reference"),
    ]
    ax.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=5,
        fontsize=9,
        frameon=False,
    )

    fig.subplots_adjust(left=0.34, right=0.98, top=0.86, bottom=0.12)
    output_dir = output_root / video
    output_dir.mkdir(parents=True, exist_ok=True)
    filename = f"{prefix}yaw_distribution_overview.png"
    output_path = output_dir / filename
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {output_path}")
    return output_path


def _uniformity_bin_label(bin_index: int) -> str:
    start_deg = UNIFORMITY_BIN_EDGES_DEG[bin_index]
    end_deg = UNIFORMITY_BIN_EDGES_DEG[bin_index + 1]
    return f"{start_deg:g}~{end_deg:g}"


def _uniformity_score_label(stats: UniformityStats) -> str:
    if stats.uniformity_score is None:
        return "N/A"
    return f"{stats.uniformity_score:.1f}"


def _uniformity_summary(stats: UniformityStats) -> str:
    expected = f"{stats.expected_per_bin:.1f}" if stats.in_range_count else "N/A"
    if stats.peak_bin_index is None:
        peak = "N/A"
    else:
        peak_index = stats.peak_bin_index
        peak = f"{_uniformity_bin_label(peak_index)}°"
    return (
        f"expected/bin={expected}  peak={peak}  "
        f"empty={stats.empty_bins}  outside={stats.outside_count}"
    )


def plot_uniformity_overview(
    video: str,
    distributions: list[MethodDistribution],
    output_root: Path,
    prefix: str = "",
) -> Path:
    """绘制固定角度区间的 yaw 覆盖均匀性热力图。"""
    if not distributions:
        raise ValueError("cannot plot an empty uniformity distribution")

    uniformities = [calculate_uniformity(distribution.yaws_deg) for distribution in distributions]
    ratio_matrix = np.vstack([stats.ratios for stats in uniformities])
    masked_ratios = np.ma.masked_invalid(ratio_matrix)

    figure_height = max(4.5, 1.2 + 0.8 * len(distributions))
    fig, ax = plt.subplots(figsize=(24, figure_height))
    cmap = plt.get_cmap("RdBu_r").copy()
    cmap.set_bad("#D9D9D9")
    norm = TwoSlopeNorm(vmin=0.0, vcenter=1.0, vmax=2.0)
    image = ax.imshow(
        masked_ratios,
        aspect="auto",
        cmap=cmap,
        norm=norm,
        interpolation="none",
    )

    bin_indices = np.arange(UNIFORMITY_BIN_COUNT)
    ax.set_xticks(bin_indices)
    ax.set_xticklabels(
        [f"{_uniformity_bin_label(index)}°" for index in bin_indices],
        rotation=60,
        ha="right",
        fontsize=8,
    )
    ax.set_yticks(np.arange(len(distributions)))
    ax.set_yticklabels(
        [
            f"{distribution.target.corner_method} / {distribution.target.refine_method}  "
            f"N={distribution.yaws_deg.size}  U={_uniformity_score_label(stats)}"
            for distribution, stats in zip(distributions, uniformities)
        ],
        fontsize=9,
    )
    ax.set_xlabel("final pose yaw bin (deg), fixed range [-70°, 70°], width 5°", fontsize=11)
    ax.set_ylabel("method / N / uniformity (0~100)", fontsize=11)
    ax.set_title(
        f"Yaw Coverage Uniformity — {video}\n"
        "Cell text: count and percent of all observations; color: actual / uniform expected",
        fontsize=14,
    )

    ax.set_xticks(np.arange(-0.5, UNIFORMITY_BIN_COUNT, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(distributions), 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=0.9)
    ax.tick_params(which="minor", bottom=False, left=False)

    for row_index, (distribution, stats) in enumerate(zip(distributions, uniformities)):
        for bin_index in range(UNIFORMITY_BIN_COUNT):
            ratio = stats.ratios[bin_index]
            if np.isfinite(ratio):
                red, green, blue, _ = cmap(norm(float(np.clip(ratio, 0.0, 2.0))))
                luminance = 0.299 * red + 0.587 * green + 0.114 * blue
                text_color = "white" if luminance < 0.48 else "black"
            else:
                text_color = "#555555"
            ax.text(
                bin_index,
                row_index,
                f"{stats.counts[bin_index]}\n{stats.percentages[bin_index]:.1f}%",
                ha="center",
                va="center",
                fontsize=7.5,
                color=text_color,
                linespacing=0.9,
            )

        ax.text(
            1.01,
            row_index,
            _uniformity_summary(stats),
            transform=ax.get_yaxis_transform(),
            ha="left",
            va="center",
            fontsize=8.5,
            color="#333333",
            clip_on=False,
        )

    fig.subplots_adjust(left=0.27, right=0.69, top=0.82, bottom=0.27)
    colorbar_axis = fig.add_axes([0.87, 0.30, 0.018, 0.50])
    colorbar = fig.colorbar(image, cax=colorbar_axis, ticks=[0.0, 0.5, 1.0, 1.5, 2.0])
    colorbar.set_label("actual / uniform expected\n(1.0 = uniform)", fontsize=10)

    output_dir = output_root / video
    output_dir.mkdir(parents=True, exist_ok=True)
    filename = f"{prefix}yaw_uniformity_overview.png"
    output_path = output_dir / filename
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {output_path}")
    return output_path


def build_distributions(
    targets: list[Target],
    video: str,
    max_frames: int,
) -> list[MethodDistribution]:
    """读取一个 video 的 target.csv，并按方法稳定排序。"""
    distributions: list[MethodDistribution] = []
    for target in targets:
        if target.video != video:
            continue
        distribution = load_method_distribution(target, max_frames)
        if distribution is not None:
            distributions.append(distribution)
    return sorted(
        distributions,
        key=lambda distribution: (
            distribution.target.corner_method,
            distribution.target.refine_method,
        ),
    )


def select_video(videos: list[str]) -> str:
    """无参数运行时交互选择一个 video。"""
    print("Available videos:")
    for index, video in enumerate(videos):
        print(f"  [{index}] {video}")
    try:
        selected = int(input("Select video index: "))
        return videos[selected]
    except (EOFError, ValueError, IndexError) as error:
        raise ValueError("Invalid video selection") from error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PoseRefine pose-yaw distribution overview")
    parser.add_argument("--video", help="Generate one overview for this video")
    parser.add_argument("--all", "-a", action="store_true", help="Generate overviews for all videos")
    parser.add_argument("--prefix", default="", help="Output filename prefix")
    parser.add_argument(
        "--max-frames",
        "-n",
        type=int,
        default=0,
        help="Only include rows with frame_index < N (0=all)",
    )
    args = parser.parse_args()
    if args.video and args.all:
        parser.error("--video and --all cannot be used together")
    if args.max_frames < 0:
        parser.error("--max-frames must be non-negative")
    return args


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    log_root = script_dir / "log"
    output_root = script_dir / "analy" / "picture"

    videos = discover_videos(log_root)
    targets = find_target_csv(log_root)
    if not videos:
        print(f"No video directory found under {log_root}", file=sys.stderr)
        return 1
    if not targets:
        print(f"No target.csv found under {log_root}", file=sys.stderr)
        return 1

    target_videos = sorted({target.video for target in targets})
    if args.video:
        if args.video not in videos:
            print(f"Unknown video: {args.video}; available: {', '.join(videos)}", file=sys.stderr)
            return 1
        selected_videos = [args.video]
    elif args.all:
        selected_videos = target_videos
    else:
        try:
            selected_videos = [select_video(videos)]
        except ValueError as error:
            print(error, file=sys.stderr)
            return 1

    generated = 0
    for video in selected_videos:
        print(f"\nProcessing video: {video}")
        distributions = build_distributions(targets, video, args.max_frames)
        if not distributions:
            print(f"No valid target.csv methods for {video}; skipping.", file=sys.stderr)
            continue
        plot_overview(video, distributions, output_root, args.prefix)
        plot_uniformity_overview(video, distributions, output_root, args.prefix)
        generated += 1

    if generated == 0:
        print("No overview was generated.", file=sys.stderr)
        return 1
    print(f"\nDone. Generated {generated} overview(s).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
