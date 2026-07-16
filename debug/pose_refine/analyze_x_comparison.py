#!/usr/bin/env python3
"""
PoseRefine X (gimbal forward) multi-method comparison script.

The script searches the same target.csv layout as analyze_xy_distribution.py,
then draws final_x_m in millimetres against frame_index for one armor_name.
"""

import argparse
import csv
import os
import re
import sys
from collections import defaultdict

import numpy as np
from matplotlib import pyplot as plt


REQUIRED_COLUMNS = {"frame_index", "armor_name", "success", "final_x_m"}


def find_target_csv(log_root):
    """Return target.csv files using the existing pose_refine directory layout."""
    results = []
    for root, _dirs, files in os.walk(log_root):
        for filename in files:
            if filename != "target.csv":
                continue

            relative_path = os.path.relpath(root, log_root)
            parts = relative_path.split(os.sep)
            if len(parts) == 2:
                video, refine_method = parts
                results.append((video, "legacy", refine_method, os.path.join(root, filename), True))
            elif len(parts) >= 3:
                video, corner_method, refine_method = parts[:3]
                results.append((video, corner_method, refine_method, os.path.join(root, filename), False))
            else:
                print(f"Skipping unexpected path depth ({len(parts)}): {relative_path}", file=sys.stderr)
    return results


def require_columns(reader, csv_path):
    field_names = set(reader.fieldnames or [])
    missing_columns = sorted(REQUIRED_COLUMNS - field_names)
    if missing_columns:
        raise ValueError(f"{csv_path} is missing required columns: {', '.join(missing_columns)}")


def frame_is_in_range(frame_index, max_frames):
    return max_frames == 0 or frame_index < max_frames


def read_armor_counts(csv_path, max_frames):
    """Return usable and ambiguous successful frame counts for every armor."""
    record_counts_by_armor = defaultdict(lambda: defaultdict(int))
    with open(csv_path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        require_columns(reader, csv_path)
        for row in reader:
            if row["success"].strip().lower() != "true":
                continue

            try:
                frame_index = int(row["frame_index"])
            except ValueError:
                continue

            armor_name = row["armor_name"].strip()
            if armor_name and frame_index >= 0 and frame_is_in_range(frame_index, max_frames):
                record_counts_by_armor[armor_name][frame_index] += 1

    return {
        armor_name: (
            sum(record_count == 1 for record_count in counts_by_frame.values()),
            sum(record_count > 1 for record_count in counts_by_frame.values()),
        )
        for armor_name, counts_by_frame in record_counts_by_armor.items()
    }


def read_x_series(csv_path, armor_name, max_frames):
    """Read one unambiguous successful X(mm) sample per frame from a CSV."""
    values_by_frame = {}
    duplicate_frames = set()
    failed_rows = 0
    invalid_rows = 0

    with open(csv_path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        require_columns(reader, csv_path)
        for row in reader:
            if row["armor_name"].strip() != armor_name:
                continue
            if row["success"].strip().lower() != "true":
                failed_rows += 1
                continue

            try:
                frame_index = int(row["frame_index"])
                x_mm = float(row["final_x_m"]) * 1000.0
            except ValueError:
                invalid_rows += 1
                continue

            if frame_index < 0 or not frame_is_in_range(frame_index, max_frames):
                continue

            if frame_index in duplicate_frames:
                continue
            if frame_index in values_by_frame:
                del values_by_frame[frame_index]
                duplicate_frames.add(frame_index)
                continue

            values_by_frame[frame_index] = x_mm

    if duplicate_frames:
        print(
            f"Warning: {csv_path}: skipped {len(duplicate_frames)} frame(s) with multiple successful "
            f"records for armor_name={armor_name}.",
            file=sys.stderr,
        )
    if invalid_rows:
        print(f"Warning: {csv_path}: skipped {invalid_rows} invalid row(s).", file=sys.stderr)

    return values_by_frame, failed_rows, len(duplicate_frames)


def normalize_methods(raw_methods):
    """Accept both '--methods a b' and '--methods a,b' forms."""
    if not raw_methods:
        return None

    methods = []
    for value in raw_methods:
        methods.extend(method.strip() for method in value.split(",") if method.strip())
    return methods


def select_targets(targets, video, corner_method, requested_methods):
    selected = []
    for target_video, target_corner_method, refine_method, csv_path, legacy in targets:
        if target_video == video and target_corner_method == corner_method:
            selected.append((refine_method, csv_path, legacy))

    if not selected:
        available_groups = sorted({(item[0], item[1]) for item in targets})
        formatted_groups = ", ".join(f"{item_video}/{item_corner}" for item_video, item_corner in available_groups)
        raise ValueError(
            f"No target.csv found for video={video}, corner_method={corner_method}. "
            f"Available groups: {formatted_groups or 'none'}"
        )

    by_method = {}
    for refine_method, csv_path, legacy in selected:
        if refine_method in by_method:
            raise ValueError(
                f"Multiple target.csv files found for refine_method={refine_method}: "
                f"{by_method[refine_method][0]} and {csv_path}"
            )
        by_method[refine_method] = (csv_path, legacy)

    if requested_methods:
        missing_methods = [method for method in requested_methods if method not in by_method]
        if missing_methods:
            raise ValueError(f"Requested method(s) not found: {', '.join(missing_methods)}")
        method_names = requested_methods
    else:
        method_names = sorted(by_method)

    return [(method, by_method[method][0], by_method[method][1]) for method in method_names]


def armor_sort_key(armor_name):
    try:
        return 0, int(armor_name)
    except ValueError:
        return 1, armor_name


def print_armor_counts(selected_targets, max_frames):
    """Print per-method usable frame coverage to help select armor_name."""
    counts_by_armor = defaultdict(dict)
    for refine_method, csv_path, _legacy in selected_targets:
        for armor_name, frame_counts in read_armor_counts(csv_path, max_frames).items():
            counts_by_armor[armor_name][refine_method] = frame_counts

    if not counts_by_armor:
        print("No successful armor records found.")
        return

    print("Available armor_name values (usable frames; ambiguous frames are skipped):")
    for armor_name in sorted(counts_by_armor, key=armor_sort_key):
        method_counts = counts_by_armor[armor_name]
        details = []
        for refine_method, _csv_path, _legacy in selected_targets:
            usable_count, ambiguous_count = method_counts.get(refine_method, (0, 0))
            detail = f"{refine_method}={usable_count}"
            if ambiguous_count:
                detail += f" (ambiguous={ambiguous_count})"
            details.append(detail)
        detail = ", ".join(details)
        print(f"  armor_name={armor_name}: {detail}")


def safe_filename_component(value):
    return re.sub(r"[^\w.-]+", "_", value, flags=re.UNICODE).strip("_.") or "unnamed"


def plot_x_comparison(series_by_method, video, corner_method, armor_name, output_path):
    all_frames = [frame_index for values_by_frame in series_by_method.values() for frame_index in values_by_frame]
    if not all_frames:
        raise ValueError("No valid X samples remain after filtering.")

    frame_start = min(all_frames)
    frame_end = max(all_frames)
    frame_axis = np.arange(frame_start, frame_end + 1)

    fig, ax = plt.subplots(figsize=(12, 6.5))
    for refine_method, values_by_frame in series_by_method.items():
        x_values_mm = np.full(frame_axis.shape, np.nan)
        for frame_index, value_mm in values_by_frame.items():
            x_values_mm[frame_index - frame_start] = value_mm
        ax.plot(
            frame_axis,
            x_values_mm,
            linewidth=1.1,
            marker=".",
            markersize=2.0,
            label=f"{refine_method} ({len(values_by_frame)} frames)",
        )

    ax.set_title(
        "X (Gimbal Forward) Comparison\n"
        f"video={video}  corner={corner_method}  armor_name={armor_name}",
        fontsize=13,
    )
    ax.set_xlabel("Frame")
    ax.set_ylabel("X (mm, gimbal)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc="best")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def parse_arguments():
    parser = argparse.ArgumentParser(description="PoseRefine X(mm) multi-method comparison plot")
    parser.add_argument("--video", default="video7", help="Video directory under log/ (default: video7)")
    parser.add_argument(
        "--corner-method",
        default="pca_gradient",
        help="Corner correction directory under the selected video (default: pca_gradient)",
    )
    parser.add_argument("--armor-name", help="Only plot this exact armor_name")
    parser.add_argument(
        "--methods",
        nargs="+",
        help="Optional refine methods to plot; accepts spaces or comma-separated values",
    )
    parser.add_argument(
        "--list-armors",
        action="store_true",
        help="List successful armor_name coverage for the selected method group and exit",
    )
    parser.add_argument(
        "--max-frames",
        "-n",
        type=int,
        default=0,
        help="Only include frame_index values in [0, N); 0 means all frames (default: 0)",
    )
    parser.add_argument("--prefix", default="", help="Optional prefix for the output filename")
    return parser.parse_args()


def main():
    args = parse_arguments()
    if args.max_frames < 0:
        print("Error: --max-frames must be non-negative.", file=sys.stderr)
        return 1

    log_root = os.path.join(os.path.dirname(__file__), "log")
    targets = find_target_csv(log_root)
    if not targets:
        print(f"No target.csv found under {log_root}", file=sys.stderr)
        return 1

    try:
        selected_targets = select_targets(
            targets,
            args.video,
            args.corner_method,
            normalize_methods(args.methods),
        )
    except ValueError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if args.list_armors or not args.armor_name:
        try:
            print_armor_counts(selected_targets, args.max_frames)
        except (OSError, ValueError) as error:
            print(f"Error: {error}", file=sys.stderr)
            return 1
        if not args.armor_name:
            print("Pass --armor-name <name> to generate a comparison plot.")
        return 0

    series_by_method = {}
    try:
        for refine_method, csv_path, _legacy in selected_targets:
            values_by_frame, failed_rows, duplicate_count = read_x_series(
                csv_path,
                args.armor_name,
                args.max_frames,
            )
            if not values_by_frame:
                print(
                    f"Warning: {refine_method}: no usable samples for armor_name={args.armor_name}; skipped.",
                    file=sys.stderr,
                )
                continue

            series_by_method[refine_method] = values_by_frame
            print(
                f"Loaded {refine_method}: {len(values_by_frame)} valid frames "
                f"(failed={failed_rows}, ambiguous={duplicate_count})"
            )
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if not series_by_method:
        print(f"Error: no usable method data for armor_name={args.armor_name}", file=sys.stderr)
        return 1

    filename_components = [args.prefix, args.video, args.corner_method, args.armor_name, "x_comparison"]
    filename = "_".join(safe_filename_component(component) for component in filename_components if component) + ".png"
    output_path = os.path.join(os.path.dirname(__file__), "analy", "picture", "cmp", filename)

    try:
        plot_x_comparison(series_by_method, args.video, args.corner_method, args.armor_name, output_path)
    except (OSError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    print(f"Saved: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
