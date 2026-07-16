#!/usr/bin/env python3
'''
Plot fixed-PnP-line pose landscapes from exported sample directories.

Usage:
    debug/pose_refine/plot_pose_landscape.py <sample_directory>
    debug/pose_refine/plot_pose_landscape.py --all <run_directory>
    debug/pose_refine/plot_pose_landscape.py --latest-run <video_directory>
'''

import csv
import os
import sys


def optional_float(value):
    '''Convert a CSV scalar to float, preserving empty fields as None.'''
    try:
        return float(value) if value else None
    except ValueError:
        return None


def parse_grid_csv(path):
    '''Read grid.csv without importing plotting dependencies.'''
    rows = []
    with open(path, 'r', encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            try:
                rows.append({
                    'distance_index': int(row['distance_index']),
                    'pose_yaw_index': int(row['pose_yaw_index']),
                    'distance_m': float(row['distance_m']),
                    'pose_yaw_deg': float(row['pose_yaw_deg']),
                    'cost': optional_float(row['cost']),
                    'mean_residual_px': optional_float(row['mean_residual_px']),
                    'status': row['status'],
                })
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def parse_markers_csv(path):
    '''Read markers.csv without importing plotting dependencies.'''
    rows = []
    with open(path, 'r', encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            try:
                rows.append({
                    'name': row['name'],
                    'available': row['available'] == 'true',
                    'inside_grid': row['inside_grid'] == 'true',
                    'status': row['status'],
                    'distance_m': optional_float(row['distance_m']),
                    'pose_yaw_deg': optional_float(row['pose_yaw_deg']),
                })
            except KeyError:
                continue
    return rows


def build_grid_data(rows):
    '''Convert sparse CSV rows to cost and E_mean 2D arrays.'''
    import numpy as np

    distances = sorted({row['distance_m'] for row in rows})
    yaw_values = sorted({row['pose_yaw_deg'] for row in rows})
    distance_indices = {value: index for index, value in enumerate(distances)}
    yaw_indices = {value: index for index, value in enumerate(yaw_values)}
    cost = np.full((len(distances), len(yaw_values)), np.nan)
    mean_residual = np.full((len(distances), len(yaw_values)), np.nan)

    for row in rows:
        if row['status'] != 'ok' or row['cost'] is None or row['mean_residual_px'] is None:
            continue
        distance_index = distance_indices[row['distance_m']]
        yaw_index = yaw_indices[row['pose_yaw_deg']]
        cost[distance_index, yaw_index] = row['cost']
        mean_residual[distance_index, yaw_index] = row['mean_residual_px']

    return np.asarray(distances), np.asarray(yaw_values), cost, mean_residual


def add_markers(axis, distances, yaw_values, markers):
    '''Overlay markers and visible diagnostics for unavailable or projected points.'''
    marker_definitions = {
        'pnp': ('o', 'tab:blue', 'PnP'),
        'yaw_search': ('^', 'tab:green', 'Yaw Search'),
        'ba_4dof_ypd': ('s', 'tab:red', 'BA (projected)'),
        'grid_min': ('D', 'tab:purple', 'Grid Min'),
    }
    notes = []
    has_marker = False
    x_min, x_max = distances.min(), distances.max()
    y_min, y_max = yaw_values.min(), yaw_values.max()

    for marker in markers:
        definition = marker_definitions.get(marker['name'])
        if definition is None:
            continue
        style, color, label = definition
        distance_m = marker['distance_m']
        pose_yaw_deg = marker['pose_yaw_deg']
        status = marker['status']
        if not marker['available'] or distance_m is None or pose_yaw_deg is None:
            notes.append(f'{label}: unavailable ({status})')
            continue

        in_bounds = x_min <= distance_m <= x_max and y_min <= pose_yaw_deg <= y_max
        if not marker['inside_grid'] or not in_bounds:
            notes.append(f'{label}: outside grid ({status})')
            continue

        axis.scatter(distance_m, pose_yaw_deg, marker=style, color=color, label=label, s=80, zorder=5)
        has_marker = True

    if notes:
        axis.text(
            0.02,
            0.98,
            '\n'.join(notes),
            transform=axis.transAxes,
            ha='left',
            va='top',
            fontsize=8,
            bbox={'boxstyle': 'round,pad=0.3', 'facecolor': 'white', 'alpha': 0.85},
        )
    if has_marker:
        axis.legend(loc='upper right')


def write_plot(distances, yaw_values, values, title, colorbar_label, markers, output_path):
    '''Write one heatmap/contour plot and overlay all applicable markers.'''
    import matplotlib.pyplot as plt
    import numpy as np

    figure, axis = plt.subplots(figsize=(10, 7))
    masked_values = np.ma.masked_invalid(values)
    if np.any(np.isfinite(values)):
        image = axis.pcolormesh(distances, yaw_values, masked_values.T, cmap='viridis', shading='auto')
        figure.colorbar(image, ax=axis, label=colorbar_label)
        finite_values = values[np.isfinite(values)]
        if len(distances) > 1 and len(yaw_values) > 1 and finite_values.min() < finite_values.max():
            vmin, vmax = finite_values.min(), finite_values.max()
            levels = np.geomspace(max(vmin * 0.5, 1e-10), vmax, 20)
            contours = axis.contour(
                distances,
                yaw_values,
                masked_values.T,
                colors='white',
                linewidths=0.6,
                levels=levels,
            )
            axis.clabel(contours, inline=True, fontsize=7, fmt='%.2g')
    else:
        axis.text(0.5, 0.5, 'No valid grid values', transform=axis.transAxes, ha='center', va='center')

    axis.set_xlabel('distance (m)')
    axis.set_ylabel('pose yaw (deg)')
    axis.set_title(title)
    add_markers(axis, distances, yaw_values, markers)
    figure.tight_layout()
    figure.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(figure)


def plot_sample(sample_dir):
    '''Create the two required PNG files for one sample directory.'''
    grid_path = os.path.join(sample_dir, 'grid.csv')
    markers_path = os.path.join(sample_dir, 'markers.csv')
    for path in (grid_path, markers_path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f'required file not found: {path}')
    grid_rows = parse_grid_csv(grid_path)
    if not grid_rows:
        raise ValueError('no readable rows found in grid.csv')
    markers = parse_markers_csv(markers_path)
    distances, yaw_values, cost, mean_residual = build_grid_data(grid_rows)

    write_plot(
        distances,
        yaw_values,
        cost,
        'BA Robust Cost Landscape',
        'C',
        markers,
        os.path.join(sample_dir, 'cost_landscape.png'),
    )
    write_plot(
        distances,
        yaw_values,
        mean_residual,
        'Mean Residual Landscape',
        r'$E_{\mathrm{mean}}$ (px)',
        markers,
        os.path.join(sample_dir, 'mean_residual_landscape.png'),
    )
    print(f'Plots saved to {sample_dir}/cost_landscape.png')
    print(f'Plots saved to {sample_dir}/mean_residual_landscape.png')


def sample_directories(run_dir):
    '''Return deterministic sample directories directly below one run directory.'''
    return sorted(
        entry.path for entry in os.scandir(run_dir)
        if entry.is_dir() and entry.name.startswith('frame_')
    )


def latest_run_directory(video_dir):
    '''Return the newest completed run directory below one video directory.'''
    candidates = [
        entry for entry in os.scandir(video_dir)
        if entry.is_dir() and os.path.isfile(os.path.join(entry.path, 'index.csv'))
    ]
    if not candidates:
        raise FileNotFoundError(f'no landscape run found below: {video_dir}')
    return max(candidates, key=lambda entry: entry.stat().st_mtime).path


def parse_target(arguments):
    '''Resolve one sample, all samples in a run, or the latest run under a video directory.'''
    if len(arguments) == 1:
        return [arguments[0]]
    if len(arguments) == 2 and arguments[0] == '--all':
        return sample_directories(arguments[1])
    if len(arguments) == 2 and arguments[0] == '--latest-run':
        return sample_directories(latest_run_directory(arguments[1]))
    raise ValueError(
        f'Usage: {os.path.basename(sys.argv[0])} <sample_directory> | '
        '--all <run_directory> | --latest-run <video_directory>'
    )


def main():
    '''Resolve requested samples and create their two PNG files.'''
    try:
        sample_dirs = parse_target(sys.argv[1:])
    except (FileNotFoundError, ValueError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    if not sample_dirs:
        print('Error: no sample directories found', file=sys.stderr)
        return 1

    try:
        import matplotlib
        import numpy  # noqa: F401

        matplotlib.use('Agg')
    except ImportError as error:
        print(f'Error: missing Python dependency: {error}', file=sys.stderr)
        print('Install the offline plotting dependencies with: pip install numpy matplotlib', file=sys.stderr)
        return 1

    failures = 0
    for sample_dir in sample_dirs:
        try:
            plot_sample(sample_dir)
        except (FileNotFoundError, ValueError, OSError) as error:
            failures += 1
            print(f'Error: {sample_dir}: {error}', file=sys.stderr)
    print(f'Generated PNG pairs for {len(sample_dirs) - failures}/{len(sample_dirs)} samples')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
