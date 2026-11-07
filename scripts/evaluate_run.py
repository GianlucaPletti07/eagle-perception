#!/usr/bin/env python3
"""Summarise one run of the LiDAR odometry node.

Reads the CSV files written by
    ros2 topic echo /odometry --csv > odometry.csv
    ros2 topic echo /lidar_odometry/processing_time --csv > timing.csv
and prints the processing time statistics and how well the end of the lap
matches the first pass over the same stretch of track. With --plot it also
saves the trajectory and the per-scan timing as an image.
"""

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

SCAN_PERIOD_MS = 100.0

# Column positions in the CSV output of `ros2 topic echo --csv`.
ODOM_SEC, ODOM_NANOSEC, ODOM_X, ODOM_Y, ODOM_QZ, ODOM_QW = 0, 1, 4, 5, 9, 10
TIMING_SEC, TIMING_NANOSEC = 0, 1
TIMING_PREPROCESSING, TIMING_REGISTRATION, TIMING_TOTAL = 3, 4, 5


@dataclass
class Pose:
    time: float
    x: float
    y: float
    yaw_deg: float


def read_rows(path):
    with open(path, newline='') as file:
        return [row for row in csv.reader(file) if row]


def stamp(row, sec_column, nanosec_column):
    return int(row[sec_column]) + int(row[nanosec_column]) * 1e-9


def read_poses(path):
    rows = read_rows(path)
    start = stamp(rows[0], ODOM_SEC, ODOM_NANOSEC)
    poses = []
    for row in rows:
        yaw = 2.0 * math.atan2(float(row[ODOM_QZ]), float(row[ODOM_QW]))
        poses.append(Pose(
            time=stamp(row, ODOM_SEC, ODOM_NANOSEC) - start,
            x=float(row[ODOM_X]),
            y=float(row[ODOM_Y]),
            yaw_deg=math.degrees(yaw)))
    return poses


def read_timing(path):
    rows = read_rows(path)
    start = stamp(rows[0], TIMING_SEC, TIMING_NANOSEC)
    return {
        'time': [stamp(row, TIMING_SEC, TIMING_NANOSEC) - start for row in rows],
        'preprocessing': [float(row[TIMING_PREPROCESSING]) for row in rows],
        'registration': [float(row[TIMING_REGISTRATION]) for row in rows],
        'total': [float(row[TIMING_TOTAL]) for row in rows],
    }


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))
    return ordered[index]


def wrap_degrees(angle):
    return (angle + 180.0) % 360.0 - 180.0


def lap_closure(poses):
    """Distance and heading difference between the final pose and the closest pose of the
    first half of the run, i.e. the first pass over the stretch where the lap ends."""
    end = poses[-1]
    first_half = poses[:len(poses) // 2]
    closest = min(first_half, key=lambda pose: math.hypot(pose.x - end.x, pose.y - end.y))
    distance = math.hypot(closest.x - end.x, closest.y - end.y)
    return distance, wrap_degrees(end.yaw_deg - closest.yaw_deg), closest.time


def path_length(poses):
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))


def print_summary(poses, timing):
    print(f'Odometry messages: {len(poses)}   timing messages: {len(timing["total"])}')
    print(f'Path length: {path_length(poses):.1f} m')
    end = poses[-1]
    print(f'Final pose: x = {end.x:.2f} m, y = {end.y:.2f} m, yaw = {end.yaw_deg:.2f} deg')
    distance, heading, time = lap_closure(poses)
    print(f'Lap closure: {distance:.2f} m and {heading:+.2f} deg from the first pass (t = {time:.1f} s)')
    print()
    print(f'{"stage [ms]":<15}{"mean":>8}{"median":>8}{"p95":>8}{"max":>8}')
    for stage in ('preprocessing', 'registration', 'total'):
        values = timing[stage]
        print(f'{stage:<15}{statistics.mean(values):>8.1f}{statistics.median(values):>8.1f}'
              f'{percentile(values, 0.95):>8.1f}{max(values):>8.1f}')
    over_budget = sum(value > SCAN_PERIOD_MS for value in timing['total'])
    print(f'Scans over the {SCAN_PERIOD_MS:.0f} ms budget: {over_budget} of {len(timing["total"])}')


def save_plot(poses, timing, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, (trajectory_axes, timing_axes) = plt.subplots(
        1, 2, figsize=(13, 5), gridspec_kw={'width_ratios': [1, 1.2]})

    trajectory_axes.plot([p.x for p in poses], [p.y for p in poses], color='tab:blue')
    trajectory_axes.plot(poses[0].x, poses[0].y, 'o', color='tab:green', label='start')
    trajectory_axes.plot(poses[-1].x, poses[-1].y, 's', color='tab:red', label='end')
    trajectory_axes.set_aspect('equal')
    trajectory_axes.set_xlabel('x [m]')
    trajectory_axes.set_ylabel('y [m]')
    trajectory_axes.set_title('Estimated trajectory in odom')
    trajectory_axes.grid(alpha=0.3)
    trajectory_axes.legend()

    timing_axes.plot(timing['time'], timing['total'], color='tab:blue', label='total')
    timing_axes.plot(timing['time'], timing['preprocessing'], color='tab:gray', label='preprocessing')
    timing_axes.axhline(SCAN_PERIOD_MS, color='tab:red', linestyle='--', label='scan period')
    timing_axes.set_xlabel('time since first scan [s]')
    timing_axes.set_ylabel('processing time per scan [ms]')
    timing_axes.set_title('Processing time')
    timing_axes.grid(alpha=0.3)
    timing_axes.legend()

    figure.tight_layout()
    figure.savefig(output, dpi=120)
    print(f'\nPlot saved to {output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('odometry_csv', type=Path)
    parser.add_argument('timing_csv', type=Path)
    parser.add_argument('--plot', type=Path, help='save trajectory and timing to this image')
    args = parser.parse_args()

    poses = read_poses(args.odometry_csv)
    timing = read_timing(args.timing_csv)
    print_summary(poses, timing)
    if args.plot:
        save_plot(poses, timing, args.plot)


if __name__ == '__main__':
    main()
