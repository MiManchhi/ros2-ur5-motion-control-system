#!/usr/bin/env python3
"""Plot max joint error from a ROS 2 bag.

The script reads the target joint positions from /planned_traj by default,
then computes max(abs(q_actual - q_target)) for each /joint_states sample.

Usage:
  source install/setup.bash
  python3 scripts/plot_max_joint_error_from_bag.py bags/graduation_test_xxx \
    --trim-static \
    --output docs/progress/fig5-8_max_joint_error.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import JointState

from robot_motion_msgs.msg import MotionCommand, PlannedTrajectory


DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot max joint error curve from /joint_states in a ROS 2 bag."
    )
    parser.add_argument("bag_path", help="Path to a ROS 2 bag directory.")
    parser.add_argument(
        "--output",
        default="docs/progress/fig5-8_max_joint_error.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--joint-state-topic",
        default="/joint_states",
        help="JointState topic name in the bag.",
    )
    parser.add_argument(
        "--planned-traj-topic",
        default="/planned_traj",
        help="PlannedTrajectory topic used to read the final target.",
    )
    parser.add_argument(
        "--fallback-command-topic",
        default="/joint_cmd",
        help="Fallback MotionCommand topic used when /planned_traj is unavailable.",
    )
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage id, usually sqlite3 for ROS 2 Humble bags.",
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        default=DEFAULT_JOINTS,
        help="Joint names used to compute max error, in display order.",
    )
    parser.add_argument(
        "--goal-tolerance",
        type=float,
        default=0.01,
        help="Goal tolerance threshold shown as a horizontal line.",
    )
    parser.add_argument(
        "--title",
        default="Max Joint Error During Normal Motion Test",
        help="Figure title.",
    )
    parser.add_argument(
        "--trim-static",
        action="store_true",
        help="Trim long static segments before and after visible error change.",
    )
    parser.add_argument(
        "--padding-sec",
        type=float,
        default=1.0,
        help="Padding seconds kept before and after motion when --trim-static is used.",
    )
    parser.add_argument(
        "--motion-threshold",
        type=float,
        default=1e-3,
        help="Minimum max joint delta in rad used to detect motion for trimming.",
    )
    return parser.parse_args()


def open_reader(bag_path: Path, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def read_target_from_bag(
    bag_path: Path,
    storage_id: str,
    planned_traj_topic: str,
    fallback_command_topic: str,
) -> Tuple[List[str], List[float], str]:
    reader = open_reader(bag_path, storage_id)
    target_names: Optional[List[str]] = None
    target_positions: Optional[List[float]] = None
    source = ""

    fallback_names: Optional[List[str]] = None
    fallback_positions: Optional[List[float]] = None

    while reader.has_next():
        topic, serialized_msg, _ = reader.read_next()

        if topic == planned_traj_topic:
            msg = deserialize_message(serialized_msg, PlannedTrajectory)
            if msg.trajectory.joint_names and msg.trajectory.points:
                final_point = msg.trajectory.points[-1]
                if len(final_point.positions) == len(msg.trajectory.joint_names):
                    target_names = list(msg.trajectory.joint_names)
                    target_positions = list(final_point.positions)
                    source = planned_traj_topic

        elif topic == fallback_command_topic:
            msg = deserialize_message(serialized_msg, MotionCommand)
            if msg.joint_names and len(msg.joint_names) == len(msg.positions):
                fallback_names = list(msg.joint_names)
                fallback_positions = list(msg.positions)

    if target_names is not None and target_positions is not None:
        return target_names, target_positions, source

    if fallback_names is not None and fallback_positions is not None:
        return fallback_names, fallback_positions, fallback_command_topic

    raise RuntimeError(
        f"No target trajectory found. Checked {planned_traj_topic!r} and "
        f"{fallback_command_topic!r}."
    )


def read_error_curve(
    bag_path: Path,
    storage_id: str,
    joint_state_topic: str,
    joints: List[str],
    target_names: List[str],
    target_positions: List[float],
) -> Dict[str, List[float]]:
    target_map = dict(zip(target_names, target_positions))
    missing_targets = [joint for joint in joints if joint not in target_map]
    if missing_targets:
        raise RuntimeError(f"Target positions missing joints: {missing_targets}")

    reader = open_reader(bag_path, storage_id)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if joint_state_topic not in topic_types:
        available = "\n".join(sorted(topic_types))
        raise RuntimeError(
            f"Topic {joint_state_topic!r} not found in bag. Available topics:\n{available}"
        )

    times: List[float] = []
    max_errors: List[float] = []
    start_time_ns = None

    while reader.has_next():
        topic, serialized_msg, timestamp_ns = reader.read_next()
        if topic != joint_state_topic:
            continue

        msg = deserialize_message(serialized_msg, JointState)
        name_to_position = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        if any(joint not in name_to_position for joint in joints):
            continue

        if start_time_ns is None:
            start_time_ns = timestamp_ns
        times.append((timestamp_ns - start_time_ns) / 1e9)
        max_errors.append(
            max(abs(name_to_position[joint] - target_map[joint]) for joint in joints)
        )

    if not times:
        raise RuntimeError(f"No usable JointState messages found on {joint_state_topic!r}.")

    return {"time": times, "max_error": max_errors}


def trim_static_segments(
    data: Dict[str, List[float]],
    padding_sec: float,
    motion_threshold: float,
) -> Dict[str, List[float]]:
    times = data["time"]
    errors = data["max_error"]
    if len(times) < 2:
        return data

    changing_indexes = [
        index
        for index in range(1, len(times))
        if abs(errors[index] - errors[index - 1]) >= motion_threshold
    ]
    if not changing_indexes:
        return data

    start_time = max(0.0, times[changing_indexes[0]] - padding_sec)
    end_time = times[changing_indexes[-1]] + padding_sec
    kept_indexes = [
        index for index, stamp in enumerate(times)
        if start_time <= stamp <= end_time
    ]
    if not kept_indexes:
        return data

    base_time = times[kept_indexes[0]]
    return {
        "time": [times[index] - base_time for index in kept_indexes],
        "max_error": [errors[index] for index in kept_indexes],
    }


def plot_error_curve(
    data: Dict[str, List[float]],
    output: Path,
    title: str,
    goal_tolerance: float,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(11, 6), dpi=160)
    plt.plot(data["time"], data["max_error"], linewidth=2.0, label="max joint error")
    plt.axhline(
        goal_tolerance,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"goal_tolerance = {goal_tolerance:g} rad",
    )
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Max joint error (rad)")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    plt.legend(loc="best", fontsize=9)
    plt.tight_layout()
    plt.savefig(output)


def print_summary(data: Dict[str, List[float]], target_source: str) -> None:
    errors = [value for value in data["max_error"] if not math.isnan(value)]
    print(f"Target source: {target_source}")
    print(f"Samples: {len(errors)}")
    print(f"Initial max error: {errors[0]:.6f} rad")
    print(f"Final max error:   {errors[-1]:.6f} rad")
    print(f"Minimum max error: {min(errors):.6f} rad")
    print(f"Maximum max error: {max(errors):.6f} rad")


def main() -> None:
    args = parse_args()
    bag_path = Path(args.bag_path)
    output = Path(args.output)

    if not bag_path.exists():
        raise SystemExit(f"Bag path does not exist: {bag_path}")

    target_names, target_positions, target_source = read_target_from_bag(
        bag_path,
        args.storage_id,
        args.planned_traj_topic,
        args.fallback_command_topic,
    )
    data = read_error_curve(
        bag_path,
        args.storage_id,
        args.joint_state_topic,
        args.joints,
        target_names,
        target_positions,
    )
    if args.trim_static:
        data = trim_static_segments(data, args.padding_sec, args.motion_threshold)

    print_summary(data, target_source)
    plot_error_curve(data, output, args.title, args.goal_tolerance)
    print(f"Saved figure: {output}")


if __name__ == "__main__":
    main()
