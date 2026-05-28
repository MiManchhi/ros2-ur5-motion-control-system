#!/usr/bin/env python3
"""Plot UR5 joint positions from a ROS 2 bag.

Usage:
  source install/setup.bash
  python3 scripts/plot_joint_states_from_bag.py bags/graduation_test_xxx \
    --output docs/progress/fig5-7_joint_positions.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import JointState


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
        description="Plot six UR5 joint angle curves from /joint_states in a ROS 2 bag."
    )
    parser.add_argument("bag_path", help="Path to a ROS 2 bag directory.")
    parser.add_argument(
        "--output",
        default="docs/progress/fig5-7_joint_positions.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--topic",
        default="/joint_states",
        help="JointState topic name in the bag.",
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
        help="Joint names to plot, in display order.",
    )
    parser.add_argument(
        "--title",
        default="UR5 Joint Positions During Normal Motion Test",
        help="Figure title.",
    )
    parser.add_argument(
        "--trim-static",
        action="store_true",
        help="Trim long static segments before and after visible motion.",
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
        help="Minimum max joint delta in rad used to detect motion.",
    )
    return parser.parse_args()


def read_joint_states(
    bag_path: Path,
    topic: str,
    storage_id: str,
    joints: List[str],
) -> Dict[str, List[float]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = "\n".join(sorted(topic_types))
        raise RuntimeError(f"Topic {topic!r} not found in bag. Available topics:\n{available}")
    if topic_types[topic] != "sensor_msgs/msg/JointState":
        raise RuntimeError(
            f"Topic {topic!r} has type {topic_types[topic]!r}, expected sensor_msgs/msg/JointState."
        )

    times: List[float] = []
    positions: Dict[str, List[float]] = {joint: [] for joint in joints}
    start_time_ns = None

    while reader.has_next():
        msg_topic, serialized_msg, timestamp_ns = reader.read_next()
        if msg_topic != topic:
            continue

        msg = deserialize_message(serialized_msg, JointState)
        if start_time_ns is None:
            start_time_ns = timestamp_ns
        times.append((timestamp_ns - start_time_ns) / 1e9)

        name_to_position = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        for joint in joints:
            positions[joint].append(name_to_position.get(joint, float("nan")))

    if not times:
        raise RuntimeError(f"No messages found on topic {topic!r}.")

    positions["__time__"] = times
    return positions


def plot_joint_positions(data: Dict[str, List[float]], joints: List[str], output: Path, title: str) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    plt.figure(figsize=(11, 6), dpi=160)
    times = data["__time__"]

    for joint in joints:
        plt.plot(times, data[joint], linewidth=1.8, label=joint)

    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Joint position (rad)")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(output)


def trim_static_segments(
    data: Dict[str, List[float]],
    joints: List[str],
    padding_sec: float,
    motion_threshold: float,
) -> Dict[str, List[float]]:
    times = data["__time__"]
    if len(times) < 2:
        return data

    motion_indexes: List[int] = []
    for index in range(1, len(times)):
      max_delta = 0.0
      for joint in joints:
          prev_value = data[joint][index - 1]
          curr_value = data[joint][index]
          if math.isnan(prev_value) or math.isnan(curr_value):
              continue
          max_delta = max(max_delta, abs(curr_value - prev_value))
      if max_delta >= motion_threshold:
          motion_indexes.append(index)

    if not motion_indexes:
        return data

    start_time = max(0.0, times[motion_indexes[0]] - padding_sec)
    end_time = times[motion_indexes[-1]] + padding_sec
    kept_indexes = [
        index for index, stamp in enumerate(times)
        if start_time <= stamp <= end_time
    ]
    if not kept_indexes:
        return data

    trimmed: Dict[str, List[float]] = {}
    base_time = times[kept_indexes[0]]
    trimmed["__time__"] = [times[index] - base_time for index in kept_indexes]
    for joint in joints:
        trimmed[joint] = [data[joint][index] for index in kept_indexes]
    return trimmed


def print_summary(data: Dict[str, List[float]], joints: List[str]) -> None:
    print("Joint summary:")
    for joint in joints:
        values = [value for value in data[joint] if not math.isnan(value)]
        if not values:
            print(f"  {joint}: no data")
            continue
        print(
            f"  {joint}: start={values[0]: .6f}, end={values[-1]: .6f}, "
            f"delta={values[-1] - values[0]: .6f}"
        )


def main() -> None:
    args = parse_args()
    bag_path = Path(args.bag_path)
    output = Path(args.output)

    if not bag_path.exists():
        raise SystemExit(f"Bag path does not exist: {bag_path}")

    data = read_joint_states(bag_path, args.topic, args.storage_id, args.joints)
    if args.trim_static:
        data = trim_static_segments(data, args.joints, args.padding_sec, args.motion_threshold)
    print_summary(data, args.joints)
    plot_joint_positions(data, args.joints, output, args.title)
    print(f"Saved figure: {output}")


if __name__ == "__main__":
    main()
