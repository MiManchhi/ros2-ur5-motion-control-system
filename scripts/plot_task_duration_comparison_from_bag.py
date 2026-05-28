#!/usr/bin/env python3
"""Plot planned trajectory duration comparison from one or more ROS 2 bags.

The script reads:
  - /motion_command for task_id and speed_scale
  - /planned_traj for planned trajectory duration
  - /task_state for final state fallback

Usage:
  source install/setup.bash
  python3 scripts/plot_task_duration_comparison_from_bag.py \
    bags/graduation_test_xxx \
    --output docs/progress/fig5-9_speed_scale_duration.png
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from builtin_interfaces.msg import Duration
from robot_motion_msgs.msg import MotionCommand, PlannedTrajectory, TaskState


TERMINAL_STATES = {"completed", "failed", "canceled", "rejected"}


@dataclass
class TaskRecord:
    task_id: str
    speed_scale: Optional[float] = None
    first_state: Optional[str] = None
    final_state: Optional[str] = None
    start_time_ns: Optional[int] = None
    end_time_ns: Optional[int] = None
    planned_duration_sec: Optional[float] = None

    @property
    def duration_sec(self) -> Optional[float]:
        if self.planned_duration_sec is not None:
            return self.planned_duration_sec
        if self.start_time_ns is None or self.end_time_ns is None:
            return None
        return (self.end_time_ns - self.start_time_ns) / 1e9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot planned trajectory duration comparison by speed_scale from ROS 2 bags."
    )
    parser.add_argument("bag_paths", nargs="+", help="One or more ROS 2 bag directories.")
    parser.add_argument(
        "--output",
        default="docs/progress/fig5-9_speed_scale_duration.png",
        help="Output PNG path.",
    )
    parser.add_argument(
        "--motion-command-topic",
        default="/motion_command",
        help="MotionCommand topic used to read speed_scale.",
    )
    parser.add_argument(
        "--planned-traj-topic",
        default="/planned_traj",
        help="Fallback PlannedTrajectory topic used to infer speed_scale.",
    )
    parser.add_argument(
        "--task-state-topic",
        default="/task_state",
        help="TaskState topic used to compute duration.",
    )
    parser.add_argument(
        "--storage-id",
        default="sqlite3",
        help="rosbag2 storage id, usually sqlite3 for ROS 2 Humble bags.",
    )
    parser.add_argument(
        "--title",
        default="Planned Trajectory Duration Under Different speed_scale",
        help="Figure title.",
    )
    parser.add_argument(
        "--base-plan-duration",
        type=float,
        default=5.0,
        help="planner_node plan_duration_sec used to infer speed_scale from trajectory duration.",
    )
    return parser.parse_args()


def open_reader(bag_path: Path, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def get_or_create(records: Dict[str, TaskRecord], task_id: str) -> TaskRecord:
    if task_id not in records:
        records[task_id] = TaskRecord(task_id=task_id)
    return records[task_id]


def duration_to_sec(duration: Duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def read_records_from_bag(
    bag_path: Path,
    storage_id: str,
    motion_command_topic: str,
    planned_traj_topic: str,
    task_state_topic: str,
    base_plan_duration: float,
) -> List[TaskRecord]:
    reader = open_reader(bag_path, storage_id)
    records: Dict[str, TaskRecord] = {}

    while reader.has_next():
        topic, serialized_msg, timestamp_ns = reader.read_next()

        if topic == motion_command_topic:
            msg = deserialize_message(serialized_msg, MotionCommand)
            if not msg.task_id:
                continue
            record = get_or_create(records, msg.task_id)
            if record.speed_scale is None:
                record.speed_scale = msg.speed_scale

        elif topic == planned_traj_topic:
            msg = deserialize_message(serialized_msg, PlannedTrajectory)
            if not msg.task_id or not msg.trajectory.points:
                continue
            final_time_sec = duration_to_sec(msg.trajectory.points[-1].time_from_start)
            if final_time_sec <= 0.0:
                continue
            record = get_or_create(records, msg.task_id)
            record.planned_duration_sec = final_time_sec
            if record.speed_scale is None:
                # SimpleJointPlanner uses actual_duration = plan_duration_sec / speed_scale.
                record.speed_scale = base_plan_duration / final_time_sec

        elif topic == task_state_topic:
            msg = deserialize_message(serialized_msg, TaskState)
            if not msg.task_id:
                continue
            record = get_or_create(records, msg.task_id)
            if record.start_time_ns is None:
                record.start_time_ns = timestamp_ns
                record.first_state = msg.state
            if msg.state in TERMINAL_STATES or msg.is_terminal:
                record.end_time_ns = timestamp_ns
                record.final_state = msg.state

    return [
        record
        for record in records.values()
        if record.speed_scale is not None and record.duration_sec is not None
    ]


def read_all_records(args: argparse.Namespace) -> List[TaskRecord]:
    records: List[TaskRecord] = []
    for bag_path_text in args.bag_paths:
        bag_path = Path(bag_path_text)
        if not bag_path.exists():
            raise RuntimeError(f"Bag path does not exist: {bag_path}")
        records.extend(
            read_records_from_bag(
                bag_path,
                args.storage_id,
                args.motion_command_topic,
                args.planned_traj_topic,
                args.task_state_topic,
                args.base_plan_duration,
            )
        )
    records.sort(key=lambda item: (item.speed_scale if item.speed_scale is not None else 999.0, item.task_id))
    return records


def plot(records: Iterable[TaskRecord], output: Path, title: str) -> None:
    records = list(records)
    if not records:
        raise RuntimeError("No completed task records found. Check /motion_command and /task_state in the bag.")

    labels = [
        f"speed={record.speed_scale:g}\n{record.final_state}"
        for record in records
        if record.speed_scale is not None
    ]
    durations = [record.duration_sec for record in records if record.duration_sec is not None]

    output.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(9, 5.5), dpi=160)
    bars = plt.bar(labels, durations, color=["#4C78A8", "#F58518", "#54A24B", "#B279A2"][: len(labels)])

    for bar, duration in zip(bars, durations):
        plt.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{duration:.2f}s",
            ha="center",
            va="bottom",
            fontsize=9,
        )

    plt.title(title)
    plt.xlabel("speed_scale")
    plt.ylabel("Planned trajectory duration (s)")
    plt.grid(True, axis="y", linestyle="--", linewidth=0.5, alpha=0.6)
    plt.tight_layout()
    plt.savefig(output)


def print_summary(records: Iterable[TaskRecord]) -> None:
    print("Planned trajectory duration summary:")
    for record in records:
        print(
            f"  task_id={record.task_id}, speed_scale={record.speed_scale:g}, "
            f"duration={record.duration_sec:.3f}s, final_state={record.final_state}"
        )


def main() -> None:
    args = parse_args()
    records = read_all_records(args)
    print_summary(records)
    plot(records, Path(args.output), args.title)
    print(f"Saved figure: {args.output}")


if __name__ == "__main__":
    main()
