#!/usr/bin/env python3
"""Capturador pasivo A/B para la prueba 8.4-A VIEW_UNKNOWN."""
import argparse
import csv
import json
import signal
import time
from pathlib import Path

import rclpy
from mission_msgs.msg import TaskStateArray, VoxelCell, VoxelMap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def cell_dict(cell):
    return {
        "ix": int(cell.ix),
        "iy": int(cell.iy),
        "iz": int(cell.iz),
        "state": int(cell.state),
        "score": float(cell.score),
    }


def task_dict(task):
    return {
        "task_id": task.task_id,
        "region_id": task.region_id,
        "assigned_drone_id": int(task.assigned_drone_id),
        "state": int(task.state),
        "state_revision": int(task.state_revision),
        "progress": float(task.progress),
        "progress_known": bool(task.progress_known),
        "coverage_intervals": [
            {"start_ratio": float(item.start_ratio), "end_ratio": float(item.end_ratio)}
            for item in task.coverage_intervals
        ],
        "detail": task.detail,
    }


def metrics(snapshot):
    voxels = snapshot["voxels"]
    return {
        "map_revision": snapshot["map_revision"],
        "num_UNKNOWN": sum(item["state"] == VoxelCell.UNKNOWN for item in voxels),
        "num_FREE": sum(item["state"] == VoxelCell.FREE for item in voxels),
        "num_OCCUPIED": sum(item["state"] == VoxelCell.OCCUPIED for item in voxels),
        "coverage_progress": [
            {"task_id": item["task_id"], "progress": item["progress"],
             "intervals": item["coverage_intervals"]}
            for item in snapshot["tasks"]
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=150.0)
    args = parser.parse_args()
    for directory in (args.raw_dir, args.processed_dir):
        directory.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    latest_map = None
    latest_tasks = []
    before = None
    after = None
    unknown_target_seen = False
    task_events = []
    map_events = []
    running = True

    def elapsed():
        return round(time.monotonic() - start, 3)

    def store(message):
        return {
            "capture_elapsed_sec": elapsed(),
            "map_stamp_sec": float(message.header.stamp.sec) +
                              float(message.header.stamp.nanosec) * 1.0e-9,
            "map_revision": int(message.map_revision),
            "voxel_size": float(message.voxel_size),
            "voxels": [cell_dict(cell) for cell in message.voxels],
            "tasks": latest_tasks,
        }

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = rclpy.create_node("chapter8_view_unknown_recorder")
    snapshot_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_tasks(message):
        nonlocal latest_tasks, unknown_target_seen, before
        latest_tasks = [task_dict(task) for task in message.tasks]
        task_events.append({"elapsed_sec": elapsed(), "tasks": latest_tasks})
        if not unknown_target_seen and any(
                "Pose de inspeccion UNKNOWN; solicita mirada depth" in task["detail"]
                for task in latest_tasks):
            unknown_target_seen = True
            if latest_map is not None and before is None:
                before = store(latest_map)

    def on_voxels(message):
        nonlocal latest_map, before, after
        latest_map = message
        current = store(message)
        current_metrics = metrics(current)
        map_events.append({"elapsed_sec": current["capture_elapsed_sec"], **current_metrics})
        if unknown_target_seen and before is None:
            before = current
        if before is not None and after is None:
            before_metrics = metrics(before)
            if (current["map_revision"] > before["map_revision"] and
                    current_metrics["num_FREE"] > before_metrics["num_FREE"]):
                after = current

    node.create_subscription(TaskStateArray, "/mission/task_states", on_tasks, snapshot_qos)
    node.create_subscription(VoxelMap, "/mission/voxel_map", on_voxels, snapshot_qos)

    deadline = start + args.duration_sec
    while running and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    node.destroy_node()
    rclpy.shutdown()
    if before is not None:
        (args.raw_dir / "before_view_unknown.json").write_text(
            json.dumps(before, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if after is not None:
        (args.raw_dir / "after_view_unknown.json").write_text(
            json.dumps(after, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (args.raw_dir / "task_events.json").write_text(
        json.dumps(task_events, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (args.raw_dir / "map_events.json").write_text(
        json.dumps(map_events, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    rows = []
    for label, snapshot in (("before", before), ("after", after)):
        if snapshot is not None:
            rows.append({"snapshot": label, **metrics(snapshot)})
    with (args.processed_dir / "view_unknown_metrics.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        fields = ("snapshot", "map_revision", "num_UNKNOWN", "num_FREE", "num_OCCUPIED",
                  "coverage_progress")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            row = dict(row)
            row["coverage_progress"] = json.dumps(row["coverage_progress"], ensure_ascii=False)
            writer.writerow(row)

    summary = {
        "unknown_target_seen": unknown_target_seen,
        "before_captured": before is not None,
        "after_captured": after is not None,
        "snapshots": rows,
        "capture_duration_sec": elapsed(),
    }
    if len(rows) == 2:
        summary["free_delta"] = rows[1]["num_FREE"] - rows[0]["num_FREE"]
        summary["occupied_delta"] = rows[1]["num_OCCUPIED"] - rows[0]["num_OCCUPIED"]
        summary["coverage_unchanged"] = (
            rows[0]["coverage_progress"] == rows[1]["coverage_progress"])
    (args.processed_dir / "view_unknown_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
