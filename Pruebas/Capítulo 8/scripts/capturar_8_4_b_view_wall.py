#!/usr/bin/env python3
"""Capturador pasivo de tarea, coverage U y VoxelMap para 8.4-B."""
import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from mission_msgs.msg import TaskStateArray, VoxelCell, VoxelMap
from orbslam3_msgs.msg import NavigationState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def task_dict(task):
    return {
        "task_id": task.task_id,
        "task_type": task.task_type,
        "region_id": task.region_id,
        "assigned_drone_id": int(task.assigned_drone_id),
        "state": int(task.state),
        "state_revision": int(task.state_revision),
        "progress": float(task.progress),
        "progress_known": bool(task.progress_known),
        "coverage_intervals": [{
            "region_id": item.region_id,
            "start_ratio": float(item.start_ratio),
            "end_ratio": float(item.end_ratio),
        } for item in task.coverage_intervals],
        "detail": task.detail,
    }


def map_counts(message):
    return {
        "map_revision": int(message.map_revision),
        "map_stamp_sec": float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9,
        "unknown": sum(cell.state == VoxelCell.UNKNOWN for cell in message.voxels),
        "free": sum(cell.state == VoxelCell.FREE for cell in message.voxels),
        "occupied": sum(cell.state == VoxelCell.OCCUPIED for cell in message.voxels),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=300.0)
    args = parser.parse_args()
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    args.processed_dir.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    running = True
    latest_tasks = []
    task_events = []
    voxel_events = []
    navigation_events = []
    seen_task_revisions = set()

    def elapsed():
        return round(time.monotonic() - start, 3)

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = rclpy.create_node("chapter8_view_wall_recorder")
    snapshot_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_tasks(message):
        nonlocal latest_tasks
        latest_tasks = [task_dict(task) for task in message.tasks]
        for task in latest_tasks:
            key = (task["task_id"], task["state_revision"])
            if key in seen_task_revisions:
                continue
            seen_task_revisions.add(key)
            task_events.append({"elapsed_sec": elapsed(), **task})

    def on_voxels(message):
        voxel_events.append({"elapsed_sec": elapsed(), **map_counts(message), "tasks": latest_tasks})

    def on_navigation(message):
        pose = message.w_t_body if message.global_valid else message.o_t_body
        navigation_events.append({
            "elapsed_sec": elapsed(),
            "global_valid": bool(message.global_valid),
            "x": float(pose.position.x), "y": float(pose.position.y), "z": float(pose.position.z),
        })

    node.create_subscription(TaskStateArray, "/mission/task_states", on_tasks, snapshot_qos)
    node.create_subscription(VoxelMap, "/mission/voxel_map", on_voxels, snapshot_qos)
    node.create_subscription(
        NavigationState, "/dron_1/orbslam/navigation_state", on_navigation,
        QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
    )
    deadline = start + args.duration_sec
    while running and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    for name, data in (("task_state_events.json", task_events),
                       ("voxel_map_events.json", voxel_events),
                       ("navigation_events.json", navigation_events)):
        (args.raw_dir / name).write_text(
            json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    summary = {
        "capture_duration_sec": elapsed(),
        "task_state_event_count": len(task_events),
        "voxel_map_event_count": len(voxel_events),
        "navigation_event_count": len(navigation_events),
    }
    (args.processed_dir / "capture_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
