#!/usr/bin/env python3
"""Capturador pasivo temporal para la prueba 8.4-A VIEW_UNKNOWN."""
import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

import rclpy
from mission_msgs.msg import TaskStateArray, VoxelCell, VoxelMap
from orbslam3_msgs.msg import NavigationState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def cell_dict(cell):
    return {"ix": int(cell.ix), "iy": int(cell.iy), "iz": int(cell.iz),
            "state": int(cell.state), "score": float(cell.score)}


def task_dict(task):
    return {
        "task_id": task.task_id, "region_id": task.region_id,
        "assigned_drone_id": int(task.assigned_drone_id), "state": int(task.state),
        "state_revision": int(task.state_revision), "progress": float(task.progress),
        "progress_known": bool(task.progress_known),
        "coverage_intervals": [{"start_ratio": float(item.start_ratio),
                                "end_ratio": float(item.end_ratio)}
                               for item in task.coverage_intervals],
        "detail": task.detail,
    }


def map_snapshot(message, tasks, elapsed_sec):
    return {
        "capture_elapsed_sec": elapsed_sec,
        "map_stamp_sec": float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9,
        "map_revision": int(message.map_revision), "voxel_size": float(message.voxel_size),
        "voxels": [cell_dict(cell) for cell in message.voxels], "tasks": tasks,
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
             "intervals": item["coverage_intervals"]} for item in snapshot["tasks"]],
    }


def yaw_deg(orientation):
    return math.degrees(math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=150.0)
    parser.add_argument("--history-window-sec", type=float, default=15.0)
    args = parser.parse_args()
    for directory in (args.raw_dir, args.processed_dir, args.raw_dir / "map_history"):
        directory.mkdir(parents=True, exist_ok=True)

    start, latest_map, latest_tasks = time.monotonic(), None, []
    before, unknown_since, unknown_target_seen, running = None, None, False, True
    task_events, map_events, navigation_events, history_index = [], [], [], 0

    def elapsed():
        return round(time.monotonic() - start, 3)

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = rclpy.create_node("chapter8_view_unknown_recorder")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_tasks(message):
        nonlocal latest_tasks, unknown_target_seen, unknown_since, before
        latest_tasks = [task_dict(task) for task in message.tasks]
        task_events.append({"elapsed_sec": elapsed(), "tasks": latest_tasks})
        if not unknown_target_seen and any(
                "Pose de inspeccion UNKNOWN; solicita mirada depth" in task["detail"]
                for task in latest_tasks):
            unknown_target_seen, unknown_since = True, elapsed()
            if latest_map is not None:
                before = map_snapshot(latest_map, latest_tasks, unknown_since)

    def on_voxels(message):
        nonlocal latest_map, before, history_index
        latest_map = message
        now = elapsed()
        current = map_snapshot(message, latest_tasks, now)
        map_events.append({"elapsed_sec": now, "map_stamp_sec": current["map_stamp_sec"], **metrics(current)})
        if unknown_target_seen and before is None:
            before = current
        if unknown_since is not None and now <= unknown_since + args.history_window_sec:
            history_index += 1
            (args.raw_dir / "map_history" / f"map_{history_index:03d}.json").write_text(
                json.dumps(current, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def on_navigation(message):
        orientation = message.w_t_body.orientation if message.global_valid else message.o_t_body.orientation
        navigation_events.append({
            "elapsed_sec": elapsed(),
            "stamp_sec": float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9,
            "global_valid": bool(message.global_valid), "yaw_deg": yaw_deg(orientation),
        })

    node.create_subscription(TaskStateArray, "/mission/task_states", on_tasks, qos)
    node.create_subscription(VoxelMap, "/mission/voxel_map", on_voxels, qos)
    node.create_subscription(NavigationState, "/dron_1/orbslam/navigation_state", on_navigation,
                             QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
    deadline = start + args.duration_sec
    while running and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if before is not None:
        (args.raw_dir / "before_view_unknown.json").write_text(
            json.dumps(before, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for name, data in (("task_events.json", task_events), ("map_events.json", map_events),
                       ("navigation_events.json", navigation_events)):
        (args.raw_dir / name).write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    summary = {"unknown_target_seen": unknown_target_seen, "before_captured": before is not None,
               "map_history_count": history_index, "capture_duration_sec": elapsed()}
    (args.processed_dir / "capture_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (args.processed_dir / "capture_status.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(summary)); writer.writeheader(); writer.writerow(summary)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
