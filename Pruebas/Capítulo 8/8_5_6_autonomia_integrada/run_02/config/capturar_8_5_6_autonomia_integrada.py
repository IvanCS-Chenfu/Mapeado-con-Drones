#!/usr/bin/env python3
"""Capturador pasivo de la ejecución autónoma integrada 8.5.6."""
import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from mission_msgs.msg import TaskState, TaskStateArray, VoxelMap
from orbslam3_msgs.msg import NavigationState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

STATE_NAMES = {
    TaskState.PENDING: "PENDING", TaskState.ASSIGNED: "ASSIGNED",
    TaskState.RUNNING: "RUNNING", TaskState.PAUSED: "PAUSED",
    TaskState.COMPLETED: "COMPLETED", TaskState.FAILED: "FAILED",
    TaskState.CANCELLED: "CANCELLED", TaskState.BLOCKED: "BLOCKED",
    TaskState.WAITING: "WAITING", TaskState.BLOCKED_BRANCH: "BLOCKED_BRANCH",
    TaskState.SUPERSEDED: "SUPERSEDED", TaskState.TO_FINISH: "TO_FINISH",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=420.0)
    args = parser.parse_args()
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    args.processed_dir.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    running = True
    task_events = []
    map_events = []
    navigation_events = []
    task_revisions = set()
    last_navigation_elapsed = {1: -1.0, 2: -1.0}
    last_persist_elapsed = -5.0

    def elapsed():
        return round(time.monotonic() - start, 3)

    def persist():
        nonlocal last_persist_elapsed
        now = elapsed()
        if now - last_persist_elapsed < 5.0:
            return
        last_persist_elapsed = now
        for name, values in (("task_state_events.json", task_events),
                             ("voxel_map_timeline.json", map_events),
                             ("navigation_timeline.json", navigation_events)):
            (args.raw_dir / name).write_text(
                json.dumps(values, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        assigned = {item["assigned_drone_id"] for item in task_events if item["state"] in ("ASSIGNED", "RUNNING")}
        summary = {
            "capture_duration_sec": now,
            "task_state_event_count": len(task_events),
            "voxel_map_event_count": len(map_events),
            "navigation_event_count": len(navigation_events),
            "assigned_drones": sorted(drone for drone in assigned if drone in (1, 2)),
            "max_reserved_voxels": max((item["reserved_voxels"] for item in map_events), default=0),
            "max_map_revision": max((item["map_revision"] for item in map_events), default=0),
        }
        (args.processed_dir / "capture_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = rclpy.create_node("chapter8_integrated_recorder")
    snapshot_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_tasks(message):
        now = elapsed()
        for task in message.tasks:
            key = (task.task_id, int(task.state_revision))
            if key in task_revisions:
                continue
            task_revisions.add(key)
            task_events.append({
                "elapsed_sec": now,
                "task_id": task.task_id,
                "task_type": task.task_type,
                "region_id": task.region_id,
                "assigned_drone_id": int(task.assigned_drone_id),
                "state": STATE_NAMES.get(int(task.state), str(int(task.state))),
                "state_revision": int(task.state_revision),
                "progress": float(task.progress),
                "progress_known": bool(task.progress_known),
                "detail": task.detail,
            })
        persist()

    def on_map(message):
        map_events.append({
            "elapsed_sec": elapsed(),
            "map_revision": int(message.map_revision),
            "raw_voxels": len(message.voxels),
            "reserved_voxels": len(message.reserved_voxels),
        })
        persist()

    def on_navigation(drone_id, message):
        now = elapsed()
        if now - last_navigation_elapsed[drone_id] < 0.5:
            return
        last_navigation_elapsed[drone_id] = now
        pose = message.w_t_body if message.global_valid else message.o_t_body
        navigation_events.append({
            "elapsed_sec": now,
            "drone_id": drone_id,
            "global_valid": bool(message.global_valid),
            "map_epoch": int(message.map_epoch),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        })
        persist()

    node.create_subscription(TaskStateArray, "/mission/task_states", on_tasks, snapshot_qos)
    node.create_subscription(VoxelMap, "/mission/voxel_map", on_map, snapshot_qos)
    for drone_id in (1, 2):
        node.create_subscription(
            NavigationState, f"/dron_{drone_id}/orbslam/navigation_state",
            lambda message, drone_id=drone_id: on_navigation(drone_id, message),
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
        )

    deadline = start + args.duration_sec
    while running and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    last_persist_elapsed = -5.0
    persist()
    print((args.processed_dir / "capture_summary.json").read_text(encoding="utf-8").strip())


if __name__ == "__main__":
    main()
