#!/usr/bin/env python3
"""Captura pasiva de autoridad global y asignacion multidron para 8.2-C."""
import argparse
import csv
import json
import signal
import time
from pathlib import Path

import rclpy
from mission_msgs.msg import TaskState, TaskStateArray
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
SOURCE_NAMES = {
    NavigationState.POSE_SOURCE_INVALID: "INVALID",
    NavigationState.POSE_SOURCE_ORB: "ORB",
    NavigationState.POSE_SOURCE_GLOBAL: "GLOBAL",
    NavigationState.POSE_SOURCE_GT_FALLBACK: "GT_FALLBACK",
    NavigationState.POSE_SOURCE_GT_FORCED: "GT_FORCED",
}
STATUS_NAMES = {
    NavigationState.GLOBAL_STATUS_INVALID: "INVALID",
    NavigationState.GLOBAL_STATUS_PROVISIONAL: "PROVISIONAL",
    NavigationState.GLOBAL_STATUS_AUTHORITATIVE: "AUTHORITATIVE",
}


def elapsed(start):
    return round(time.monotonic() - start, 3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=240.0)
    args = parser.parse_args()
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    args.processed_dir.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    running = True
    task_events = []
    nav_events = {1: [], 2: []}
    seen_task_revisions = set()
    assignments = {}
    global_valid_seen = {1: False, 2: False}
    first_global_valid_elapsed = {1: None, 2: None}

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    rclpy.init()
    node = rclpy.create_node("chapter8_assignment_recorder")
    snapshot_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    def on_tasks(message):
        now = elapsed(start)
        for task in message.tasks:
            key = (task.task_id, int(task.state_revision))
            if key in seen_task_revisions:
                continue
            seen_task_revisions.add(key)
            item = {
                "elapsed_sec": now,
                "task_id": task.task_id,
                "task_type": task.task_type,
                "region_id": task.region_id,
                "assigned_drone_id": int(task.assigned_drone_id),
                "state": STATE_NAMES.get(int(task.state), str(int(task.state))),
                "state_revision": int(task.state_revision),
                "detail": task.detail,
            }
            task_events.append(item)
            if item["assigned_drone_id"] in (1, 2) and item["state"] in ("ASSIGNED", "RUNNING"):
                assignments.setdefault(item["assigned_drone_id"], {})[item["task_id"]] = item

    def on_navigation(drone_id, message):
        now = elapsed(start)
        if message.global_valid:
            if not global_valid_seen[drone_id]:
                first_global_valid_elapsed[drone_id] = now
            global_valid_seen[drone_id] = True
        latest = nav_events[drone_id][-1] if nav_events[drone_id] else None
        current = {
            "elapsed_sec": now,
            "sample_sequence": int(message.sample_sequence),
            "map_epoch": int(message.map_epoch),
            "local_valid": bool(message.local_valid),
            "global_valid": bool(message.global_valid),
            "pose_source": SOURCE_NAMES.get(int(message.pose_source), str(int(message.pose_source))),
            "global_status": STATUS_NAMES.get(int(message.global_status), str(int(message.global_status))),
        }
        if latest is None or any(latest[k] != current[k] for k in (
                "map_epoch", "local_valid", "global_valid", "pose_source", "global_status")):
            nav_events[drone_id].append(current)

    node.create_subscription(TaskStateArray, "/mission/task_states", on_tasks, snapshot_qos)
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

    for drone_id, events in nav_events.items():
        (args.raw_dir / f"navigation_dron_{drone_id}.json").write_text(
            json.dumps(events, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (args.raw_dir / "task_state_events.json").write_text(
        json.dumps(task_events, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    rows = [item for drone in sorted(assignments) for item in assignments[drone].values()]
    with (args.processed_dir / "assignments.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "elapsed_sec", "assigned_drone_id", "task_id", "task_type", "region_id",
            "state", "state_revision", "detail"))
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda item: (item["elapsed_sec"], item["task_id"])))

    distinct_tasks = {item["task_id"] for item in rows}
    passed = global_valid_seen[1] and global_valid_seen[2] and set(assignments) == {1, 2} and len(distinct_tasks) >= 2
    summary = {
        "result": "PASS" if passed else "FAIL",
        "global_valid_seen": {str(k): value for k, value in global_valid_seen.items()},
        "first_global_valid_elapsed_sec": {str(k): value for k, value in first_global_valid_elapsed.items()},
        "assignments_by_drone": {str(k): sorted(value.values(), key=lambda item: item["task_id"]) for k, value in assignments.items()},
        "distinct_assigned_task_count": len(distinct_tasks),
        "task_state_event_count": len(task_events),
        "capture_duration_sec": elapsed(start),
        "checks": {
            "dron_1_global_valid": global_valid_seen[1],
            "dron_2_global_valid": global_valid_seen[2],
            "dron_1_assigned": 1 in assignments,
            "dron_2_assigned": 2 in assignments,
            "tasks_distintas": len(distinct_tasks) >= 2,
        },
    }
    (args.processed_dir / "assignment_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
