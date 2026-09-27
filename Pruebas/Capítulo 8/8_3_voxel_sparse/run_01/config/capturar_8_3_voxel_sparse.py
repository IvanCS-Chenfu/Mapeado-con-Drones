#!/usr/bin/env python3
"""Captura pasiva de snapshots voxel sparse para la prueba 8.3."""
import argparse
import csv
import json
import math
import signal
import time
from pathlib import Path

import rclpy
from mission_msgs.msg import KeyframeSparseEvidenceDelta, VoxelCell, VoxelMap
from orbslam3_msgs.msg import NavigationState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def cell_dict(cell):
    return {"ix": int(cell.ix), "iy": int(cell.iy), "iz": int(cell.iz),
            "state": int(cell.state), "score": float(cell.score)}


def metrics(snapshot, keyframes):
    voxels = snapshot["voxels"]
    return {
        "map_revision": snapshot["map_revision"],
        "num_FREE": sum(item["state"] == VoxelCell.FREE for item in voxels),
        "num_OCCUPIED": sum(item["state"] == VoxelCell.OCCUPIED for item in voxels),
        "num_keyframes_con_evidencia": len(keyframes),
        "num_sources": "ver [F6E-KF-EVIDENCE-WRITTEN] en log reducido",
    }


def render(snapshots, figures_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = ["A: antes de anclar", "B: tras anclar", "C: tramo final"]
    colors = {VoxelCell.FREE: "#2f9e68", VoxelCell.OCCUPIED: "#d34f4f"}
    fig = plt.figure(figsize=(15, 5))
    for index, snapshot in enumerate(snapshots):
        axis = fig.add_subplot(1, 3, index + 1, projection="3d")
        for state, color in colors.items():
            cells = [cell for cell in snapshot["voxels"] if cell["state"] == state]
            if cells:
                scale = snapshot["voxel_size"]
                axis.scatter([cell["ix"] * scale for cell in cells],
                             [cell["iy"] * scale for cell in cells],
                             [cell["iz"] * scale for cell in cells],
                             s=2, c=color, label="FREE" if state == VoxelCell.FREE else "OCCUPIED")
        axis.set_title(labels[index])
        axis.set_xlabel("X [m]")
        axis.set_ylabel("Y [m]")
        axis.set_zlabel("Z [m]")
        axis.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(figures_dir / "voxel_snapshots_3d.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharex=True, sharey=True)
    for index, snapshot in enumerate(snapshots):
        axis = axes[index]
        scale = snapshot["voxel_size"]
        target_iz = round(1.0 / scale)
        cells = [cell for cell in snapshot["voxels"] if abs(cell["iz"] - target_iz) <= 1]
        for state, color in colors.items():
            selected = [cell for cell in cells if cell["state"] == state]
            if selected:
                axis.scatter([cell["ix"] * scale for cell in selected],
                             [cell["iy"] * scale for cell in selected],
                             s=5, c=color, label="FREE" if state == VoxelCell.FREE else "OCCUPIED")
        axis.set_title(labels[index])
        axis.set_xlabel("X [m]")
        axis.grid(True, linewidth=0.4, alpha=0.4)
    axes[0].set_ylabel("Y [m]")
    axes[0].legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(figures_dir / "voxel_snapshots_z1_2d.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--figures-dir", required=True, type=Path)
    parser.add_argument("--duration-sec", type=float, default=210.0)
    args = parser.parse_args()
    for directory in (args.raw_dir, args.processed_dir, args.figures_dir):
        directory.mkdir(parents=True, exist_ok=True)

    latest_map = None
    snapshots = []
    keyframes = {}
    global_valid = False
    target_final = False
    running = True
    start = time.monotonic()

    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def store(label, message):
        nonlocal snapshots
        if any(item["label"] == label for item in snapshots):
            return
        snapshots.append({
            "label": label,
            "capture_elapsed_sec": round(time.monotonic() - start, 3),
            "map_revision": int(message.map_revision),
            "voxel_size": float(message.voxel_size),
            "voxels": [cell_dict(cell) for cell in message.voxels],
        })

    rclpy.init()
    node = rclpy.create_node("chapter8_voxel_sparse_recorder")
    snapshot_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_voxels(message):
        nonlocal latest_map
        latest_map = message
        labels = {item["label"] for item in snapshots}
        if global_valid and "B" not in labels and "A" in labels and int(message.map_revision) > snapshots[-1]["map_revision"]:
            store("B", message)
        if target_final and "C" not in labels and "B" in labels and int(message.map_revision) > snapshots[-1]["map_revision"]:
            store("C", message)

    def on_navigation(message):
        nonlocal global_valid, target_final
        if message.global_valid and not global_valid:
            global_valid = True
            if latest_map is not None:
                store("A", latest_map)
        position = message.w_t_body.position
        if message.global_valid and math.dist((position.x, position.y, position.z), (10.0, -10.0, 1.0)) <= 0.7:
            target_final = True

    def on_evidence(delta):
        for item in delta.deletes:
            keyframes.pop((int(item.drone_id), int(item.map_epoch), int(item.keyframe_id)), None)
        for item in delta.upserts:
            keyframes[(int(item.drone_id), int(item.map_epoch), int(item.keyframe_id))] = {
                "geometry_revision": int(item.geometry_revision),
                "pose_revision": int(item.pose_revision),
                "points": len(item.points_k),
            }

    node.create_subscription(VoxelMap, "/mission/voxel_map", on_voxels, snapshot_qos)
    node.create_subscription(KeyframeSparseEvidenceDelta, "/global_keyframe_sparse_evidence_delta", on_evidence, snapshot_qos)
    node.create_subscription(NavigationState, "/dron_1/orbslam/navigation_state", on_navigation,
                             QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))

    deadline = start + args.duration_sec
    while running and time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)

    if latest_map is not None and "A" not in {item["label"] for item in snapshots}:
        store("A", latest_map)
    if latest_map is not None and "B" not in {item["label"] for item in snapshots}:
        store("B", latest_map)
    if latest_map is not None and "C" not in {item["label"] for item in snapshots}:
        store("C", latest_map)
    snapshots.sort(key=lambda item: ("ABC".index(item["label"]), item["capture_elapsed_sec"]))

    node.destroy_node()
    rclpy.shutdown()
    for snapshot in snapshots:
        (args.raw_dir / f"snapshot_{snapshot['label']}.json").write_text(
            json.dumps(snapshot, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    keyframe_json = {
        f"{drone_id}:{epoch}:{keyframe_id}": value
        for (drone_id, epoch, keyframe_id), value in keyframes.items()
    }
    (args.raw_dir / "keyframe_evidence.json").write_text(
        json.dumps(keyframe_json, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    rows = []
    for snapshot in snapshots:
        row = {"snapshot": snapshot["label"], **metrics(snapshot, keyframes)}
        rows.append(row)
    with (args.processed_dir / "voxel_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("snapshot", "map_revision", "num_FREE", "num_OCCUPIED", "num_sources", "num_keyframes_con_evidencia"))
        writer.writeheader()
        writer.writerows(rows)
    strictly_incremental = (
        len(snapshots) == 3 and
        snapshots[0]["map_revision"] < snapshots[1]["map_revision"] < snapshots[2]["map_revision"]
    )
    summary = {"result": "PASS" if strictly_incremental and global_valid and target_final else "FAIL",
               "global_valid_seen": global_valid, "final_target_seen": target_final,
               "map_revision_strictly_incremental": strictly_incremental,
               "snapshots": rows, "keyframes_with_evidence": len(keyframes),
               "capture_duration_sec": round(time.monotonic() - start, 3)}
    (args.processed_dir / "voxel_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if len(snapshots) == 3:
        render(snapshots, args.figures_dir)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
