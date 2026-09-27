#!/usr/bin/env python3
"""Captura pasiva de snapshots voxel sparse para la prueba 8.3."""
import argparse
import csv
import json
import math
import signal
import struct
import time
import zlib
from pathlib import Path

import rclpy
from mission_msgs.msg import KeyframeSparseEvidenceDelta, VoxelCell, VoxelMap
from orbslam3_msgs.msg import NavigationState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def cell_dict(cell):
    return {"ix": int(cell.ix), "iy": int(cell.iy), "iz": int(cell.iz),
            "state": int(cell.state), "score": float(cell.score)}


def metrics(snapshot):
    voxels = snapshot["voxels"]
    return {
        "map_revision": snapshot["map_revision"],
        "num_FREE": sum(item["state"] == VoxelCell.FREE for item in voxels),
        "num_OCCUPIED": sum(item["state"] == VoxelCell.OCCUPIED for item in voxels),
        "num_keyframes_con_evidencia": snapshot["num_keyframes_con_evidencia"],
        "num_sources": "ver [F6E-KF-EVIDENCE-WRITTEN] en log reducido",
    }


def write_png(path, width, height, pixels):
    def chunk(kind, payload):
        return (struct.pack(">I", len(payload)) + kind + payload +
                struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff))
    raw = b"".join(b"\x00" + bytes(pixels[row * width * 3:(row + 1) * width * 3])
                     for row in range(height))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) +
                     chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def render_projection(snapshots, figures_dir, filename, projector):
    width, height, panel_width, margin = 1500, 480, 500, 24
    pixels = bytearray([255] * (width * height * 3))
    colors = {VoxelCell.FREE: (47, 158, 104), VoxelCell.OCCUPIED: (211, 79, 79)}
    projected = []
    for snapshot in snapshots:
        scale = snapshot["voxel_size"]
        projected.append([(projector(cell["ix"] * scale, cell["iy"] * scale, cell["iz"] * scale), cell["state"])
                          for cell in snapshot["voxels"] if cell["state"] in colors])
    points = [point for panel in projected for point, _ in panel]
    if points:
        xmin, xmax = min(point[0] for point in points), max(point[0] for point in points)
        ymin, ymax = min(point[1] for point in points), max(point[1] for point in points)
    else:
        xmin = ymin = -1.0
        xmax = ymax = 1.0
    span_x, span_y = max(xmax - xmin, 1.0), max(ymax - ymin, 1.0)
    for panel_index, panel in enumerate(projected):
        left, right = panel_index * panel_width + margin, (panel_index + 1) * panel_width - margin
        top, bottom = margin, height - margin
        for x in range(left, right):
            for y in (top, bottom - 1):
                offset = (y * width + x) * 3
                pixels[offset:offset + 3] = b"\xb0\xb0\xb0"
        for y in range(top, bottom):
            for x in (left, right - 1):
                offset = (y * width + x) * 3
                pixels[offset:offset + 3] = b"\xb0\xb0\xb0"
        stride = max(1, len(panel) // 25000)
        for (x_value, y_value), state in panel[::stride]:
            x = int(left + (x_value - xmin) / span_x * (right - left - 1))
            y = int(bottom - 1 - (y_value - ymin) / span_y * (bottom - top - 1))
            color = colors[state]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    px, py = x + dx, y + dy
                    if left <= px < right and top <= py < bottom:
                        offset = (py * width + px) * 3
                        pixels[offset:offset + 3] = bytes(color)
    write_png(figures_dir / filename, width, height, pixels)


def render(snapshots, figures_dir):
    render_projection(snapshots, figures_dir, "voxel_snapshots_3d.png",
                      lambda x, y, z: (x - y, 0.5 * (x + y) - z))
    render_projection(snapshots, figures_dir, "voxel_snapshots_z1_2d.png",
                      lambda x, y, _z: (x, y))
    (figures_dir / "README.md").write_text(
        "Paneles de izquierda a derecha: A antes de anclar, B tras anclar y C tras el tramo final. Verde: FREE. Rojo: OCCUPIED.\n",
        encoding="utf-8")


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
    evidence_events = []
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
            "map_stamp_sec": float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9,
            "map_revision": int(message.map_revision),
            "voxel_size": float(message.voxel_size),
            "num_keyframes_con_evidencia": len(keyframes),
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
        if global_valid and keyframes and "B" not in labels and "A" in labels and int(message.map_revision) > snapshots[-1]["map_revision"]:
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
        evidence_events.append({
            "elapsed_sec": round(time.monotonic() - start, 3),
            "map_revision": int(delta.map_revision),
            "upserts": len(delta.upserts),
            "deletes": len(delta.deletes),
            "num_keyframes_con_evidencia": len(keyframes),
        })

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
    (args.raw_dir / "keyframe_evidence_events.json").write_text(
        json.dumps(evidence_events, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    rows = []
    for snapshot in snapshots:
        row = {"snapshot": snapshot["label"], **metrics(snapshot)}
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
