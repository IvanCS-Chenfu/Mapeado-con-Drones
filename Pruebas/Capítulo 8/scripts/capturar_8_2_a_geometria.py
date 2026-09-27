#!/usr/bin/env python3
"""Captura pasiva y valida la geometria publicada para la prueba 8.2-A."""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from mission_msgs.msg import MissionGeometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


SIDE_NAMES = {0: "AB", 1: "BC", 2: "CD", 3: "DA"}


def box_to_dict(box):
    return {
        "min": [box.min.x, box.min.y, box.min.z],
        "max": [box.max.x, box.max.y, box.max.z],
    }


def nearly_equal(left, right, tolerance=1.0e-9):
    return abs(float(left) - float(right)) <= tolerance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mission-config", required=True, type=Path)
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    args = parser.parse_args()
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    args.processed_dir.mkdir(parents=True, exist_ok=True)

    config = yaml.safe_load(args.mission_config.read_text(encoding="utf-8"))
    expected_roi = config["mapping_roi"]
    expected_height = float(config["level_height"])
    expected_levels = math.ceil(
        (float(expected_roi["max"][2]) - float(expected_roi["min"][2])) / expected_height
    )

    rclpy.init()
    node = rclpy.create_node("chapter8_geometry_recorder")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    captured = []
    subscription = node.create_subscription(
        MissionGeometry, "/mission/geometry", captured.append, qos
    )
    deadline = time.monotonic() + args.timeout_sec
    while not captured and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    if not captured:
        raise RuntimeError("No se recibio /mission/geometry antes del timeout")

    message = captured[-1]
    regions = []
    for region in message.regions:
        regions.append({
            "region_id": region.region_id,
            "level_index": region.level_index,
            "side": SIDE_NAMES.get(region.side, f"UNKNOWN_{region.side}"),
            "bounds": box_to_dict(region.bounds),
        })
    levels = [
        {"level_index": level.level_index, "z_min": level.z_min, "z_max": level.z_max}
        for level in message.levels
    ]
    snapshot = {
        "mission_id": message.mission_id,
        "config_revision": message.config_revision,
        "mapping_roi": box_to_dict(message.mapping_roi),
        "level_height": message.level_height,
        "levels": levels,
        "regions": regions,
    }
    (args.raw_dir / "mission_geometry.json").write_text(
        json.dumps(snapshot, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    roi_ok = all(
        nearly_equal(snapshot["mapping_roi"][edge][axis], expected_roi[edge][axis])
        for edge in ("min", "max") for axis in range(3)
    )
    height_ok = nearly_equal(snapshot["level_height"], expected_height)
    levels_ok = len(levels) == expected_levels
    regions_by_level = {level["level_index"]: [] for level in levels}
    for region in regions:
        regions_by_level.setdefault(region["level_index"], []).append(region)
    side_sets = {
        level: sorted(region["side"] for region in level_regions)
        for level, level_regions in regions_by_level.items()
    }
    subrois_ok = all(sides == ["AB", "BC", "CD", "DA"] for sides in side_sets.values())
    bounds_ok = all(
        region["bounds"]["min"][axis] >= snapshot["mapping_roi"]["min"][axis]
        and region["bounds"]["max"][axis] <= snapshot["mapping_roi"]["max"][axis]
        for region in regions for axis in range(3)
    )
    valid = roi_ok and height_ok and levels_ok and subrois_ok and bounds_ok
    summary = {
        "result": "PASS" if valid else "FAIL",
        "mission_id": message.mission_id,
        "numero_niveles": len(levels),
        "numero_subrois_total": len(regions),
        "subrois_por_nivel": {str(level): len(items) for level, items in regions_by_level.items()},
        "ids_de_region": [region["region_id"] for region in regions],
        "bounds_de_cada_region": {region["region_id"]: region["bounds"] for region in regions},
        "checks": {
            "roi_igual_a_yaml": roi_ok,
            "level_height_igual_a_yaml": height_ok,
            "numero_niveles_esperado": expected_levels,
            "niveles_coherentes": levels_ok,
            "cuatro_lados_ab_bc_cd_da_por_nivel": subrois_ok,
            "bounds_dentro_de_roi": bounds_ok,
        },
    }
    (args.processed_dir / "geometry_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    with (args.processed_dir / "regions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "region_id", "level_index", "side", "min_x", "min_y", "min_z", "max_x", "max_y", "max_z"))
        writer.writeheader()
        for region in regions:
            bounds = region["bounds"]
            writer.writerow({
                "region_id": region["region_id"], "level_index": region["level_index"], "side": region["side"],
                "min_x": bounds["min"][0], "min_y": bounds["min"][1], "min_z": bounds["min"][2],
                "max_x": bounds["max"][0], "max_y": bounds["max"][1], "max_z": bounds["max"][2],
            })
    lines = [
        "# Prueba 8.2-A - resumen automatico", "",
        f"Resultado: **{summary['result']}**", "",
        f"- Mision: `{summary['mission_id']}`", f"- Niveles: `{summary['numero_niveles']}`",
        f"- SubROIs totales: `{summary['numero_subrois_total']}`",
        f"- SubROIs por nivel: `{summary['subrois_por_nivel']}`",
        f"- ROI igual al YAML: `{roi_ok}`", f"- Level height igual al YAML: `{height_ok}`",
        f"- Cuatro lados AB/BC/CD/DA por nivel: `{subrois_ok}`", f"- Bounds dentro de ROI: `{bounds_ok}`", "",
        "La evidencia visual corresponde al nivel central (indice 1).",
    ]
    (args.processed_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
