#!/usr/bin/env python3
"""Correlaciona el primer LOOK_AND_CAPTURE con VoxelMap y yaw de 8.4-A."""
import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def metrics(snapshot):
    voxels = snapshot["voxels"]
    return {
        "map_revision": snapshot["map_revision"],
        "num_UNKNOWN": sum(item["state"] == 0 for item in voxels),
        "num_FREE": sum(item["state"] == 1 for item in voxels),
        "num_OCCUPIED": sum(item["state"] == 2 for item in voxels),
        "coverage_progress": [{"task_id": item["task_id"], "progress": item["progress"],
                               "intervals": item["coverage_intervals"]} for item in snapshot["tasks"]],
    }


def stamp(line):
    match = re.search(r"\[([0-9]+\.[0-9]+)\]", line)
    return float(match.group(1)) if match else None


def signed_delta(before, after):
    return (after - before + 180.0) % 360.0 - 180.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reduced-log", required=True, type=Path)
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    args = parser.parse_args()
    lines = args.reduced_log.read_text(encoding="utf-8", errors="replace").splitlines()
    accepted = next((line for line in lines if "[F6C-AUTONOMOUS-COMMAND-ACCEPTED]" in line and "type=2" in line), None)
    if accepted is None:
        raise SystemExit("No aparece un LOOK_AND_CAPTURE aceptado (type=2).")
    command_match = re.search(r"command=([^ ]+)", accepted)
    command_id = command_match.group(1) if command_match else ""
    applied = next((line for line in lines if (
        "[F6F-DEPTH-SOURCES-APPLIED]" in line or "[F8A-VIEW-UNKNOWN-TEST-APPLIED]" in line
    ) and f"command={command_id}" in line), None)
    written = next((line for line in lines if "[F6F-DEPTH-SOURCES-WRITTEN]" in line and f"command={command_id}" in line), None)
    if applied is None or written is None:
        raise SystemExit("No se completo la correlacion depth del LOOK inicial.")
    applied_stamp, accepted_stamp = stamp(applied), stamp(accepted)
    sources_match = re.search(r"free=(\d+) direct_free=(\d+) occupied=(\d+)", written)
    if sources_match is None:
        raise SystemExit("El marcador F6F no contiene los contadores de fuentes depth.")
    source_free, source_direct_free, source_occupied = map(int, sources_match.groups())
    before = load(args.raw_dir / "before_view_unknown.json")
    candidates = sorted((load(path) for path in (args.raw_dir / "map_history").glob("map_*.json")),
                        key=lambda item: item["map_stamp_sec"])
    after = next((item for item in candidates if item["map_stamp_sec"] >= applied_stamp), None)
    if after is None:
        raise SystemExit("No hay snapshot de mapa posterior al depth aplicado.")
    clock_offsets = [item["map_stamp_sec"] - item["capture_elapsed_sec"] for item in candidates]
    clock_offset = statistics.median(clock_offsets)
    accepted_elapsed = accepted_stamp - clock_offset
    applied_elapsed = applied_stamp - clock_offset
    nav = load(args.raw_dir / "navigation_events.json")
    yaw_before = min(nav, key=lambda item: abs(item["elapsed_sec"] - accepted_elapsed), default=None)
    yaw_after = min(nav, key=lambda item: abs(item["elapsed_sec"] - applied_elapsed), default=None)
    before_metrics, after_metrics = metrics(before), metrics(after)
    coverage_unchanged = before_metrics["coverage_progress"] == after_metrics["coverage_progress"]
    result = {
        "command_id": command_id, "accepted_stamp_sec": accepted_stamp,
        "applied_stamp_sec": applied_stamp, "accepted_elapsed_sec": accepted_elapsed,
        "applied_elapsed_sec": applied_elapsed, "written_kind_vista_unknown": "kind=vista_unknown" in written,
        "source_free": source_free, "source_direct_free": source_direct_free,
        "source_occupied": source_occupied,
        "map_revision_before": before_metrics["map_revision"], "map_revision_after": after_metrics["map_revision"],
        "free_before": before_metrics["num_FREE"], "free_after": after_metrics["num_FREE"],
        "free_delta": after_metrics["num_FREE"] - before_metrics["num_FREE"],
        "occupied_before": before_metrics["num_OCCUPIED"], "occupied_after": after_metrics["num_OCCUPIED"],
        "occupied_delta": after_metrics["num_OCCUPIED"] - before_metrics["num_OCCUPIED"],
        "unknown_before": before_metrics["num_UNKNOWN"], "unknown_after": after_metrics["num_UNKNOWN"],
        "coverage_unchanged": coverage_unchanged,
        "yaw_before_deg": yaw_before["yaw_deg"] if yaw_before else None,
        "yaw_after_deg": yaw_after["yaw_deg"] if yaw_after else None,
    }
    result["yaw_delta_deg"] = (signed_delta(result["yaw_before_deg"], result["yaw_after_deg"])
                               if yaw_before and yaw_after else None)
    result["result"] = "PASS" if (result["written_kind_vista_unknown"] and
                                    result["source_free"] > 0 and result["source_direct_free"] == 0 and
                                    result["source_occupied"] == 0 and coverage_unchanged and
                                    result["yaw_delta_deg"] is not None and -120.0 <= result["yaw_delta_deg"] <= -60.0) else "FAIL"
    (args.raw_dir / "after_view_unknown.json").write_text(
        json.dumps(after, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (args.processed_dir / "view_unknown_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (args.processed_dir / "view_unknown_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(result)); writer.writeheader(); writer.writerow(result)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
