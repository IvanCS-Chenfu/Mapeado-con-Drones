#!/usr/bin/env python3
"""Asocia los snapshots 8.3 con el numero exacto de fuentes F6E del log reducido."""
import argparse
import csv
import json
import re
from pathlib import Path

PATTERN = re.compile(r"\[([0-9]+\.[0-9]+)\].*\[F6E-KF-EVIDENCE-WRITTEN\].*sources=(\d+)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    parser.add_argument("--reduced-log", required=True, type=Path)
    args = parser.parse_args()
    events = [(float(match.group(1)), int(match.group(2)))
              for match in PATTERN.finditer(args.reduced_log.read_text(encoding="utf-8"))]
    rows = []
    for label in "ABC":
        snapshot = json.loads((args.raw_dir / f"snapshot_{label}.json").read_text(encoding="utf-8"))
        stamp = snapshot["map_stamp_sec"]
        sources = 0
        for event_stamp, event_sources in events:
            if event_stamp <= stamp:
                sources = event_sources
            else:
                break
        voxels = snapshot["voxels"]
        rows.append({
            "snapshot": label,
            "map_revision": snapshot["map_revision"],
            "num_FREE": sum(item["state"] == 1 for item in voxels),
            "num_OCCUPIED": sum(item["state"] == 2 for item in voxels),
            "num_sources": sources,
            "num_keyframes_con_evidencia": snapshot["num_keyframes_con_evidencia"],
        })
    with (args.processed_dir / "voxel_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary_path = args.processed_dir / "voxel_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["snapshots"] = rows
    summary["sources_from_f6e_reduced_log"] = True
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
