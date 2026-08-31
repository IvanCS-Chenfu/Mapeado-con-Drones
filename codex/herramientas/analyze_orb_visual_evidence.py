#!/usr/bin/env python3
"""Resume evidencia visual ORB por dron sin imponer umbrales absolutos."""

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


FIELDS = (
    "n_tracking_inliers",
    "inlier_ratio",
    "n_valid_stereo_depth",
    "stereo_depth_ratio",
    "depth_p50",
    "depth_p90",
    "disparity_p25",
    "disparity_p50",
    "grid_coverage_ratio",
)


def quantile(values, probability):
    values = sorted(values)
    if not values:
        return 0.0
    index = max(0.0, min(1.0, probability)) * (len(values) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    fraction = index - lower
    return values[lower] * (1.0 - fraction) + values[upper] * fraction


def summarize_metrics(rows):
    numeric = {
        field: [float(row[field]) for row in rows if row.get(field) not in (None, "")]
        for field in FIELDS
    }
    return {
        field: {
            "min": min(values) if values else 0.0,
            "p10": quantile(values, 0.10),
            "median": median(values) if values else 0.0,
            "p90": quantile(values, 0.90),
            "max": max(values) if values else 0.0,
        }
        for field, values in numeric.items()
    }


def summarize(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    tracking_ok_rows = [row for row in rows if row.get("tracking_state") == "2"]
    tracking_counts = {}
    for row in rows:
        state = row.get("tracking_state", "unknown")
        tracking_counts[state] = tracking_counts.get(state, 0) + 1
    return {
        "file": str(path),
        "frames": len(rows),
        "duration_sec": (
            float(rows[-1]["image_stamp_sec"]) - float(rows[0]["image_stamp_sec"])
            if len(rows) > 1 else 0.0
        ),
        "tracking_counts": tracking_counts,
        "metrics": summarize_metrics(rows),
        "tracking_ok_frames": len(tracking_ok_rows),
        "tracking_ok_metrics": summarize_metrics(tracking_ok_rows),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    summaries = [summarize(path) for path in sorted(
        args.input_dir.glob("*_orb_visual_evidence.csv"))]
    result = {"runs": summaries}
    encoded = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
