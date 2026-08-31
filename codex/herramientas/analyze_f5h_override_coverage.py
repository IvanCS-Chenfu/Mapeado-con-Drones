#!/usr/bin/env python3
"""Resume la cobertura temporal de los overrides diagnosticos F5H."""

import argparse
import json
import math
import re
from pathlib import Path


FIELD_RE = re.compile(r"([a-z_]+)=([^ ]+)")


def percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(ratio * len(ordered)) - 1))
    return ordered[index]


def summarize(lines, drone_id):
    token = f"[dron_{drone_id}.navigation_state_mux]"
    samples = []
    for line in lines:
        if token not in line or "[F5H-CHANNEL-OVERRIDE]" not in line:
            continue
        fields = dict(FIELD_RE.findall(line))
        if fields.get("override_requested") != "true" or fields.get("navigation_source") != "ORB":
            continue
        samples.append(fields)

    applied = [sample.get("applied") == "true" for sample in samples]
    current_misses = 0
    max_misses = 0
    for is_applied in applied:
        current_misses = 0 if is_applied else current_misses + 1
        max_misses = max(max_misses, current_misses)
    skews = [
        float(sample["gt_alignment_skew"])
        for sample in samples
        if float(sample.get("gt_alignment_skew", "-1")) >= 0.0
    ]
    requested = len(samples)
    applied_count = sum(applied)
    return {
        "drone_id": drone_id,
        "override_requested_count": requested,
        "override_valid_count": sum(sample.get("valid") == "true" for sample in samples),
        "override_applied_count": applied_count,
        "override_missed_count": requested - applied_count,
        "override_applied_ratio": applied_count / requested if requested else 0.0,
        "max_consecutive_override_misses": max_misses,
        "gt_alignment_skew_mean": sum(skews) / len(skews) if skews else None,
        "gt_alignment_skew_p50": percentile(skews, 0.50),
        "gt_alignment_skew_p95": percentile(skews, 0.95),
        "gt_alignment_skew_p99": percentile(skews, 0.99),
        "gt_alignment_skew_max": max(skews) if skews else None,
        "gt_interpolated_count": sum(
            sample.get("gt_interpolated") == "true" for sample in samples),
        "gt_causal_propagation_count": sum(
            sample.get("gt_causal_propagation") == "true" for sample in samples),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reduced-log", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--drone-id", type=int, default=2)
    args = parser.parse_args()
    summary = summarize(
        Path(args.reduced_log).read_text(errors="replace").splitlines(), args.drone_id)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
