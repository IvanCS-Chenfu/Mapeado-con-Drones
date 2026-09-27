#!/usr/bin/env python3
"""Normaliza los diagnosticos F3 y C6 de una ejecucion del Capitulo 6."""

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path


LOG_TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
MARKER_RE = re.compile(r"\[(F3[A-Z0-9-]+|C6-QUEUE-SAMPLE)\]\s*(.*)")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")
RELEVANT_DECISIONS = {
    "fusion_candidate",
    "optimization_evidence",
    "optimization_committed",
    "constraint_activated",
    "anchor_proposed",
}


def parse_line(line):
    marker_match = MARKER_RE.search(line)
    if not marker_match:
        return None
    times = LOG_TIME_RE.findall(line[:marker_match.start()])
    if not times:
        return None
    payload = marker_match.group(2)
    return {
        "time_s": float(times[-1]),
        "marker": marker_match.group(1),
        "fields": dict(FIELD_RE.findall(payload)),
        "raw": line.rstrip("\n"),
    }


def as_int(fields, name):
    try:
        return int(fields.get(name, "0"))
    except ValueError:
        return 0


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    events = []
    with args.log.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            parsed = parse_line(line)
            if parsed:
                events.append(parsed)

    if not events:
        raise SystemExit("No se encontraron marcadores F3/C6 con timestamp en el log.")

    start_time = events[0]["time_s"]
    events_rows = []
    reduced_lines = []
    for event in events:
        event["relative_time_s"] = event["time_s"] - start_time
        events_rows.append({
            "time_s": f"{event['relative_time_s']:.6f}",
            "marker": event["marker"],
            "fields_json": json.dumps(event["fields"], sort_keys=True),
        })
        reduced_lines.append(event["raw"])
    write_csv(output_dir / "eventos_f3.csv", ["time_s", "marker", "fields_json"], events_rows)
    (output_dir / "eventos_f3_reducidos.log").write_text(
        "\n".join(reduced_lines) + "\n", encoding="utf-8")

    bow = regions = ransac = ransac_accepted = ransac_rejected = 0
    decisions = optimizations = loop_tasks = 0
    pipeline_rows = []
    for event in events:
        fields = event["fields"]
        if event["marker"] == "F3O-LOOP-DONE":
            loop_tasks += 1
            bow += as_int(fields, "bow")
            regions += as_int(fields, "regions")
            if fields.get("decision") in RELEVANT_DECISIONS:
                decisions += 1
        elif event["marker"] == "F3O-RANSAC":
            ransac += 1
            if fields.get("accepted") == "true":
                ransac_accepted += 1
            else:
                ransac_rejected += 1
        elif event["marker"] == "F3Q-OPT-START":
            optimizations += 1
        else:
            continue
        pipeline_rows.append({
            "time_s": f"{event['relative_time_s']:.6f}",
            "loop_tasks_cum": loop_tasks,
            "bow_candidates_cum": bow,
            "regions_cum": regions,
            "ransac_runs_cum": ransac,
            "ransac_accepted_cum": ransac_accepted,
            "ransac_rejected_cum": ransac_rejected,
            "loop_decisions_cum": decisions,
            "optimizations_cum": optimizations,
        })
    pipeline_fields = [
        "time_s", "loop_tasks_cum", "bow_candidates_cum", "regions_cum",
        "ransac_runs_cum", "ransac_accepted_cum", "ransac_rejected_cum",
        "loop_decisions_cum", "optimizations_cum",
    ]
    write_csv(output_dir / "pipeline_loop.csv", pipeline_fields, pipeline_rows)

    queue_rows = []
    queue_start_ns = None
    for event in events:
        if event["marker"] != "C6-QUEUE-SAMPLE":
            continue
        fields = event["fields"]
        sim_time_ns = as_int(fields, "sim_time_ns")
        if queue_start_ns is None:
            queue_start_ns = sim_time_ns
        queue_rows.append({
            "time_s": f"{(sim_time_ns - queue_start_ns) / 1_000_000_000.0:.6f}",
            "primary_pending": as_int(fields, "primary_pending"),
            "secondary_pending": as_int(fields, "secondary_pending"),
            "secondary_critical": as_int(fields, "secondary_critical"),
            "secondary_maintenance": as_int(fields, "secondary_maintenance"),
            "optimization_active": int(fields.get("optimization_active") == "true"),
            "backpressure_active": int(fields.get("backpressure_active") == "true"),
            "primary_high": as_int(fields, "primary_high"),
            "primary_low": as_int(fields, "primary_low"),
            "secondary_high": as_int(fields, "secondary_high"),
            "secondary_low": as_int(fields, "secondary_low"),
        })
    queue_fields = [
        "time_s", "primary_pending", "secondary_pending", "secondary_critical",
        "secondary_maintenance", "optimization_active", "backpressure_active",
        "primary_high", "primary_low", "secondary_high", "secondary_low",
    ]
    write_csv(output_dir / "colas_backpressure.csv", queue_fields, queue_rows)

    digest = sha256_file(args.log)
    summary = {
        "source_log": str(args.log),
        "source_log_sha256": digest,
        "events": len(events),
        "pipeline_rows": len(pipeline_rows),
        "queue_samples": len(queue_rows),
        "bow_candidates": bow,
        "regions": regions,
        "ransac_runs": ransac,
        "ransac_accepted": ransac_accepted,
        "ransac_rejected": ransac_rejected,
        "relevant_loop_decisions": decisions,
        "optimization_starts": optimizations,
        "backpressure_samples_active": sum(
            row["backpressure_active"] for row in queue_rows),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
