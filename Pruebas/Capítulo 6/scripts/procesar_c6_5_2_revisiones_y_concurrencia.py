#!/usr/bin/env python3
"""Extrae evidencia temporal de revisiones y KFs tardios de la prueba 6.5.2."""

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
MARKER_RE = re.compile(r"\[(C6-KF-POSE|F3[A-Z0-9-]+)\]\s*(.*)")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")
RELEVANT = {
    "F3I-GRAPH-BUILD", "F3J-OPTIMIZE", "F3L-VALIDATE",
    "F3K-ATOMIC-COMMIT", "F3K-COMMIT-STALE", "F3K-CONTINUATION-UPDATE",
    "F3K-FUTURE-KF-PROPAGATE", "F3H-FID-REVALIDATE",
    "F3P-FUSION-RETRY", "F3Q-OPT-START", "F3Q-OPT-END",
    "F3Q-POST-OPT-LOOPS",
}


def parse_events(path):
    events = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = MARKER_RE.search(line)
        if not match:
            continue
        stamps = TIME_RE.findall(line[:match.start()])
        if not stamps:
            continue
        marker = match.group(1)
        if marker != "C6-KF-POSE" and marker not in RELEVANT:
            continue
        events.append({
            "time_abs_s": float(stamps[-1]),
            "marker": marker,
            "fields": dict(FIELD_RE.findall(match.group(2))),
        })
    if not events:
        raise RuntimeError("El log reducido no contiene eventos C6/F3 requeridos.")
    start = min(event["time_abs_s"] for event in events)
    for event in events:
        event["time_s"] = event["time_abs_s"] - start
    return events


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def float_or_none(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def first_kf_arrivals(events):
    arrivals = {}
    for event in events:
        if event["marker"] != "C6-KF-POSE":
            continue
        fields = event["fields"]
        key = (fields.get("drone_id"), fields.get("epoch"), fields.get("kf"))
        if key not in arrivals:
            arrivals[key] = event
    return list(arrivals.values())


def event_time(events, marker):
    matches = [event["time_s"] for event in events if event["marker"] == marker]
    return matches[0] if matches else None


def build_summary(events, arrivals):
    graph_time = event_time(events, "F3I-GRAPH-BUILD")
    commit_time = event_time(events, "F3K-ATOMIC-COMMIT")
    phases = Counter()
    for event in arrivals:
        if graph_time is not None and event["time_s"] >= graph_time and (
                commit_time is None or event["time_s"] < commit_time):
            phases["during_optimization"] += 1
        elif commit_time is not None and event["time_s"] >= commit_time:
            phases["after_commit"] += 1
        else:
            phases["before_optimization"] += 1
    counts = Counter(event["marker"] for event in events)
    after_commit_counts = Counter(
        event["marker"] for event in events
        if commit_time is not None and event["time_s"] >= commit_time
        and event["marker"] != "C6-KF-POSE")
    commit = next((event for event in events if event["marker"] == "F3K-ATOMIC-COMMIT"), None)
    fields = commit["fields"] if commit else {}
    return {
        "optimization_start_source": "F3Q-OPT-START" if event_time(events, "F3Q-OPT-START") is not None else "F3I-GRAPH-BUILD",
        "optimization_start_time_s": event_time(events, "F3Q-OPT-START") or graph_time,
        "commit_time_s": commit_time,
        "unique_keyframes": len(arrivals),
        "keyframes_before_optimization": phases["before_optimization"],
        "keyframes_during_optimization": phases["during_optimization"],
        "keyframes_after_commit": phases["after_commit"],
        "commit_late_window": float_or_none(fields.get("late_window")),
        "commit_tail": float_or_none(fields.get("tail")),
        "commit_moved_kfs": float_or_none(fields.get("moved_kfs")),
        "event_counts": dict(sorted(counts.items())),
        "event_counts_after_commit": dict(sorted(after_commit_counts.items())),
    }


def write_event_tables(events, arrivals, summary, directory):
    rows = []
    for event in events:
        if event["marker"] == "C6-KF-POSE":
            continue
        fields = event["fields"]
        rows.append({
            "time_s": round(event["time_s"], 6),
            "marker": event["marker"],
            "task": fields.get("task", fields.get("previous_task", "")),
            "arrival_id": fields.get("arrival_id", ""),
            "commit": fields.get("commit", ""),
            "revision": fields.get("revision", ""),
            "late_window": fields.get("late_window", ""),
            "tail": fields.get("tail", ""),
            "count": fields.get("count", ""),
            "decision": fields.get("decision", ""),
            "reason": fields.get("reason", fields.get("cause", "")),
        })
    fields = list(rows[0]) if rows else ["time_s", "marker"]
    write_csv(directory / "eventos_revision.csv", fields, rows)
    kf_rows = []
    start_time = summary["optimization_start_time_s"]
    commit_time = summary["commit_time_s"]
    for event in arrivals:
        phase = "before_optimization"
        if start_time is not None and event["time_s"] >= start_time:
            phase = "during_optimization"
        if commit_time is not None and event["time_s"] >= commit_time:
            phase = "after_commit"
        kf_rows.append({
            "time_s": round(event["time_s"], 6),
            "drone_id": event["fields"].get("drone_id", ""),
            "epoch": event["fields"].get("epoch", ""),
            "keyframe_id": event["fields"].get("kf", ""),
            "pose_revision": event["fields"].get("pose_revision", ""),
            "phase": phase,
        })
    write_csv(directory / "keyframes_temporales.csv", list(kf_rows[0]), kf_rows)


def plot_timeline(events, arrivals, summary, directory):
    categories = [
        ("F3K-FUTURE-KF-PROPAGATE", "Propagacion futura"),
        ("F3H-FID-REVALIDATE", "Revalidacion fiducial"),
        ("F3Q-POST-OPT-LOOPS", "Tareas post-optimizacion"),
    ]
    figure, axis = plt.subplots(figsize=(11.0, 5.0))
    axis.scatter([event["time_s"] for event in arrivals], [0] * len(arrivals),
                 color="#1f77b4", marker="|", s=130, label="Creacion de KF")
    for index, (marker, label) in enumerate(categories, start=1):
        times = [event["time_s"] for event in events if event["marker"] == marker]
        if times:
            axis.scatter(times, [index] * len(times), s=42, label=label)
    start_time = summary["optimization_start_time_s"]
    if start_time is not None:
        axis.axvline(start_time, color="#2ca02c", linestyle="--",
                    label="Inicio efectivo de optimizacion")
    if summary["commit_time_s"] is not None:
        axis.axvline(summary["commit_time_s"], color="#9467bd", linestyle=":",
                    label="Commit")
    axis.set_yticks(range(len(categories) + 1), ["Creacion de KF"] + [label for _, label in categories])
    axis.set_xlabel("Tiempo desde el primer evento observado [s]")
    axis.grid(True, axis="x", alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc="upper left")
    figure.tight_layout()
    directory.mkdir(parents=True, exist_ok=True)
    figure.savefig(directory / "timeline_revisiones_y_kfs_tardios.png", dpi=180, bbox_inches="tight")
    figure.savefig(directory / "timeline_revisiones_y_kfs_tardios.pdf", bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--figure-dir", required=True, type=Path)
    args = parser.parse_args()
    events = parse_events(args.log)
    arrivals = first_kf_arrivals(events)
    summary = build_summary(events, arrivals)
    write_event_tables(events, arrivals, summary, args.data_dir)
    (args.data_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    plot_timeline(events, arrivals, summary, args.figure_dir)


if __name__ == "__main__":
    main()
