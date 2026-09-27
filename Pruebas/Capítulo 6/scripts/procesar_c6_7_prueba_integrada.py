#!/usr/bin/env python3
"""Procesa los eventos F3/C6 de la prueba integrada 6.7."""

import argparse
import csv
import json
import re
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt

TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
MARKER_RE = re.compile(r"\[(F[13][A-Z0-9-]+|C6-QUEUE-SAMPLE)\]\s*(.*)")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")
SCORE_RE = re.compile(r"score=\(\+(\d+),-(\d+),dirty=(\d+)\)")


def integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def parse_events(path):
    events = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = MARKER_RE.search(line)
        if not match:
            continue
        stamps = TIME_RE.findall(line[:match.start()])
        if not stamps:
            continue
        events.append({
            "absolute_time_s": float(stamps[-1]),
            "marker": match.group(1),
            "fields": dict(FIELD_RE.findall(match.group(2))),
            "payload": match.group(2),
        })
    if not events:
        raise RuntimeError("El log reducido no contiene eventos F1/F3/C6 con timestamp.")
    origin = events[0]["absolute_time_s"]
    for event in events:
        event["time_s"] = event["absolute_time_s"] - origin
    return events


def write_csv(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def category(marker):
    if marker.startswith("F1"):
        return "Fiduciales"
    if marker.startswith("F3O"):
        return "Loops/RANSAC"
    if marker.startswith("F3Q"):
        return "Optimizacion"
    if marker.startswith("F3P"):
        return "Fusion"
    if marker.startswith("F3R"):
        return "Score/commit"
    return "Colas/backpressure"


def plot_timeline(events, output):
    labels = ["Fiduciales", "Loops/RANSAC", "Optimizacion", "Fusion", "Score/commit", "Colas/backpressure"]
    colors = ["#17becf", "#9467bd", "#d62728", "#2ca02c", "#ff7f0e", "#1f77b4"]
    figure, axis = plt.subplots(figsize=(12, 4.8))
    for index, (label, color) in enumerate(zip(labels, colors)):
        times = [event["time_s"] for event in events if category(event["marker"]) == label]
        if times:
            axis.scatter(times, [index] * len(times), s=13, color=color, label=label)
    axis.set_yticks(range(len(labels)), labels)
    axis.set_xlabel("Tiempo desde el primer evento instrumentado [s]")
    axis.set_title("Timeline global de eventos 6.7")
    axis.grid(axis="x", alpha=0.25)
    axis.legend(loc="upper center", ncol=3)
    figure.tight_layout()
    save(figure, output / "timeline_global")


def plot_counters(events, output):
    counters = {"loops": 0, "ransac": 0, "ransac_accepted": 0, "optimization": 0, "fusion": 0, "commits": 0}
    rows = []
    for event in events:
        marker = event["marker"]
        if marker == "F3O-LOOP-DONE":
            counters["loops"] += 1
        elif marker == "F3O-RANSAC":
            counters["ransac"] += 1
            counters["ransac_accepted"] += int(event["fields"].get("accepted") == "true")
        elif marker == "F3Q-LOOP-OPT":
            counters["optimization"] += 1
        elif marker == "F3P-FUSION":
            counters["fusion"] += 1
        if marker == "F3K-ATOMIC-COMMIT":
            counters["commits"] += 1
        if marker in {"F3O-LOOP-DONE", "F3O-RANSAC", "F3Q-LOOP-OPT", "F3P-FUSION", "F3K-ATOMIC-COMMIT"}:
            rows.append({"time_s": event["time_s"], **counters})
    if not rows:
        return []
    figure, axis = plt.subplots(figsize=(11, 5.2))
    for key, label, color in (
        ("loops", "Loops", "#9467bd"), ("ransac", "RANSAC", "#17becf"),
        ("ransac_accepted", "RANSAC aceptados", "#2ca02c"),
        ("optimization", "Optimizaciones", "#d62728"), ("fusion", "Fusiones", "#1f77b4"),
        ("commits", "Commits", "#ff7f0e"),
    ):
        axis.step([row["time_s"] for row in rows], [row[key] for row in rows], where="post", label=label, color=color)
    axis.set_xlabel("Tiempo desde el primer evento instrumentado [s]")
    axis.set_ylabel("Conteo acumulado")
    axis.set_title("Contadores principales del pipeline")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=2)
    figure.tight_layout()
    save(figure, output / "contadores_pipeline")
    return rows


def queue_rows(events):
    rows = []
    for event in events:
        if event["marker"] != "C6-QUEUE-SAMPLE":
            continue
        fields = event["fields"]
        rows.append({
            "time_s": event["time_s"],
            "primary_pending": integer(fields.get("primary_pending")),
            "secondary_pending": integer(fields.get("secondary_pending")),
            "optimization_active": int(fields.get("optimization_active") == "true"),
            "backpressure_active": int(fields.get("backpressure_active") == "true"),
        })
    return rows


def plot_queues(rows, output):
    if not rows:
        return
    figure, axis = plt.subplots(figsize=(11, 5.0))
    time = [row["time_s"] for row in rows]
    axis.step(time, [row["primary_pending"] for row in rows], where="post", label="Cola primaria", color="#1f77b4")
    axis.step(time, [row["secondary_pending"] for row in rows], where="post", label="Cola secundaria", color="#9467bd")
    axis.step(time, [row["optimization_active"] for row in rows], where="post", label="Optimizacion activa", color="#d62728")
    axis.step(time, [row["backpressure_active"] for row in rows], where="post", label="Backpressure activo", color="#ff7f0e")
    axis.set_xlabel("Tiempo desde el primer evento instrumentado [s]")
    axis.set_ylabel("Profundidad / estado binario")
    axis.set_title("Colas, optimizacion y backpressure")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    save(figure, output / "colas_y_backpressure")


def plot_target_trajectories(output):
    d1 = [(0, -10), (-10, -10), (-10, 0), (-10, 10), (0, 10), (10, 10), (10, 0), (10, -10), (0, -10)] * 2
    d2 = [(0, -10), (10, -10), (10, 0), (10, 10), (0, 10), (-10, 10), (-10, 0), (-10, -10), (0, -10)] * 2
    figure, axis = plt.subplots(figsize=(6.4, 6.2))
    axis.plot(*zip(*d1), marker="o", color="#1f77b4", label="dron_1, Z=1.0 m")
    axis.plot(*zip(*d2), marker="o", color="#ff7f0e", label="dron_2, Z=1.3 m")
    axis.add_patch(plt.Rectangle((-7, -7), 14, 14, fill=False, linestyle="--", color="#555555", label="Edificio"))
    axis.scatter([0, 0], [-10, 10], color="#d62728", marker="s", zorder=4, label="Fiduciales")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_title("Trayectorias objetivo XY: dos vueltas contrarias")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper right")
    figure.tight_layout()
    save(figure, output / "trayectorias_objetivo_xy")


def save(figure, stem):
    stem.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(stem.with_suffix(".png"), dpi=180, bbox_inches="tight")
    figure.savefig(stem.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--figure-dir", required=True, type=Path)
    args = parser.parse_args()
    events = parse_events(args.log)
    event_rows = [{"time_s": f"{event['time_s']:.6f}", "marker": event["marker"], "category": category(event["marker"]), "fields_json": json.dumps(event["fields"], sort_keys=True)} for event in events]
    write_csv(args.data_dir / "eventos_integrados.csv", list(event_rows[0]), event_rows)
    counters = plot_counters(events, args.figure_dir)
    queues = queue_rows(events)
    if counters:
        write_csv(args.data_dir / "contadores_pipeline.csv", list(counters[0]), counters)
    if queues:
        write_csv(args.data_dir / "colas_backpressure.csv", list(queues[0]), queues)
    plot_timeline(events, args.figure_dir)
    plot_queues(queues, args.figure_dir)
    plot_target_trajectories(args.figure_dir)
    marker_counts = Counter(event["marker"] for event in events)
    score_positive = score_negative = 0
    for event in events:
        match = SCORE_RE.search(event["payload"])
        if match:
            score_positive += integer(match.group(1))
            score_negative += integer(match.group(2))
    summary = {
        "events": len(events), "marker_counts": dict(sorted(marker_counts.items())),
        "loop_tasks": sum(event["marker"] == "F3O-LOOP-DONE" for event in events),
        "ransac_runs": sum(event["marker"] == "F3O-RANSAC" for event in events),
        "ransac_accepted": sum(event["marker"] == "F3O-RANSAC" and event["fields"].get("accepted") == "true" for event in events),
        "fiducial_graph_builds": sum(event["marker"] == "F3I-GRAPH-BUILD" for event in events),
        "fiducial_optimizations": sum(event["marker"] == "F3J-OPTIMIZE" for event in events),
        "fiducial_atomic_commits": sum(event["marker"] == "F3K-ATOMIC-COMMIT" for event in events),
        "fiducial_stale_commits": sum(event["marker"] == "F3K-COMMIT-STALE" for event in events),
        "loop_optimizations": sum(event["marker"] == "F3Q-LOOP-OPT" for event in events),
        "loop_optimization_starts": sum(event["marker"] == "F3Q-OPT-START" for event in events),
        "loop_optimization_ends": sum(event["marker"] == "F3Q-OPT-END" for event in events),
        "loop_anchors": sum(integer(event["fields"].get("anchors")) for event in events if event["marker"] == "F3O-LOOP-DONE"),
        "fusion_events": sum(event["marker"] == "F3P-FUSION" for event in events),
        "fusion_retries": sum(event["marker"] == "F3P-FUSION-RETRY" for event in events),
        "hard_failures": sum(event["marker"] == "F3L-HARD-FAILURE" for event in events),
        "score_positive_events": score_positive, "score_negative_events": score_negative,
        "queue_samples": len(queues), "backpressure_samples_active": sum(row["backpressure_active"] for row in queues),
    }
    (args.data_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
