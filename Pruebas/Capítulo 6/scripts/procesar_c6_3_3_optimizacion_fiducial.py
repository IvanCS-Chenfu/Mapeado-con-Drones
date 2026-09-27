#!/usr/bin/env python3
"""Procesa la evidencia pasiva de la prueba 6.3.3."""

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LOG_TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
MARKER_RE = re.compile(r"\[(C6-KF-POSE|F3[A-Z0-9-]+)\]\s*(.*)")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")
TRIPLE_RE = re.compile(r"(before|after|final)=\(([^,]+),([^,]+),([^\)]+)\)")


def parse_log(path):
    events = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        marker = MARKER_RE.search(line)
        if not marker:
            continue
        stamps = LOG_TIME_RE.findall(line[:marker.start()])
        if not stamps:
            continue
        payload = marker.group(2)
        triples = {
            name: tuple(float(value) for value in values)
            for name, *values in TRIPLE_RE.findall(payload)
        }
        events.append({
            "time_s": float(stamps[-1]),
            "marker": marker.group(1),
            "fields": dict(FIELD_RE.findall(payload)),
            "triples": triples,
            "raw": line,
        })
    return events


def float_field(fields, name, default=math.nan):
    try:
        return float(fields[name])
    except (KeyError, ValueError):
        return default


def int_field(fields, name, default=0):
    try:
        return int(fields[name])
    except (KeyError, ValueError):
        return default


def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def read_gt(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append({key: float(value) for key, value in row.items()})
    if len(rows) < 2:
        raise RuntimeError("El CSV GT necesita al menos dos muestras.")
    rows.sort(key=lambda row: row["stamp_sec"])
    return rows


def interpolate_gt(rows, stamp_ns):
    stamp_sec = stamp_ns / 1_000_000_000.0
    times = [row["stamp_sec"] for row in rows]
    if stamp_sec < times[0] or stamp_sec > times[-1]:
        return None
    fields = ("x", "y", "z", "qx", "qy", "qz", "qw")
    values = {
        field: float(np.interp(stamp_sec, times, [row[field] for row in rows]))
        for field in fields
    }
    norm = math.sqrt(sum(values[field] ** 2 for field in ("qx", "qy", "qz", "qw")))
    for field in ("qx", "qy", "qz", "qw"):
        values[field] /= norm
    values["stamp_sec"] = stamp_sec
    values["yaw_rad"] = yaw_from_quaternion(
        values["qx"], values["qy"], values["qz"], values["qw"])
    return values


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save_figure(figure, directory, name):
    directory.mkdir(parents=True, exist_ok=True)
    figure.savefig(directory / f"{name}.png", dpi=180, bbox_inches="tight")
    figure.savefig(directory / f"{name}.pdf", bbox_inches="tight")
    plt.close(figure)


def process_keyframes(events, gt_rows):
    samples = []
    for event in events:
        if event["marker"] != "C6-KF-POSE":
            continue
        fields = event["fields"]
        gt = interpolate_gt(gt_rows, int_field(fields, "kf_stamp_ns"))
        if gt is None:
            continue
        sample = {
            "event_time_s": event["time_s"],
            "sim_time_s": int_field(fields, "sim_time_ns") / 1_000_000_000.0,
            "drone_id": int_field(fields, "drone_id"),
            "map_epoch": int_field(fields, "epoch"),
            "keyframe_id": int_field(fields, "kf"),
            "keyframe_stamp_s": gt["stamp_sec"],
            "pose_revision": int_field(fields, "pose_revision"),
            "x": float_field(fields, "x"),
            "y": float_field(fields, "y"),
            "z": float_field(fields, "z"),
            "qx": float_field(fields, "qx"),
            "qy": float_field(fields, "qy"),
            "qz": float_field(fields, "qz"),
            "qw": float_field(fields, "qw"),
            "gt_x": gt["x"],
            "gt_y": gt["y"],
            "gt_z": gt["z"],
            "gt_yaw_rad": gt["yaw_rad"],
        }
        sample["yaw_rad"] = yaw_from_quaternion(
            sample["qx"], sample["qy"], sample["qz"], sample["qw"])
        sample["position_error_m"] = math.dist(
            (sample["x"], sample["y"], sample["z"]),
            (sample["gt_x"], sample["gt_y"], sample["gt_z"]))
        samples.append(sample)
    return samples


def optimization_rows(events, start_time):
    grouped = defaultdict(dict)
    fiducials = []
    for event in events:
        if event["marker"] == "F3H-FID-POSE-ERROR":
            fields = event["fields"]
            fiducials.append({
                "time_s": event["time_s"] - start_time,
                "fiducial_id": int_field(fields, "fid"),
                "keyframe_id": int_field(fields, "kf"),
                "visit": int_field(fields, "visit"),
                "translation_error_m": float_field(fields, "translation"),
                "rotation_error_rad": float_field(fields, "rotation"),
                "yaw_error_rad": float_field(fields, "yaw"),
                "status": fields.get("status", ""),
                "reason": fields.get("reason", ""),
            })
        task = event["fields"].get("task")
        if task and event["marker"] in {
            "F3I-GRAPH-BUILD", "F3J-OPTIMIZE", "F3L-VALIDATE", "F3K-ATOMIC-COMMIT"
        }:
            grouped[task][event["marker"]] = event
    rows = []
    for task, group in sorted(grouped.items(), key=lambda item: int(item[0])):
        graph = group.get("F3I-GRAPH-BUILD")
        optimize = group.get("F3J-OPTIMIZE")
        validate = group.get("F3L-VALIDATE")
        commit = group.get("F3K-ATOMIC-COMMIT")
        fields = (graph or optimize or validate or commit)["fields"]
        before = optimize["triples"].get("before", (math.nan,) * 3) if optimize else (math.nan,) * 3
        after = optimize["triples"].get("after", (math.nan,) * 3) if optimize else (math.nan,) * 3
        final = validate["triples"].get("final", (math.nan,) * 3) if validate else (math.nan,) * 3
        rows.append({
            "task_id": task,
            "pass": int_field(fields, "pass"),
            "time_s": (graph or optimize or validate or commit)["time_s"] - start_time,
            "commit_time_s": commit["time_s"] - start_time if commit else math.nan,
            "control_kf": int_field(graph["fields"], "control") if graph else "",
            "target_kf": int_field(graph["fields"], "target") if graph else "",
            "window_size": int_field(graph["fields"], "window") if graph else "",
            "controls": int_field(graph["fields"], "controls") if graph else "",
            "temporal_edges": int_field(graph["fields"], "edges") if graph else "",
            "translation_error_before": before[0],
            "rotation_error_before": before[1],
            "yaw_error_before": before[2],
            "translation_error_after": after[0],
            "rotation_error_after": after[1],
            "yaw_error_after": after[2],
            "validation_translation_error": final[0],
            "validation_rotation_error": final[1],
            "validation_yaw_error": final[2],
            "validation_decision": validate["fields"].get("decision", "") if validate else "",
            "full_commit": commit["fields"].get("full", "") if commit else "",
            "moved_kfs": int_field(commit["fields"], "moved_kfs") if commit else "",
            "propagated_kfs": int_field(commit["fields"], "control_propagated") if commit else "",
            "commit_id": int_field(commit["fields"], "commit") if commit else "",
            "pose_revision": int_field(commit["fields"], "revision") if commit else "",
        })
    return fiducials, rows


def write_tables(rows, directory):
    fields = list(rows[0]) if rows else ["task_id", "pass"]
    write_csv(directory / "tabla_optimizacion_fiducial.csv", fields, rows)
    markdown = ["| " + " | ".join(fields) + " |", "|" + "|".join(["---"] * len(fields)) + "|"]
    for row in rows:
        markdown.append("| " + " | ".join(str(row[field]) for field in fields) + " |")
    (directory / "tabla_optimizacion_fiducial.md").write_text(
        "\n".join(markdown) + "\n", encoding="utf-8")
    latex = ["\\begin{tabular}{" + "l" * len(fields) + "}", " & ".join(fields) + " \\\\ \\hline"]
    for row in rows:
        latex.append(" & ".join(str(row[field]) for field in fields) + " " + chr(92) * 2)
    latex.append("\\end{tabular}")
    (directory / "tabla_optimizacion_fiducial.tex").write_text(
        "\n".join(latex) + "\n", encoding="utf-8")


def build_live_error_timeline(samples):
    """Reconstruye el estado vigente del mapa despues de cada publicacion."""
    batches = defaultdict(list)
    for row in samples:
        batches[(row["sim_time_s"], row["pose_revision"])].append(row)
    start = min(row["event_time_s"] for row in samples)
    live_state = {}
    timeline = []
    for _, rows in sorted(batches.items(), key=lambda item: min(
            row["event_time_s"] for row in item[1])):
        for row in rows:
            key = (row["drone_id"], row["map_epoch"], row["keyframe_id"])
            live_state[key] = row
        errors = np.array([row["position_error_m"] for row in live_state.values()])
        timeline.append({
            "time_s": min(row["event_time_s"] for row in rows) - start,
            "pose_revision": rows[0]["pose_revision"],
            "keyframes_in_map": len(errors),
            "position_error_sum_m": float(np.sum(errors)),
            "position_error_mean_m": float(np.mean(errors)),
            "position_error_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
            "position_error_max_m": float(np.max(errors)),
        })
    return timeline


def plot_error_timeline(timeline, fiducials, optimizations, directory):
    if not timeline:
        return
    commit_times = [row["commit_time_s"] for row in optimizations
                    if math.isfinite(row["commit_time_s"])]
    if not commit_times:
        return
    commit_time = commit_times[0]
    figure, axis = plt.subplots(figsize=(10.5, 4.8))
    axis.plot([row["time_s"] for row in timeline],
              [row["position_error_sum_m"] for row in timeline],
              color="#1f77b4", linewidth=2.0,
              label="Error acumulado del mapa vivo")
    for item in fiducials:
        if item["status"] == "optimization_required":
            axis.axvline(item["time_s"], color="#2ca02c", linestyle="--", alpha=0.7,
                        label="Fiducial que requiere optimizacion")
    axis.axvline(commit_time, color="#9467bd", linestyle=":", alpha=0.9,
                    label="Commit de optimizacion")
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc="upper left")
    axis.set_xlabel("Tiempo desde la primera pose de KF [s]")
    axis.set_ylabel("Error de posicion acumulado frente a GT [m]")
    axis.grid(True, alpha=0.25)
    save_figure(figure, directory, "error_cartografico_temporal")


def plot_xy(samples, directory):
    grouped = defaultdict(list)
    for row in samples:
        grouped[(row["drone_id"], row["map_epoch"], row["keyframe_id"])].append(row)
    initial = [min(rows, key=lambda row: row["event_time_s"]) for rows in grouped.values()]
    final = [max(rows, key=lambda row: row["event_time_s"]) for rows in grouped.values()]
    figure, axis = plt.subplots(figsize=(7.2, 7.2))
    axis.scatter([row["gt_x"] for row in final], [row["gt_y"] for row in final],
                 label="GT de KFs", marker="x", color="#111111")
    axis.scatter([row["x"] for row in initial], [row["y"] for row in initial],
                 label="Servidor, primera publicación", s=22, color="#d62728")
    axis.scatter([row["x"] for row in final], [row["y"] for row in final],
                 label="Servidor, última publicación", s=22, color="#1f77b4")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best")
    save_figure(figure, directory, "keyframes_xy_gt_inicial_final")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--gt", required=True, type=Path)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--figure-dir", required=True, type=Path)
    args = parser.parse_args()

    events = parse_log(args.log)
    if not events:
        raise SystemExit("No se encontraron eventos F3 o C6 en el log reducido.")
    gt_rows = read_gt(args.gt)
    samples = process_keyframes(events, gt_rows)
    if not samples:
        raise SystemExit("No se pudieron sincronizar poses de KFs con GT.")
    start = min(row["event_time_s"] for row in samples)
    for row in samples:
        row["time_s"] = row["event_time_s"] - start
    fields = list(samples[0])
    write_csv(args.data_dir / "keyframes_con_gt.csv", fields, samples)
    fiducials, optimizations = optimization_rows(events, start)
    write_csv(args.data_dir / "fiduciales.csv", list(fiducials[0]) if fiducials else ["time_s"], fiducials)
    write_tables(optimizations, args.data_dir)
    live_timeline = build_live_error_timeline(samples)
    write_csv(args.data_dir / "error_mapa_vivo_temporal.csv", list(live_timeline[0]), live_timeline)
    plot_error_timeline(live_timeline, fiducials, optimizations, args.figure_dir)
    plot_xy(samples, args.figure_dir)
    final_by_kf = defaultdict(list)
    for row in samples:
        final_by_kf[(row["drone_id"], row["map_epoch"], row["keyframe_id"])].append(row)
    final_errors = np.array([
        max(rows, key=lambda row: row["event_time_s"])["position_error_m"]
        for rows in final_by_kf.values()
    ])
    summary = {
        "keyframe_pose_samples": len(samples),
        "unique_keyframes": len(final_by_kf),
        "gt_samples": len(gt_rows),
        "position_rmse_m_final": float(np.sqrt(np.mean(final_errors ** 2))),
        "position_mean_m_final": float(np.mean(final_errors)),
        "position_max_m_final": float(np.max(final_errors)),
        "fiducial_events": len(fiducials),
        "optimization_passes": len(optimizations),
    }
    (args.data_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
