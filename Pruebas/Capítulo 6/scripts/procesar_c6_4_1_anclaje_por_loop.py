#!/usr/bin/env python3
"""Procesa los eventos pasivos de la prueba 6.4.1."""

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt


TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
MARKER_RE = re.compile(r"\[(C6-KF-POSE|F3[A-Z0-9-]+)\]\s*(.*)")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")


def events_from_log(path):
    events = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        marker = MARKER_RE.search(line)
        if not marker:
            continue
        stamps = TIME_RE.findall(line[:marker.start()])
        if not stamps:
            continue
        events.append({
            "time_abs_s": float(stamps[-1]),
            "marker": marker.group(1),
            "text": marker.group(2),
            "fields": dict(FIELD_RE.findall(marker.group(2))),
        })
    if not events:
        raise RuntimeError("No hay eventos F3/C6 en el log reducido.")
    start = min(event["time_abs_s"] for event in events)
    for event in events:
        event["time_s"] = event["time_abs_s"] - start
    return events


def parse_submap(text, field):
    match = re.search(rf"{field}=\((\d+),(\d+)(?:,(\d+))?\)", text)
    return tuple(part for part in match.groups() if part is not None) if match else None


def as_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def write_csv(path, rows, fields):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def relevant_events(events):
    rows = []
    for event in events:
        if event["marker"] not in {
            "F3O-LOOP-DONE", "F3O-RANSAC", "F3O-FID-LOOP-REANCHOR",
            "F3H-FID-POSE-ERROR", "F3K-ATOMIC-COMMIT",
            "F3E-FID-FIRST-ANCHOR",
        }:
            continue
        fields = event["fields"]
        query = parse_submap(event["text"], "query")
        submap = parse_submap(event["text"], "submap")
        rows.append({
            "time_s": round(event["time_s"], 6),
            "marker": event["marker"],
            "drone_id": (query or submap or ("",))[0],
            "query": ",".join(query or ()),
            "submap": ",".join(submap or ()),
            "keyframe_id": fields.get("kf", fields.get("query", "")),
            "decision": fields.get("decision", ""),
            "reason": fields.get("reason", ""),
            "anchors": fields.get("anchors", ""),
            "bow": fields.get("bow", ""),
            "regions": fields.get("regions", ""),
            "geometry": fields.get("geometry", ""),
            "translation": fields.get("translation", ""),
            "rotation": fields.get("rotation", ""),
            "status": fields.get("status", ""),
            "commit": fields.get("commit", ""),
            "revision": fields.get("revision", ""),
            "hard_added": fields.get("hard_added", ""),
        })
    return rows


def kf_rows(events):
    rows = []
    for event in events:
        if event["marker"] != "C6-KF-POSE":
            continue
        fields = event["fields"]
        if "drone_id" not in fields:
            continue
        rows.append({
            "time_s": round(event["time_s"], 6),
            "sim_time_ns": fields.get("sim_time_ns", ""),
            "drone_id": fields.get("drone_id", ""),
            "epoch": fields.get("epoch", ""),
            "keyframe_id": fields.get("kf", ""),
            "pose_revision": fields.get("pose_revision", ""),
            "x": fields.get("x", ""),
            "y": fields.get("y", ""),
            "z": fields.get("z", ""),
        })
    return rows


def find_authority_events(events):
    loop_anchor = next((event for event in events if event["marker"] == "F3O-LOOP-DONE"
                        and parse_submap(event["text"], "query")
                        and parse_submap(event["text"], "query")[0] == "2"
                        and event["fields"].get("decision") == "anchor_proposed"
                        and as_int(event["fields"].get("anchors")) > 0), None)
    fiducial = next((event for event in events if event["marker"] == "F3H-FID-POSE-ERROR"
                     and parse_submap(event["text"], "submap")
                     and parse_submap(event["text"], "submap")[0] == "2"), None)
    hard_commit = next((event for event in events if event["marker"] == "F3K-ATOMIC-COMMIT"
                        and as_int(event["fields"].get("hard_added")) > 0
                        and (fiducial is None or event["time_s"] >= fiducial["time_s"])), None)
    explicit_reanchor = next((event for event in events if event["marker"] == "F3O-FID-LOOP-REANCHOR"), None)
    return loop_anchor, fiducial, hard_commit, explicit_reanchor


def plot_timeline(loop_anchor, fiducial, hard_commit, explicit_reanchor, figure_dir):
    events = [(0.0, "NO ANCLADO", "#7f7f7f")]
    if loop_anchor:
        events.append((loop_anchor["time_s"], "ANCLADO POR LOOP", "#1f77b4"))
    if fiducial:
        events.append((fiducial["time_s"], "FIDUCIAL OBSERVADO", "#ff7f0e"))
    if hard_commit:
        events.append((hard_commit["time_s"], "COMMIT FIDUCIAL", "#2ca02c"))
    if explicit_reanchor:
        events.append((explicit_reanchor["time_s"], "REANCLAJE EXPLICITO", "#9467bd"))
    end = max(time for time, _, _ in events) + 8.0
    figure, axis = plt.subplots(figsize=(10.5, 2.8))
    for index, (time, label, color) in enumerate(events):
        right = events[index + 1][0] if index + 1 < len(events) else end
        axis.broken_barh([(time, max(right - time, 0.1))], (8, 8), facecolors=color, label=label)
        axis.axvline(time, color=color, linewidth=1.1)
    axis.set_ylim(4, 22)
    axis.set_yticks([])
    axis.set_xlabel("Tiempo desde el primer evento cartografico [s]")
    axis.grid(True, axis="x", alpha=0.25)
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), ncol=2, loc="upper center", bbox_to_anchor=(0.5, 1.35))
    figure.tight_layout()
    figure_dir.mkdir(parents=True, exist_ok=True)
    figure.savefig(figure_dir / "timeline_autoridad_global_d2.png", dpi=180, bbox_inches="tight")
    figure.savefig(figure_dir / "timeline_autoridad_global_d2.pdf", bbox_inches="tight")
    plt.close(figure)


def plot_xy(kfs, loop_anchor, figure_dir):
    if not loop_anchor:
        return
    by_revision = {}
    for row in kfs:
        if row["time_s"] < loop_anchor["time_s"]:
            continue
        by_revision.setdefault(row["pose_revision"], []).append(row)
    snapshots = list(by_revision.values())
    d2_snapshots = [snapshot for snapshot in snapshots if any(row["drone_id"] == "2" for row in snapshot)]
    if not d2_snapshots:
        return
    selected = [d2_snapshots[0], d2_snapshots[-1]]
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.4), sharex=True, sharey=True)
    for axis, snapshot, title in zip(axes, selected, ("Primera publicacion tras anclaje", "Ultima publicacion con KFs de D2")):
        for drone, color, label in (("1", "#1f77b4", "D1"), ("2", "#d62728", "D2")):
            points = [(float(row["x"]), float(row["y"])) for row in snapshot if row["drone_id"] == drone]
            if points:
                axis.scatter(*zip(*points), s=18, color=color, label=label)
        axis.set_title(title)
        axis.set_xlabel("X [m]")
        axis.grid(True, alpha=0.25)
        axis.set_aspect("equal", adjustable="box")
    axes[0].set_ylabel("Y [m]")
    axes[0].legend()
    figure.tight_layout()
    figure.savefig(figure_dir / "mapa_xy_despues_anclaje_loop.png", dpi=180, bbox_inches="tight")
    figure.savefig(figure_dir / "mapa_xy_despues_anclaje_loop.pdf", bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--figure-dir", required=True, type=Path)
    args = parser.parse_args()
    events = events_from_log(args.log)
    observations = relevant_events(events)
    kfs = kf_rows(events)
    loop_anchor, fiducial, hard_commit, explicit_reanchor = find_authority_events(events)
    write_csv(args.data_dir / "eventos_loop_y_fiducial.csv", observations, list(observations[0]))
    write_csv(args.data_dir / "keyframes_globales.csv", kfs, list(kfs[0]))
    authority = [
        {"state": "NO_ANCLADO", "time_s": 0.0, "evidence": "inicio de la ejecucion"},
    ]
    for event, state, evidence in (
        (loop_anchor, "ANCLADO_POR_LOOP", "F3O-LOOP-DONE anchor_proposed"),
        (fiducial, "FIDUCIAL_OBSERVADO", "F3H-FID-POSE-ERROR D2"),
        (hard_commit, "COMMIT_FIDUCIAL", "F3K-ATOMIC-COMMIT hard_added>0"),
        (explicit_reanchor, "REANCLAJE_EXPLICITO", "F3O-FID-LOOP-REANCHOR"),
    ):
        if event:
            authority.append({"state": state, "time_s": round(event["time_s"], 6), "evidence": evidence})
    write_csv(args.data_dir / "timeline_autoridad_d2.csv", authority, list(authority[0]))
    summary = {
        "loop_anchor_observed": bool(loop_anchor),
        "loop_anchor_time_s": loop_anchor["time_s"] if loop_anchor else None,
        "first_d2_fiducial_time_s": fiducial["time_s"] if fiducial else None,
        "hard_fiducial_commit_time_s": hard_commit["time_s"] if hard_commit else None,
        "explicit_fid_loop_reanchor_observed": bool(explicit_reanchor),
        "d2_keyframe_telemetry_rows": sum(row["drone_id"] == "2" for row in kfs),
    }
    (args.data_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plot_timeline(loop_anchor, fiducial, hard_commit, explicit_reanchor, args.figure_dir)
    plot_xy(kfs, loop_anchor, args.figure_dir)


if __name__ == "__main__":
    main()
