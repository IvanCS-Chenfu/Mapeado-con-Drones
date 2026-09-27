#!/usr/bin/env python3
"""Extrae actividad de fusion y score de la prueba 6.6.2."""

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt


TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
MARKER_RE = re.compile(r"\[(F3P-FUSION|F3R-FUSED-SCORE-COMMIT|F3R-RAW-SCORE-COMMIT|F3R-SCORE-STATS)\]\s*(.*)")
FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")
TRACKS_RE = re.compile(r"tracks=\((\d+),(\d+),(\d+)\)")
SCORE_RE = re.compile(r"score=\(\+(\d+),-(\d+),dirty=(\d+)\)")


def integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def parse(path):
    events = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = MARKER_RE.search(line)
        if not match:
            continue
        stamps = TIME_RE.findall(line[:match.start()])
        if not stamps:
            continue
        events.append({
            "time_abs_s": float(stamps[-1]),
            "marker": match.group(1),
            "text": match.group(2),
            "fields": dict(FIELD_RE.findall(match.group(2))),
        })
    if not events:
        raise RuntimeError("El log reducido no contiene eventos F3P/F3R.")
    start = min(event["time_abs_s"] for event in events)
    for event in events:
        event["time_s"] = event["time_abs_s"] - start
    return events


def write_csv(path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fusion_series(events):
    totals = {"pairs": 0, "created": 0, "updated": 0, "retired": 0, "hidden": 0}
    rows = []
    for event in events:
        if event["marker"] != "F3P-FUSION":
            continue
        tracks = TRACKS_RE.search(event["text"])
        fields = event["fields"]
        values = {
            "pairs": integer(fields.get("pairs")),
            "created": integer(tracks.group(1)) if tracks else 0,
            "updated": integer(tracks.group(2)) if tracks else 0,
            "retired": integer(tracks.group(3)) if tracks else 0,
            "hidden": integer(fields.get("hidden")),
        }
        for key, value in values.items():
            totals[key] += value
        rows.append({"time_s": round(event["time_s"], 6), **totals, "committed": fields.get("committed", "")})
    return rows


def evidence_series(events):
    totals = {"positive": 0, "negative": 0, "raw_dirty": 0, "raw_updated": 0}
    rows = []
    for event in events:
        fields = event["fields"]
        if event["marker"] == "F3P-FUSION":
            score = SCORE_RE.search(event["text"])
            if score:
                totals["positive"] += integer(score.group(1))
                totals["negative"] += integer(score.group(2))
                totals["raw_dirty"] += integer(score.group(3))
        elif event["marker"] == "F3R-RAW-SCORE-COMMIT":
            totals["raw_updated"] += integer(fields.get("updated"))
        elif event["marker"] == "F3R-FUSED-SCORE-COMMIT":
            totals["positive"] += integer(fields.get("positive"))
            totals["negative"] += integer(fields.get("negative"))
            totals["raw_dirty"] += integer(fields.get("dirty"))
        else:
            continue
        rows.append({"time_s": round(event["time_s"], 6), **totals, "source": event["marker"]})
    return rows


def stats_series(events):
    rows = []
    for event in events:
        if event["marker"] != "F3R-SCORE-STATS":
            continue
        fields = event["fields"]
        rows.append({
            "time_s": round(event["time_s"], 6),
            "revision": integer(fields.get("revision")),
            "tracked": integer(fields.get("tracked")),
            "bad": integer(fields.get("bad")),
            "anchored": integer(fields.get("anchored")),
            "isolated": integer(fields.get("isolated")),
            "near": integer(fields.get("near")),
            "far": integer(fields.get("far")),
            "score_min": number(fields.get("min")),
            "score_mean": number(fields.get("mean")),
            "score_max": number(fields.get("max")),
        })
    return rows


def plot_fusion(rows, output):
    figure, axis = plt.subplots(figsize=(10.5, 5.0))
    for key, label, color in (
        ("pairs", "Pares procesados", "#1f77b4"),
        ("created", "Tracks creados", "#2ca02c"),
        ("updated", "Tracks actualizados", "#ff7f0e"),
        ("retired", "Tracks retirados", "#9467bd"),
        ("hidden", "Miembros raw ocultados", "#d62728"),
    ):
        axis.step([row["time_s"] for row in rows], [row[key] for row in rows], where="post", label=label, color=color)
    axis.set_xlabel("Tiempo desde el primer evento de fusion [s]")
    axis.set_ylabel("Conteo acumulado")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    save(figure, output / "actividad_fusion")


def plot_evidence(rows, output):
    figure, axis = plt.subplots(figsize=(10.5, 5.0))
    for key, label, color in (
        ("positive", "Evidencia positiva", "#2ca02c"),
        ("negative", "Evidencia negativa", "#d62728"),
        ("raw_dirty", "Cambios raw dirty", "#9467bd"),
        ("raw_updated", "Actualizaciones raw", "#1f77b4"),
    ):
        axis.step([row["time_s"] for row in rows], [row[key] for row in rows], where="post", label=label, color=color)
    axis.set_xlabel("Tiempo desde el primer evento de score [s]")
    axis.set_ylabel("Conteo acumulado")
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    save(figure, output / "evidencia_score")


def plot_stats(rows, output):
    figure, axis = plt.subplots(figsize=(10.5, 5.0))
    time = [row["time_s"] for row in rows]
    for key, label, color in (("score_min", "Score minimo", "#d62728"), ("score_mean", "Score medio", "#1f77b4"), ("score_max", "Score maximo", "#2ca02c")):
        axis.plot(time, [row[key] for row in rows], marker="o", markersize=3, label=label, color=color)
    axis.set_xlabel("Tiempo desde el primer estadistico [s]")
    axis.set_ylabel("Score")
    axis.set_ylim(-0.05, 1.05)
    axis.grid(True, alpha=0.25)
    secondary = axis.twinx()
    secondary.step(time, [row["bad"] for row in rows], where="post", color="#9467bd", label="Puntos bad")
    secondary.step(time, [row["tracked"] for row in rows], where="post", color="#ff7f0e", label="Puntos tracked")
    secondary.set_ylabel("Numero de MapPoints")
    handles, labels = axis.get_legend_handles_labels()
    second_handles, second_labels = secondary.get_legend_handles_labels()
    axis.legend(handles + second_handles, labels + second_labels, loc="best")
    figure.tight_layout()
    save(figure, output / "estadisticos_score")


def save(figure, output_stem):
    output_stem.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_stem.with_suffix(".png"), dpi=180, bbox_inches="tight")
    figure.savefig(output_stem.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--figure-dir", type=Path, required=True)
    args = parser.parse_args()
    events = parse(args.log)
    fusion = fusion_series(events)
    evidence = evidence_series(events)
    stats = stats_series(events)
    if not fusion or not evidence or not stats:
        raise RuntimeError("Falta al menos una familia de eventos necesaria para las tres figuras.")
    write_csv(args.data_dir / "actividad_fusion.csv", list(fusion[0]), fusion)
    write_csv(args.data_dir / "evidencia_score.csv", list(evidence[0]), evidence)
    write_csv(args.data_dir / "estadisticos_score.csv", list(stats[0]), stats)
    summary = {
        "fusion_events": len(fusion),
        "score_evidence_events": len(evidence),
        "score_stats_events": len(stats),
        "final_fusion": fusion[-1],
        "final_evidence": evidence[-1],
        "final_score_stats": stats[-1],
    }
    (args.data_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plot_fusion(fusion, args.figure_dir)
    plot_evidence(evidence, args.figure_dir)
    plot_stats(stats, args.figure_dir)


if __name__ == "__main__":
    main()
