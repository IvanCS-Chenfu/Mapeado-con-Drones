#!/usr/bin/env python3
"""Genera las figuras temporales de la prueba 6.2.4."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def values(rows, name):
    return [float(row[name]) for row in rows]


def save(figure, output_dir, stem):
    figure.tight_layout()
    figure.savefig(output_dir / f"{stem}.png", dpi=180)
    figure.savefig(output_dir / f"{stem}.pdf")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    pipeline = read_csv(args.data_dir / "pipeline_loop.csv")
    figure, axis = plt.subplots(figsize=(10, 5))
    if pipeline:
        x = values(pipeline, "time_s")
        series = (
            ("bow_candidates_cum", "Candidatos BoW"),
            ("ransac_runs_cum", "RANSAC"),
            ("ransac_accepted_cum", "RANSAC aceptado"),
        )
        for field, label in series:
            axis.step(x, values(pipeline, field), where="post", label=label)
        axis.legend(loc="upper left")
    else:
        axis.text(0.5, 0.5, "Sin eventos de pipeline", ha="center", va="center")
    axis.set(xlabel="Tiempo relativo [s]", ylabel="Contador acumulado",
             title="6.2.4 - Pipeline de loop: candidatos y RANSAC")
    axis.grid(True, alpha=0.3)
    save(figure, args.output_dir, "pipeline_loop")

    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                                gridspec_kw={"height_ratios": [2, 1]})
    if pipeline:
        x = values(pipeline, "time_s")
        axes[0].step(x, values(pipeline, "regions_cum"), where="post",
                     color="C0", label="Regiones")
        axes[1].step(x, values(pipeline, "loop_decisions_cum"), where="post",
                     color="C1", label="Decisiones relevantes")
        axes[1].step(x, values(pipeline, "optimizations_cum"), where="post",
                     color="C2", label="Optimizaciones")
        axes[0].legend(loc="upper left")
        axes[1].legend(loc="upper left")
    else:
        axes[0].text(0.5, 0.5, "Sin eventos de pipeline", ha="center", va="center")
    axes[0].set(ylabel="Regiones", title="6.2.4 - Pipeline de loop: regiones y decisiones")
    axes[1].set(xlabel="Tiempo relativo [s]", ylabel="Contador")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    save(figure, args.output_dir, "pipeline_decisiones")

    queues = read_csv(args.data_dir / "colas_backpressure.csv")
    figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True,
                                gridspec_kw={"height_ratios": [3, 1]})
    if queues:
        x = values(queues, "time_s")
        for field, label in (
            ("primary_pending", "Primaria"),
            ("secondary_pending", "Secundaria total"),
            ("secondary_critical", "Secundaria critica"),
        ):
            axes[0].step(x, values(queues, field), where="post", label=label)
        axes[0].axhline(values(queues, "primary_high")[0], color="C0", ls="--",
                        alpha=0.6, label="Watermark primaria")
        axes[0].axhline(values(queues, "secondary_high")[0], color="C2", ls="--",
                        alpha=0.6, label="Watermark secundaria")
        axes[0].legend(loc="upper left")
        axes[1].step(x, values(queues, "optimization_active"), where="post", color="C3")
    else:
        axes[0].text(0.5, 0.5, "Sin muestras de cola", ha="center", va="center")
    axes[0].set(ylabel="Pendientes", title="6.2.4 - Colas y optimizacion")
    axes[1].set(xlabel="Tiempo de simulacion relativo [s]", ylabel="Optimizacion", ylim=(-0.1, 1.1))
    for axis in axes:
        axis.grid(True, alpha=0.3)
    save(figure, args.output_dir, "colas_y_optimizacion")

    figure, axis = plt.subplots(figsize=(10, 3))
    if queues:
        axis.step(values(queues, "time_s"), values(queues, "backpressure_active"),
                  where="post", color="C4")
    else:
        axis.text(0.5, 0.5, "Sin muestras de backpressure", ha="center", va="center")
    axis.set(xlabel="Tiempo de simulacion relativo [s]", ylabel="Activo",
             ylim=(-0.1, 1.1), title="6.2.4 - Backpressure")
    axis.grid(True, alpha=0.3)
    save(figure, args.output_dir, "backpressure")


if __name__ == "__main__":
    main()
