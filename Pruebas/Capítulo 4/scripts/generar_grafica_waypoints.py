#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import numpy

if tuple(int(part) for part in numpy.__version__.split(".")[:2]) < (1, 23):
    numpy.__version__ = "1.23.0"

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "4_2_generadores" / "datos"
FIGURE_DIR = ROOT / "4_2_generadores" / "figuras"


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    cases = (
        ("waypoints_cubica", "#2e7d32", "Blend = 5 s", "waypoints_cubica_xy.png"),
        (
            "waypoints_cubica_blend_1s",
            "#1565c0",
            "Blend = 1 s",
            "waypoints_cubica_blend_1s_xy.png",
        ),
    )
    for stem, color, blend_label, figure_name in cases:
        trajectory = read_rows(DATA_DIR / f"{stem}.csv")
        targets = read_rows(DATA_DIR / f"{stem}_objetivos.csv")
        x = [float(row["x_m"]) for row in trajectory]
        y = [float(row["y_m"]) for row in trajectory]
        if any(not math.isfinite(value) for value in x + y):
            raise RuntimeError(f"La trayectoria contiene valores no finitos: {stem}")

        figure, axis = plt.subplots(figsize=(9, 8))
        axis.plot(x, y, color=color, linewidth=2.0, label="Trayectoria cubica")
        target_x = [float(row["x_m"]) for row in targets]
        target_y = [float(row["y_m"]) for row in targets]
        axis.scatter(target_x, target_y, color="#c62828", zorder=3, label="Waypoints")
        for row, point_x, point_y in zip(targets, target_x, target_y):
            axis.annotate(
                f"WP {row['waypoint_index']}",
                (point_x, point_y),
                xytext=(6, 6),
                textcoords="offset points",
            )
        axis.set_title(f"Trayectoria cubica por waypoints en XY ({blend_label})")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
        figure.tight_layout()
        FIGURE_DIR.mkdir(parents=True, exist_ok=True)
        figure.savefig(FIGURE_DIR / figure_name, dpi=180)
        plt.close(figure)


if __name__ == "__main__":
    main()
