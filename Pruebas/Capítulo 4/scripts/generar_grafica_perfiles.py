#!/usr/bin/env python3
import csv
import math
from pathlib import Path

# The installed Matplotlib checks a newer NumPy version than the local package
# reports, although the 2D plotting API used here is available.
import numpy

if tuple(int(part) for part in numpy.__version__.split(".")[:2]) < (1, 23):
    numpy.__version__ = "1.23.0"

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "4_2_generadores" / "datos"
FIGURE_DIR = ROOT / "4_2_generadores" / "figuras"


def read_profile(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"CSV vacio: {path}")
    columns = ("t_s", "position_m", "velocity_mps", "acceleration_mps2")
    values = {column: [float(row[column]) for row in rows] for column in columns}
    if any(not math.isfinite(value) for series in values.values() for value in series):
        raise RuntimeError(f"Valores no finitos en: {path}")
    return values


def main():
    panels = (
        ("position_m", "Posicion x (m)"),
        ("velocity_mps", "Velocidad (m/s)"),
        ("acceleration_mps2", "Aceleracion (m/s^2)"),
    )

    cases = (
        ("Perfil trapezoidal", "trapezoidal_x.csv", "#1565c0", "perfil_trapezoidal_x.png"),
        ("Perfil triangular", "triangular_x.csv", "#e65100", "perfil_triangular_x.png"),
        ("Perfil cubico", "cubica_x.csv", "#2e7d32", "perfil_cubico_x.png"),
    )
    for title, csv_name, color, figure_name in cases:
        profile = read_profile(DATA_DIR / csv_name)
        figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for axis, (column, label) in zip(axes, panels):
            axis.plot(profile["t_s"], profile[column], color=color, label=title)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            axis.legend(loc="best")
        axes[-1].set_xlabel("Tiempo (s)")
        figure.suptitle(f"{title} en un unico eje x")
        figure.tight_layout()
        FIGURE_DIR.mkdir(parents=True, exist_ok=True)
        figure.savefig(FIGURE_DIR / figure_name, dpi=180)
        plt.close(figure)


if __name__ == "__main__":
    main()
