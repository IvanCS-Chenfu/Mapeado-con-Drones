#!/usr/bin/env python3
"""Sincroniza y grafica una ejecución de seguimiento de la prueba 4.5."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


AXES = ("x", "y", "z", "yaw")
ROWS = ("pose", "velocidad", "aceleracion")


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"CSV vacio: {path}")
    result = {}
    for key in rows[0]:
        if key in {"trajectory_id"}:
            result[key] = np.array([row[key] for row in rows], dtype=object)
        else:
            try:
                result[key] = np.array([float(row[key]) for row in rows], dtype=float)
            except ValueError:
                result[key] = np.array([row[key] for row in rows], dtype=object)
    return result


def unique_sorted(data, time_key="receive_sec"):
    order = np.argsort(data[time_key])
    result = {}
    last = None
    selected = []
    for index in order:
        value = float(data[time_key][index])
        if last is None or value > last:
            selected.append(index)
            last = value
    for key, values in data.items():
        result[key] = values[selected]
    return result


def interp(data, timeline, names):
    return {
        name: np.interp(timeline, data["receive_sec"], data[name])
        for name in names
    }


def nearest(data, timeline, names):
    source_time = data["receive_sec"]
    indices = np.searchsorted(source_time, timeline, side="left")
    indices = np.clip(indices, 0, len(source_time) - 1)
    previous = np.maximum(indices - 1, 0)
    choose_previous = np.abs(timeline - source_time[previous]) <= np.abs(
        source_time[indices] - timeline)
    indices = np.where(choose_previous, previous, indices)
    return {name: data[name][indices] for name in names}


def wrap_degrees(values):
    return (values + 180.0) % 360.0 - 180.0


def write_csv(path, timeline, columns):
    path.parent.mkdir(parents=True, exist_ok=True)
    names = ["time_sec"] + list(columns)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index, time_value in enumerate(timeline):
            writer.writerow([f"{time_value:.9f}"] + [f"{columns[name][index]:.9f}" for name in columns])


def metric(values):
    values = np.asarray(values, dtype=float)
    return {
        "rmse": float(np.sqrt(np.mean(values * values))),
        "mae": float(np.mean(np.abs(values))),
        "max": float(np.max(np.abs(values))),
    }


def plot_xy(path, data, title):
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(data["gt_pose_x"], data["gt_pose_y"], label="GT", linewidth=1.8)
    ax.plot(data["tray_pose_x"], data["tray_pose_y"], label="Trayectoria", linewidth=1.4)
    ax.scatter(data["tray_pose_x"][::max(1, len(data["tray_pose_x"]) // 12)],
               data["tray_pose_y"][::max(1, len(data["tray_pose_y"]) // 12)],
               s=12, label="Muestras trayectoria")
    ax.set(title=title, xlabel="X [m]", ylabel="Y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_comparison(path, data, title):
    labels = {
        "pose": ("Posición", "m", ("x", "y", "z", "yaw")),
        "velocidad": ("Velocidad", "m/s; yaw: deg/s", ("x", "y", "z", "yaw")),
        "aceleracion": ("Aceleración", "m/s²; yaw: deg/s²", ("x", "y", "z", "yaw")),
    }
    fig, axes = plt.subplots(3, 4, figsize=(18, 10), sharex=False)
    for row_index, row_name in enumerate(ROWS):
        for col_index, axis_name in enumerate(AXES):
            ax = axes[row_index, col_index]
            ax.plot(data["gt_" + row_name + "_" + axis_name], label="GT", linewidth=1.0)
            ax.plot(data["tray_" + row_name + "_" + axis_name], label="Trayectoria", linewidth=1.0)
            ax.set_title(f"{labels[row_name][0]} {axis_name}")
            ax.set_ylabel(labels[row_name][1])
            ax.grid(True, alpha=0.25)
            if row_index == 2:
                ax.set_xlabel("Muestra sincronizada")
            if row_index == 0 and col_index == 0:
                ax.legend(fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_errors(path, data, title):
    units = {
        "pose": "m; yaw: deg",
        "velocidad": "m/s; yaw: deg/s",
        "aceleracion": "m/s²; yaw: deg/s²",
    }
    fig, axes = plt.subplots(3, 4, figsize=(18, 10), sharex=False)
    for row_index, row_name in enumerate(ROWS):
        for col_index, axis_name in enumerate(AXES):
            ax = axes[row_index, col_index]
            key = "error_" + row_name + "_" + axis_name
            ax.plot(data[key], color="#b22222", linewidth=1.0)
            ax.axhline(0.0, color="black", linewidth=0.6)
            ax.set_title(f"Error {row_name} {axis_name}")
            ax.set_ylabel(units[row_name])
            ax.grid(True, alpha=0.25)
            if row_index == 2:
                ax.set_xlabel("Muestra sincronizada")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def process_profile(profile_dir, profile, data_profile=None, action_duration=None):
    variant = data_profile or profile
    data_dir = profile_dir / "datos" / variant
    figure_dir = profile_dir / "figuras" / variant
    result_dir = profile_dir / "resultados"
    figure_dir.mkdir(parents=True, exist_ok=True)
    result_dir.mkdir(parents=True, exist_ok=True)
    pose = unique_sorted(read_csv(data_dir / "gt_pose.csv"))
    velocity = unique_sorted(read_csv(data_dir / "gt_velocity.csv"))
    acceleration = unique_sorted(read_csv(data_dir / "gt_acceleration.csv"))
    tray = unique_sorted(read_csv(data_dir / "trajectory_feedback.csv"))

    start = max(
        pose["receive_sec"][0], velocity["receive_sec"][0],
        acceleration["receive_sec"][0], tray["receive_sec"][0])
    end = min(
        pose["receive_sec"][-1], velocity["receive_sec"][-1],
        acceleration["receive_sec"][-1], tray["receive_sec"][-1])
    if end <= start:
        raise RuntimeError(f"No hay intervalo comun para {variant}: {start}..{end}")
    timeline = pose["receive_sec"][(pose["receive_sec"] >= start) & (pose["receive_sec"] <= end)]
    if len(timeline) < 20:
        raise RuntimeError(f"Muy pocas muestras sincronizadas para {variant}: {len(timeline)}")

    gt_pose = interp(pose, timeline, ["x", "y", "z", "yaw_rad"])
    gt_velocity = interp(velocity, timeline,
                         ["linear_x", "linear_y", "linear_z", "angular_z"])
    gt_acceleration = interp(acceleration, timeline,
                             ["linear_x", "linear_y", "linear_z", "angular_z"])
    tray_names = [f"{axis}_{suffix}" for axis in ("x", "y", "z", "yaw")
                  for suffix in ("pos", "vel", "acc")]
    tray_values = interp(tray, timeline, tray_names)
    tray_values.update(nearest(
        tray, timeline, ["yaw_pos", "yaw_vel", "yaw_acc"]))

    columns = {}
    for axis, source in zip(("x", "y", "z"), ("x", "y", "z")):
        columns[f"gt_pose_{axis}"] = gt_pose[source]
        columns[f"gt_velocidad_{axis}"] = gt_velocity["linear_" + source]
        columns[f"gt_aceleracion_{axis}"] = gt_acceleration["linear_" + source]
        columns[f"tray_pose_{axis}"] = tray_values[f"{axis}_pos"]
        columns[f"tray_velocidad_{axis}"] = tray_values[f"{axis}_vel"]
        columns[f"tray_aceleracion_{axis}"] = tray_values[f"{axis}_acc"]
    gt_yaw = np.unwrap(gt_pose["yaw_rad"]) * 180.0 / math.pi
    tray_yaw = np.unwrap(tray_values["yaw_pos"]) * 180.0 / math.pi
    columns["gt_pose_yaw"] = gt_yaw
    columns["gt_velocidad_yaw"] = gt_velocity["angular_z"] * 180.0 / math.pi
    columns["gt_aceleracion_yaw"] = gt_acceleration["angular_z"] * 180.0 / math.pi
    columns["tray_pose_yaw"] = tray_yaw
    columns["tray_velocidad_yaw"] = tray_values["yaw_vel"] * 180.0 / math.pi
    columns["tray_aceleracion_yaw"] = tray_values["yaw_acc"] * 180.0 / math.pi

    for row_name in ROWS:
        for axis_name in AXES:
            columns[f"error_{row_name}_{axis_name}"] = (
                columns[f"tray_{row_name}_{axis_name}"] - columns[f"gt_{row_name}_{axis_name}"])
    columns["error_pose_yaw"] = wrap_degrees(columns["error_pose_yaw"])

    write_csv(data_dir / "synchronized.csv", timeline, columns)
    position_error = np.column_stack([
        columns["error_pose_x"], columns["error_pose_y"], columns["error_pose_z"]])
    position_norm = np.linalg.norm(position_error, axis=1)
    metrics = {
        "profile": variant,
        "trajectory_type": profile,
        "sample_count": int(len(timeline)),
        "duration_sec": float(timeline[-1] - timeline[0]),
        "action_duration_sec": None if action_duration is None else float(action_duration),
        "rmse_position_m": float(np.sqrt(np.mean(position_norm ** 2))),
        "mae_position_m": float(np.mean(np.abs(position_norm))),
        "max_position_error_m": float(np.max(np.abs(position_norm))),
        "final_position_error_m": float(position_norm[-1]),
        "axis_metrics": {
            row_name: {axis_name: metric(columns[f"error_{row_name}_{axis_name}"])
                       for axis_name in AXES}
            for row_name in ROWS
        },
    }
    with (result_dir / f"{variant}_metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(metrics, stream, indent=2, ensure_ascii=False)
    plot_xy(figure_dir / "xy_gt_vs_trayectoria.png", columns, f"4.5 {variant}: plano XY")
    plot_comparison(figure_dir / "gt_vs_trayectoria_3x4.png", columns,
                    f"4.5 {variant}: GT frente a trayectoria")
    plot_errors(figure_dir / "errores_3x4.png", columns, f"4.5 {variant}: errores")
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


def write_table(profile_dir):
    result_dir = profile_dir / "resultados"
    variants = (
        ("cubica_lenta", "Cúbica lenta"),
        ("cubica_normal", "Cúbica normal"),
        ("trapezoidal_lenta", "Trapezoidal lenta"),
        ("trapezoidal_rapida", "Trapezoidal rápida"),
    )
    rows = []
    for variant, label in variants:
        path = result_dir / f"{variant}_metrics.json"
        with path.open(encoding="utf-8") as stream:
            metrics = json.load(stream)
        rows.append((label, metrics))
    table_csv = result_dir / "tabla_4_5.csv"
    with table_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["Metrica"] + [label for label, _ in rows])
        for key, label, unit in (
            ("rmse_position_m", "RMSE de posicion", "m"),
            ("mae_position_m", "MAE de posicion", "m"),
            ("max_position_error_m", "Error maximo de posicion", "m"),
            ("final_position_error_m", "Error final de posicion", "m"),
            ("action_duration_sec", "Suma de t_total de los goals", "s"),
            ("duration_sec", "Duracion total del recorrido", "s"),
        ):
            writer.writerow([f"{label} [{unit}]" ] + [f"{metrics[key]:.6f}" for _, metrics in rows])
    table_md = result_dir / "tabla_4_5.md"
    with table_md.open("w", encoding="utf-8") as stream:
        stream.write("| Métrica | " + " | ".join(label for label, _ in rows) + " |\n")
        stream.write("|---|" + "---:|" * len(rows) + "\n")
        for key, label, unit in (
            ("rmse_position_m", "RMSE de posición", "m"),
            ("mae_position_m", "MAE de posición", "m"),
            ("max_position_error_m", "Error máximo de posición", "m"),
            ("final_position_error_m", "Error final de posición", "m"),
            ("action_duration_sec", "Suma de t_total de los goals", "s"),
            ("duration_sec", "Duración total del recorrido", "s"),
        ):
            stream.write(
                f"| {label} [{unit}] | " +
                " | ".join(f"{metrics[key]:.6f}" for _, metrics in rows) +
                " |\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile-dir", type=Path, required=True)
    parser.add_argument("--profile", choices=("cubica", "trapezoidal"))
    parser.add_argument("--data-profile")
    parser.add_argument("--action-duration", type=float)
    parser.add_argument("--build-table", action="store_true")
    args = parser.parse_args()
    if args.profile:
        process_profile(args.profile_dir, args.profile, args.data_profile, args.action_duration)
    if args.build_table:
        write_table(args.profile_dir)


if __name__ == "__main__":
    main()
