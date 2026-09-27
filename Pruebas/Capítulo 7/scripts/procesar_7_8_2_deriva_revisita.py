#!/usr/bin/env python3
"""Procesa la prueba 7.8.2: ORB largo, revisita fiducial y correccion global."""
import argparse
import bisect
import csv
import json
import math
import os
import re
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-chapter7")


def number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return -1


def finite(value):
    return math.isfinite(value)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def nearest(rows, times, instant, key, max_skew):
    if not rows:
        return None
    index = bisect.bisect_left(times, instant)
    options = []
    if index < len(rows):
        options.append(rows[index])
    if index:
        options.append(rows[index - 1])
    best = min(options, key=lambda row: abs(number(row[key]) - instant))
    return best if abs(number(best[key]) - instant) <= max_skew else None


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def metric(values):
    values = [value for value in values if finite(value)]
    if not values:
        return {"count": 0, "rmse": None, "mae": None, "max": None, "mean": None}
    return {
        "count": len(values),
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
        "mae": sum(abs(value) for value in values) / len(values),
        "max": max(abs(value) for value in values),
        "mean": sum(values) / len(values),
    }


def moving_mean(rows, key, half_window_sec):
    values = []
    left = 0
    right = 0
    total = 0.0
    count = 0
    for index, row in enumerate(rows):
        instant = row["time_sec"]
        while right < len(rows) and rows[right]["time_sec"] <= instant + half_window_sec:
            candidate = rows[right][key]
            if finite(candidate):
                total += candidate
                count += 1
            right += 1
        while left < len(rows) and rows[left]["time_sec"] < instant - half_window_sec:
            candidate = rows[left][key]
            if finite(candidate):
                total -= candidate
                count -= 1
            left += 1
        values.append(total / count if count else math.nan)
    return values


def interpolate_for_display(values):
    result = list(values)
    known = [index for index, value in enumerate(result) if finite(value)]
    if not known:
        return result
    first, last = known[0], known[-1]
    for index in range(first):
        result[index] = result[first]
    for index in range(last + 1, len(result)):
        result[index] = result[last]
    for left, right in zip(known, known[1:]):
        if right == left + 1:
            continue
        start, end = result[left], result[right]
        for index in range(left + 1, right):
            ratio = (index - left) / (right - left)
            result[index] = start + ratio * (end - start)
    return result


def parse_server_events(path):
    if path is None or not path.exists():
        return []
    markers = (
        "F3E-FID-OBS", "F3E-FID-FIRST-ANCHOR", "F3H-FID-POSE-ERROR",
        "F3I-GRAPH-BUILD", "F3J-OPTIMIZE", "F3L-VALIDATE",
        "F3K-ATOMIC-COMMIT", "F3Q-OPT-START", "F3Q-OPT-END",
    )
    events = []
    time_re = re.compile(r"\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*\[([0-9]+(?:\.[0-9]+)?)\]")
    for line_number, line in enumerate(path.read_text(errors="replace").splitlines(), start=1):
        marker = next((item for item in markers if f"[{item}]" in line), None)
        if marker is None:
            continue
        match = time_re.search(line)
        fields = dict(re.findall(r"([A-Za-z_]+)=([^\s,]+)", line))
        events.append({
            "marker": marker,
            "time_sec": number(match.group(1)) if match else math.nan,
            "line": line_number,
            "fields": fields,
            "raw": line,
        })
    return events


def event_time(events, marker, minimum=-math.inf):
    candidates = [event["time_sec"] for event in events
                  if event["marker"] == marker and finite(event["time_sec"])
                  and event["time_sec"] >= minimum]
    return candidates[0] if candidates else math.nan


def vertical_marks(axis, marks):
    colors = {"fid1": "#6a3d9a", "opt": "#e31a1c", "commit": "#33a02c"}
    labels = {"fid1": "fiducial 1", "opt": "optimizacion", "commit": "commit"}
    for name, instant in marks.items():
        if finite(instant):
            axis.axvline(instant, color=colors[name], linewidth=1.0, linestyle="--", label=labels[name])


def save_figures(output_dir, rows, marks, before_rows, after_rows):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    if not rows:
        return []
    paths = []
    time = [row["time_sec"] for row in rows]

    figure, axis = plt.subplots(figsize=(10, 8))
    axis.plot([row["gt_x"] for row in rows], [row["gt_y"] for row in rows], label="GT W", linewidth=1.6)
    global_rows = [row for row in rows if row["global_usable"]]
    axis.plot([row["w_x"] for row in global_rows], [row["w_y"] for row in global_rows], label="ORB W", linewidth=1.2)
    jump_start = marks["commit"] if finite(marks["commit"]) else marks["fid1"]
    jump_pairs = []
    if finite(jump_start):
        for previous, current in zip(global_rows, global_rows[1:]):
            if current["time_sec"] < jump_start:
                continue
            distance = norm3(
                current["w_x"] - previous["w_x"],
                current["w_y"] - previous["w_y"],
                current["w_z"] - previous["w_z"],
            )
            jump_pairs.append((distance, previous, current))
    if jump_pairs:
        _, previous, current = max(jump_pairs, key=lambda item: item[0])
        axis.plot(
            [previous["w_x"], current["w_x"]],
            [previous["w_y"], current["w_y"]],
            color="#e31a1c", linewidth=3.0, label="salto W observado",
        )
    axis.scatter([0.0, 0.0], [-8.5, 8.5], marker="s", color=["#1f78b4", "#6a3d9a"], label="fiduciales 2 / 1")
    commit_row = min(rows, key=lambda row: abs(row["time_sec"] - marks["commit"])) if finite(marks["commit"]) else None
    if commit_row:
        axis.scatter([commit_row["w_x"]], [commit_row["w_y"]], marker="*", s=130, color="#33a02c", label="commit")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("X mundial [m]")
    axis.set_ylabel("Y mundial [m]")
    axis.grid(True, alpha=0.25)
    axis.legend()
    axis.set_title("7.8.2: trayectoria GT frente a ORB en W")
    path = output_dir / "figura_1_trayectoria_xy_w.png"
    figure.tight_layout(); figure.savefig(path, dpi=180); plt.close(figure); paths.append(path.name)

    figure, axis = plt.subplots(figsize=(12, 5))
    raw_error_w = [row["error_w_m"] for row in rows]
    smooth_error_w = interpolate_for_display(moving_mean(rows, "error_w_m", 1.0))
    axis.plot(time, raw_error_w, color="#bdbdbd", alpha=0.35, linewidth=0.7, label="e_W instantaneo")
    axis.plot(time, smooth_error_w, color="#1f78b4", linewidth=1.8, label="e_W media movil de 2 s")
    vertical_marks(axis, marks)
    axis.set_xlabel("Tiempo desde autoridad ORB [s]")
    axis.set_ylabel("Error global W [m]")
    axis.grid(True, alpha=0.25); axis.legend()
    axis.set_title("7.8.2: error global y eventos")
    path = output_dir / "figura_2_error_global_w.png"
    figure.tight_layout(); figure.savefig(path, dpi=180); plt.close(figure); paths.append(path.name)

    figure, axis = plt.subplots(figsize=(12, 5))
    axis.plot(time, [row["error_o_m"] for row in rows], label="e_O = ||ORB O - referencia O||")
    vertical_marks(axis, marks)
    axis.set_xlabel("Tiempo desde autoridad ORB [s]")
    axis.set_ylabel("Error local O [m]")
    axis.grid(True, alpha=0.25); axis.legend()
    axis.set_title("7.8.2: continuidad local de control")
    path = output_dir / "figura_3_error_local_o.png"
    figure.tight_layout(); figure.savefig(path, dpi=180); plt.close(figure); paths.append(path.name)

    if finite(marks["commit"]):
        low, high = marks["commit"] - 5.0, marks["commit"] + 5.0
        zoom = [row for row in rows if low <= row["time_sec"] <= high]
        zoom_title = "7.8.2: zoom alrededor del commit"
        zoom_name = "figura_4_zoom_commit.png"
    elif finite(marks["fid1"]):
        low, high = marks["fid1"] - 5.0, marks["fid1"] + 5.0
        zoom = [row for row in rows if low <= row["time_sec"] <= high]
        zoom_title = "7.8.2: zoom alrededor de la revisita al fiducial 1"
        zoom_name = "figura_4_zoom_revisita_fiducial_1.png"
    else:
        zoom = rows
        zoom_title = "7.8.2: zoom de trayectoria ORB"
        zoom_name = "figura_4_zoom_trayectoria_orb.png"
    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    zoom_time = [row["time_sec"] for row in zoom]
    for axis, coordinate, label in zip(axes, ("x", "y", "z"), ("X", "Y", "Z")):
        axis.plot(zoom_time, [row[f"w_{coordinate}"] for row in zoom], label=f"ORB W {label}")
        axis.plot(zoom_time, [row[f"gt_{coordinate}"] for row in zoom], label=f"GT W {label}")
        axis.plot(zoom_time, [row[f"o_{coordinate}"] for row in zoom], label=f"ORB O {label}")
        axis.set_ylabel(f"{label} [m]")
        axis.grid(True, alpha=0.25)
        axis.legend()
        vertical_marks(axis, marks)
    axes[-1].set_xlabel("Tiempo desde autoridad ORB [s]")
    figure.suptitle(zoom_title)
    path = output_dir / zoom_name
    figure.tight_layout(rect=(0, 0, 1, 0.97)); figure.savefig(path, dpi=180); plt.close(figure); paths.append(path.name)

    for label, subset in (("antes", before_rows), ("despues", after_rows)):
        if not subset:
            continue
        figure, axis = plt.subplots(figsize=(8, 7))
        valid = [row for row in subset if row["global_usable"]]
        axis.plot([row["gt_x"] for row in subset], [row["gt_y"] for row in subset], label="GT W", linewidth=1.8)
        axis.plot([row["w_x"] for row in valid], [row["w_y"] for row in valid], label="ORB W", linewidth=1.4)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("X mundial [m]"); axis.set_ylabel("Y mundial [m]")
        axis.grid(True, alpha=0.25); axis.legend()
        axis.set_title(f"7.8.2: GT frente a ORB W {label} del commit")
        path = output_dir / f"gt_vs_orb_w_{label}_commit.png"
        figure.tight_layout(); figure.savefig(path, dpi=180); plt.close(figure); paths.append(path.name)
    return paths


def markdown_table(summary):
    table = summary["table"]
    lines = ["| Magnitud | Resultado |", "|---|---:|"]
    for label, value in table.items():
        lines.append(f"| {label} | {value} |")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--server-log", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    navigation = read_csv(args.raw_dir / "navigation_state.csv")
    gt = read_csv(args.raw_dir / "gt_pose.csv")
    feedback = read_csv(args.raw_dir / "trajectory_feedback.csv")
    fiducials_path = args.raw_dir / "fiducial_primary_observations.csv"
    fiducials = read_csv(fiducials_path) if fiducials_path.exists() else []
    navigation.sort(key=lambda row: number(row["receive_sec"]))
    gt.sort(key=lambda row: number(row["receive_sec"]))
    feedback.sort(key=lambda row: number(row["receive_sec"]))
    fiducials.sort(key=lambda row: number(row["receive_sec"]))
    gt_times = [number(row["receive_sec"]) for row in gt]
    feedback_times = [number(row["receive_sec"]) for row in feedback]

    orb = [row for row in navigation if integer(row["pose_source"]) == 1
           and integer(row["local_valid"]) == 1
           and integer(row["local_continuity_valid"]) == 1
           and integer(row["velocity_valid"]) == 1]
    if not orb:
        raise RuntimeError("No hay NavigationState ORB local valido para evaluar")
    start = number(orb[0]["receive_sec"])
    rows = []
    previous_kf = None
    previous_revision = None
    for nav in orb:
        instant = number(nav["receive_sec"])
        gt_row = nearest(gt, gt_times, instant, "receive_sec", 0.10)
        ref = nearest(feedback, feedback_times, instant, "receive_sec", 0.25)
        if gt_row is None or ref is None:
            continue
        keyframe = integer(nav["reference_keyframe_id"]) if integer(nav["reference_keyframe_valid"]) else -1
        revision = integer(nav["pose_revision"])
        w_values = [number(nav[key]) for key in ("w_x", "w_y", "w_z")]
        global_usable = integer(nav["global_valid"]) == 1 and all(finite(value) for value in w_values)
        error_w = norm3(w_values[0] - number(gt_row["x"]), w_values[1] - number(gt_row["y"]), w_values[2] - number(gt_row["z"])) if global_usable else math.nan
        error_o = norm3(number(nav["o_x"]) - number(ref["x_pos"]), number(nav["o_y"]) - number(ref["y_pos"]), number(nav["o_z"]) - number(ref["z_pos"]))
        rows.append({
            "time_sec": instant - start, "receive_sec": instant, "sample_sequence": integer(nav["sample_sequence"]),
            "map_epoch": integer(nav["map_epoch"]), "tracking_state": integer(nav["tracking_state"]),
            "pose_revision": revision, "reference_keyframe_id": keyframe,
            "keyframe_changed": int(previous_kf is not None and keyframe != previous_kf),
            "revision_changed": int(previous_revision is not None and revision != previous_revision),
            "global_usable": int(global_usable),
            "o_x": number(nav["o_x"]), "o_y": number(nav["o_y"]), "o_z": number(nav["o_z"]), "o_yaw": number(nav["o_yaw"]),
            "w_x": w_values[0], "w_y": w_values[1], "w_z": w_values[2], "w_yaw": number(nav["w_yaw"]),
            "gt_x": number(gt_row["x"]), "gt_y": number(gt_row["y"]), "gt_z": number(gt_row["z"]), "gt_yaw": number(gt_row["yaw"]),
            "ref_x": number(ref["x_pos"]), "ref_y": number(ref["y_pos"]), "ref_z": number(ref["z_pos"]), "ref_yaw": number(ref["yaw_pos"]),
            "error_w_m": error_w, "error_o_m": error_o,
            "error_o_yaw_rad": wrap(number(nav["o_yaw"]) - number(ref["yaw_pos"])),
            "navigation_age_sec": instant - number(nav["stamp_sec"]) if abs(instant - number(nav["stamp_sec"])) < 1.0 else math.nan,
        })
        previous_kf, previous_revision = keyframe, revision
    if not rows:
        raise RuntimeError("No se pudieron sincronizar NavigationState, GT y feedback")

    fid1 = next((row for row in fiducials if integer(row["drone_id"]) == 1 and integer(row["object_id"]) == 1 and number(row["receive_sec"]) >= start), None)
    fid1_time = number(fid1["receive_sec"]) - start if fid1 else math.nan
    events = parse_server_events(args.server_log)
    opt_time = event_time(events, "F3Q-OPT-START", start)
    commit_time = event_time(events, "F3K-ATOMIC-COMMIT", start)
    if finite(opt_time):
        opt_time -= start
    if finite(commit_time):
        commit_time -= start
    commit_source = "F3K-ATOMIC-COMMIT" if finite(commit_time) else "no_observado"
    marks = {"fid1": fid1_time, "opt": opt_time, "commit": commit_time}
    before_rows = [row for row in rows if finite(commit_time) and commit_time - 6.0 <= row["time_sec"] <= commit_time - 1.0]
    after_rows = [row for row in rows if finite(commit_time) and commit_time + 1.0 <= row["time_sec"] <= commit_time + 6.0]
    if not finite(commit_time):
        before_rows, after_rows = [], []
    write_csv(args.output_dir / "samples.csv", rows)
    figures = save_figures(args.output_dir, rows, marks, before_rows, after_rows)
    write_csv(args.output_dir / "server_events.csv", events)

    before_w = metric([row["error_w_m"] for row in before_rows])
    after_w = metric([row["error_w_m"] for row in after_rows])
    local_commit = [row for row in rows if finite(commit_time) and abs(row["time_sec"] - commit_time) <= 5.0]
    kf_changes = sum(row["keyframe_changed"] for row in rows)
    tracking_losses = sum(row["tracking_state"] not in (2, 5) for row in rows)
    revision_values = [row["pose_revision"] for row in rows]
    table = {
        "Duracion ORB": f"{rows[-1]['time_sec']:.3f} s",
        "Muestras ORB sincronizadas": str(len(rows)),
        "KF de referencia distintos": str(len({row['reference_keyframe_id'] for row in rows if row['reference_keyframe_id'] >= 0})),
        "Cambios de KF": str(kf_changes),
        "Deteccion fiducial 1": f"{fid1_time:.3f} s" if finite(fid1_time) else "no observada",
        "Commit": f"{commit_time:.3f} s ({commit_source})" if finite(commit_time) else "no observado",
        "RMSE W antes": f"{before_w['rmse']:.4f} m" if before_w['rmse'] is not None else "no disponible",
        "RMSE W despues": f"{after_w['rmse']:.4f} m" if after_w['rmse'] is not None else "no disponible",
        "Error W maximo": f"{metric([row['error_w_m'] for row in rows])['max']:.4f} m",
        "Error O maximo en ventana commit": f"{metric([row['error_o_m'] for row in local_commit])['max']:.4f} m" if local_commit else "no disponible",
        "Fallbacks": str(sum(integer(row.get('pose_source', '1')) == 3 for row in navigation)),
        "Muestras tracking no OK": str(tracking_losses),
        "pose_revision inicio -> fin": f"{revision_values[0]} -> {revision_values[-1]}",
    }
    summary = {
        "test": "7.8.2", "orb_start_receive_sec": start, "event_times_sec_since_orb_start": marks,
        "commit_source": commit_source, "fiducial_1": fid1,
        "server_event_counts": {marker: sum(event['marker'] == marker for event in events) for marker in sorted({event['marker'] for event in events})},
        "global_w": {"all": metric([row["error_w_m"] for row in rows]), "before_commit": before_w, "after_commit": after_w},
        "local_o": {"all": metric([row["error_o_m"] for row in rows]), "commit_window": metric([row["error_o_m"] for row in local_commit])},
        "keyframes": {"changes": kf_changes, "distinct": len({row['reference_keyframe_id'] for row in rows if row['reference_keyframe_id'] >= 0})},
        "tracking": {"fallback_samples": sum(integer(row.get('pose_source', '1')) == 3 for row in navigation), "non_ok_samples": tracking_losses},
        "figures": figures, "table": table,
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    (args.output_dir / "tabla_resultados.md").write_text(markdown_table(summary))


if __name__ == "__main__":
    main()
