#!/usr/bin/env python3
"""Extrae tablas y figuras de optimizaciones por loop ya registradas."""

import argparse
import csv
import hashlib
import re
from pathlib import Path

import matplotlib.pyplot as plt


TIME_RE = re.compile(r"\[(\d+\.\d+)\]")
LOOP_RE = re.compile(
    r"\[F3Q-LOOP-OPT\] task=(?P<task>\d+) .*?committed=(?P<committed>\w+) "
    r"stale=(?P<stale>\w+) submaps=(?P<submaps>\d+) "
    r"window=(?P<window>\d+) controls=(?P<controls>\d+) "
    r"edges=\(temporal=(?P<temporal>\d+),covis=(?P<covis>\d+),loop=(?P<loop>\d+)\) "
    r"iterations=(?P<iterations>\d+) error_before=\((?P<translation_before>[\d.]+),(?P<rotation_before>[\d.]+)\) "
    r"error_after=\((?P<translation_after>[\d.]+),(?P<rotation_after>[\d.]+)\) "
    r".*?cost=\((?P<cost_before>[\d.]+)->(?P<cost_after>[\d.]+)\) "
    r"time_ms=\(graph=(?P<graph_ms>[\d.]+),solve=(?P<solve_ms>[\d.]+),"
    r"validation=(?P<validation_ms>[\d.]+),commit=(?P<commit_ms>[\d.]+)\) "
    r".*?moved=(?P<moved>\d+) propagated=(?P<propagated>\d+) "
    r".*?reason=(?P<reason>\S+)")
START_RE = re.compile(r"\[F3Q-OPT-START\] task=(\d+)")
END_RE = re.compile(r"\[F3Q-OPT-END\] task=(\d+).*?committed=(\w+).*?reason=(\S+)")


def timestamp(line):
    matches = TIME_RE.findall(line)
    return float(matches[-1]) if matches else None


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save(figure, output_dir, stem):
    figure.tight_layout()
    figure.savefig(output_dir / f"{stem}.png", dpi=180)
    figure.savefig(output_dir / f"{stem}.pdf")
    plt.close(figure)


def write_tables(rows, output_dir):
    markdown = [
        "# Optimizaciones por loop (6.4.2)",
        "",
        "| Paso | Task | Ventana | Controles | Aristas T/C/L | Error t antes -> despues [m] | "
        "Error r antes -> despues [rad] | Coste antes -> despues | KFs movidos | Commit [ms] |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    latex = [
        "\\begin{tabular}{rrrrrrrr}",
        "\\hline",
        "Paso & Ventana & Controles & Aristas & $e_t$ antes & $e_t$ despues & Coste antes & Coste despues \\\\",
        "\\hline",
    ]
    for index, row in enumerate(rows, start=1):
        markdown.append(
            f"| {index} | {row['task']} | {row['window']} | {row['controls']} | "
            f"{row['temporal']}/{row['covis']}/{row['loop']} | "
            f"{float(row['translation_before']):.3f} -> {float(row['translation_after']):.3f} | "
            f"{float(row['rotation_before']):.3f} -> {float(row['rotation_after']):.3f} | "
            f"{float(row['cost_before']):.3f} -> {float(row['cost_after']):.3f} | "
            f"{row['moved']} | {float(row['commit_ms']):.1f} |")
        latex.append(
            f"{index} & {row['window']} & {row['controls']} & "
            f"{int(row['temporal']) + int(row['covis']) + int(row['loop'])} & "
            f"{float(row['translation_before']):.3f} & {float(row['translation_after']):.3f} & "
            f"{float(row['cost_before']):.3f} & {float(row['cost_after']):.3f} \\\\")
    latex.extend(["\\hline", "\\end{tabular}", ""])
    (output_dir / "tabla_optimizaciones_loop.md").write_text(
        "\n".join(markdown) + "\n", encoding="utf-8")
    (output_dir / "tabla_optimizaciones_loop.tex").write_text(
        "\n".join(latex), encoding="utf-8")


def plot(rows, output_dir):
    index = list(range(1, len(rows) + 1))
    trans_before = [float(row["translation_before"]) for row in rows]
    trans_after = [float(row["translation_after"]) for row in rows]
    rot_before = [float(row["rotation_before"]) for row in rows]
    rot_after = [float(row["rotation_after"]) for row in rows]
    cost_before = [float(row["cost_before"]) for row in rows]
    cost_after = [float(row["cost_after"]) for row in rows]

    figure, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for axis, before, after, label in (
            (axes[0], trans_before, trans_after, "Error de traslacion [m]"),
            (axes[1], rot_before, rot_after, "Error de rotacion [rad]"),
            (axes[2], cost_before, cost_after, "Coste")):
        axis.plot(index, before, "o-", label="Antes")
        axis.plot(index, after, "o-", label="Despues")
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[0].set_title("6.4.2 - Reduccion de error y coste por optimizacion de loop")
    axes[-1].set_xlabel("Optimizacion por loop")
    save(figure, output_dir, "error_y_coste_loop")

    windows = [int(row["window"]) for row in rows]
    controls = [int(row["controls"]) for row in rows]
    temporal = [int(row["temporal"]) for row in rows]
    covis = [int(row["covis"]) for row in rows]
    loop_edges = [int(row["loop"]) for row in rows]
    moved = [int(row["moved"]) for row in rows]
    graph = [float(row["graph_ms"]) for row in rows]
    solve = [float(row["solve_ms"]) for row in rows]
    validation = [float(row["validation_ms"]) for row in rows]
    commit = [float(row["commit_ms"]) for row in rows]
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(index, windows, "o-", label="Ventana")
    axes[0].plot(index, controls, "o-", label="Controles")
    axes[0].plot(index, moved, "o-", label="KFs movidos")
    axes[0].bar(index, temporal, alpha=0.35, label="Aristas temporales")
    axes[0].bar(index, covis, bottom=temporal, alpha=0.35, label="Aristas covisibilidad")
    axes[0].bar(index, loop_edges, bottom=[a + b for a, b in zip(temporal, covis)],
                alpha=0.6, label="Aristas loop")
    axes[0].set_ylabel("Cantidad")
    axes[0].set_title("6.4.2 - Estructura del grafo y KFs corregidos")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncol=2, fontsize=8)
    axes[1].bar(index, graph, label="Grafo")
    axes[1].bar(index, solve, bottom=graph, label="Solve")
    axes[1].bar(index, validation, bottom=[a + b for a, b in zip(graph, solve)],
                label="Validacion")
    axes[1].bar(index, commit,
                bottom=[a + b + c for a, b, c in zip(graph, solve, validation)],
                label="Commit")
    axes[1].set(xlabel="Optimizacion por loop", ylabel="Tiempo [ms]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(ncol=4, fontsize=8)
    save(figure, output_dir, "estructura_y_tiempos_loop")

    start = [float(row["start_time_s"]) for row in rows]
    result = [float(row["time_s"]) for row in rows]
    end = [float(row["end_time_s"]) for row in rows]
    figure, axis = plt.subplots(figsize=(10, 3.8))
    for number, start_t, result_t, end_t in zip(index, start, result, end):
        axis.hlines(number, start_t, end_t, color="C0", linewidth=2)
        axis.plot(start_t, number, "o", color="C1", label="Inicio" if number == 1 else "")
        axis.plot(result_t, number, "s", color="C2", label="Resultado" if number == 1 else "")
        axis.plot(end_t, number, "o", color="C3", label="Fin" if number == 1 else "")
    axis.set(title="6.4.2 - Timeline de optimizaciones por loop",
             xlabel="Tiempo relativo desde el primer marcador F3 [s]",
             ylabel="Optimizacion")
    axis.set_yticks(index)
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=3)
    save(figure, output_dir, "timeline_optimizaciones_loop")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--data-dir", required=True, type=Path)
    parser.add_argument("--figure-dir", required=True, type=Path)
    args = parser.parse_args()
    args.data_dir.mkdir(parents=True, exist_ok=True)
    args.figure_dir.mkdir(parents=True, exist_ok=True)

    starts = {}
    ends = {}
    first_time = None
    results = []
    for line in args.log.read_text(encoding="utf-8", errors="replace").splitlines():
        line_time = timestamp(line)
        if line_time is None:
            continue
        if first_time is None:
            first_time = line_time
        start_match = START_RE.search(line)
        if start_match:
            starts[start_match.group(1)] = line_time
            continue
        end_match = END_RE.search(line)
        if end_match:
            ends[end_match.group(1)] = (line_time, end_match.group(2), end_match.group(3))
            continue
        match = LOOP_RE.search(line)
        if not match:
            continue
        results.append((line_time, match.groupdict()))

    if not results:
        raise SystemExit("No se encontraron eventos [F3Q-LOOP-OPT].")
    rows = []
    for line_time, row in results:
        task = row["task"]
        end_time, end_committed, end_reason = ends.get(task, (line_time, "unknown", "missing"))
        row.update({
            "time_s": f"{line_time - first_time:.6f}",
            "start_time_s": f"{starts.get(task, line_time) - first_time:.6f}",
            "end_time_s": f"{end_time - first_time:.6f}",
            "end_committed": end_committed,
            "end_reason": end_reason,
        })
        rows.append(row)
    write_csv(args.data_dir / "optimizaciones_loop.csv", rows)
    write_tables(rows, args.data_dir)
    (args.data_dir / "summary.json").write_text(
        "{\n"
        f"  \"source_log\": \"{args.log}\",\n"
        f"  \"source_log_sha256\": \"{sha256(args.log)}\",\n"
        f"  \"loop_optimizations\": {len(rows)},\n"
        f"  \"all_committed\": {str(all(row['committed'] == 'true' for row in rows)).lower()}\n"
        "}\n", encoding="utf-8")
    plot(rows, args.figure_dir)


if __name__ == "__main__":
    main()
