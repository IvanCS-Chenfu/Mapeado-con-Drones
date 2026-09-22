#!/usr/bin/env python3
"""Extrae y representa los contadores nativos de RawMapDatabase."""

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


RAW_STATS_PATTERN = re.compile(
    r"\[([0-9]+\.[0-9]+)\].*\[F3A-RAW-STATS\] "
    r"arrival_id=(\d+) source=([^ ]+) full_snapshot=(true|false) "
    r"submap=\((\d+),(\d+)\) submaps=(\d+) keyframes=(\d+) "
    r"mappoints=(\d+) delta_entries=(\d+) fiducial_observations=(\d+)"
)


def parse_records(path):
    records_by_arrival = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = RAW_STATS_PATTERN.search(line)
        if not match:
            continue
        values = match.groups()
        record = {
            "stamp_sec": float(values[0]),
            "arrival_id": int(values[1]),
            "source": values[2],
            "full_snapshot": values[3] == "true",
            "drone_id": int(values[4]),
            "map_epoch": int(values[5]),
            "submaps": int(values[6]),
            "keyframes": int(values[7]),
            "mappoints": int(values[8]),
            "delta_entries": int(values[9]),
            "fiducial_observations": int(values[10]),
        }
        previous = records_by_arrival.get(record["arrival_id"])
        if previous is not None:
            if previous != record:
                raise RuntimeError(
                    f"arrival_id duplicado con valores distintos: {record['arrival_id']}")
            continue
        records_by_arrival[record["arrival_id"]] = record
    records = list(records_by_arrival.values())
    if not records:
        raise RuntimeError(f"No se encontraron marcadores F3A-RAW-STATS en {path}")
    records.sort(key=lambda record: record["arrival_id"])
    for previous, current in zip(records, records[1:]):
        if current["arrival_id"] <= previous["arrival_id"]:
            raise RuntimeError("arrival_id no es estrictamente creciente")
    first_stamp = records[0]["stamp_sec"]
    snapshot_count = 0
    for record in records:
        record["elapsed_sec"] = record["stamp_sec"] - first_stamp
        snapshot_count += int(record["full_snapshot"])
        record["snapshot_count"] = snapshot_count
    return records


def write_csv(path, records):
    fields = (
        "elapsed_sec", "arrival_id", "source", "full_snapshot", "drone_id",
        "map_epoch", "submaps", "keyframes", "mappoints", "delta_entries",
        "fiducial_observations", "snapshot_count",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = dict(record)
            row.pop("stamp_sec")
            row["elapsed_sec"] = f"{record['elapsed_sec']:.9f}"
            row["full_snapshot"] = str(record["full_snapshot"]).lower()
            writer.writerow(row)


def plot(path, records):
    time = [record["elapsed_sec"] for record in records]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    flat_axes = axes.flatten()
    flat_axes[0].plot(time, [record["mappoints"] for record in records],
                      color="#d1495b", linewidth=1.8)
    flat_axes[0].set_title("MapPoints")
    flat_axes[0].set_ylabel("MapPoints almacenados")
    flat_axes[1].plot(time, [record["keyframes"] for record in records],
                      color="#6a994e", linewidth=1.8)
    flat_axes[1].set_title("KeyFrames")
    flat_axes[1].set_ylabel("KeyFrames almacenados")
    flat_axes[2].plot(time, [record["submaps"] for record in records],
                      color="#2a6f97", linewidth=1.8)
    flat_axes[2].set_title("Submapas")
    flat_axes[2].set_xlabel("Tiempo desde la primera inserción raw [s]")
    flat_axes[2].set_ylabel("Submapas almacenados")
    flat_axes[3].plot(time, [record["delta_entries"] for record in records],
                 label="Entradas delta", color="#7b2cbf", linewidth=1.8)
    flat_axes[3].plot(time, [record["fiducial_observations"] for record in records],
                 label="Observaciones fiduciales", color="#f77f00", linewidth=1.8)
    snapshot_axis = flat_axes[3].twinx()
    snapshot_axis.step(time, [record["snapshot_count"] for record in records],
                       where="post", label="Snapshots", color="#2a6f97", linewidth=1.8)
    snapshot_axis.set_ylabel("Snapshots acumulados", color="#2a6f97")
    snapshot_axis.tick_params(axis="y", colors="#2a6f97")
    flat_axes[3].set_title("Deltas, observaciones y snapshots")
    flat_axes[3].set_xlabel("Tiempo desde la primera inserción raw [s]")
    flat_axes[3].set_ylabel("Entradas almacenadas")
    left_handles, left_labels = flat_axes[3].get_legend_handles_labels()
    right_handles, right_labels = snapshot_axis.get_legend_handles_labels()
    flat_axes[3].legend(left_handles + right_handles, left_labels + right_labels)
    for axis in flat_axes:
        axis.grid(True, alpha=0.28)
    fig.suptitle("Prueba 5.5: evolución de RawMapDatabase")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_summary(path, records):
    final = records[-1]
    summary = {
        "sample_count": len(records),
        "duration_sec": final["elapsed_sec"],
        "all_sources_live": all(record["source"] == "live" for record in records),
        "full_snapshot_count": sum(record["full_snapshot"] for record in records),
        "final": {name: final[name] for name in (
            "arrival_id", "submaps", "keyframes", "mappoints", "delta_entries",
            "fiducial_observations",
        )},
    }
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    records = parse_records(args.input)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "datos_raw.csv", records)
    plot(args.output_dir / "grafica_datos_raw.png", records)
    print(json.dumps(write_summary(args.output_dir / "resumen.json", records), indent=2,
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
