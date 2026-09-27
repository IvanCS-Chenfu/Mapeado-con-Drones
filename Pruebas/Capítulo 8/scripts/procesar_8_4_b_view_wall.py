#!/usr/bin/env python3
"""Correlaciona MOVE_AND_CAPTURE, fuentes depth y coverage para 8.4-B."""
import argparse
import csv
import json
import re
from pathlib import Path


def stamp(line):
    match = re.search(r"\[([0-9]+\.[0-9]+)\]", line)
    return float(match.group(1)) if match else None


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reduced-log", required=True, type=Path)
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--processed-dir", required=True, type=Path)
    args = parser.parse_args()
    args.processed_dir.mkdir(parents=True, exist_ok=True)
    lines = args.reduced_log.read_text(encoding="utf-8", errors="replace").splitlines()

    accepted = next((line for line in lines if "[F6C-AUTONOMOUS-COMMAND-ACCEPTED]" in line and "type=1" in line), None)
    if accepted is None:
        raise SystemExit("No aparece un MOVE_AND_CAPTURE aceptado (type=1).")
    command_match = re.search(r"command=([^ ]+)", accepted)
    command_id = command_match.group(1) if command_match else ""
    written = next((line for line in lines if "[F6F-DEPTH-SOURCES-WRITTEN]" in line and f"command={command_id}" in line), None)
    pending = next((line for line in lines if "[F6H-COVERAGE-SOURCE-PENDING]" in line and f"command={command_id}" in line), None)
    if written is None:
        raise SystemExit("No se escribieron fuentes depth para el MOVE_AND_CAPTURE seleccionado.")
    kind_match = re.search(r"kind=([^ ]+)", written)
    counts_match = re.search(r"free=(\d+) direct_free=(\d+) occupied=(\d+)", written)
    if kind_match is None or counts_match is None:
        raise SystemExit("El marcador F6F no contiene tipo o contadores de fuente.")
    kind = kind_match.group(1)
    source_free, source_direct_free, source_occupied = map(int, counts_match.groups())
    source_id = re.search(r"source=([^ ]+)", pending).group(1) if pending and re.search(r"source=([^ ]+)", pending) else ""
    claims = next((line for line in lines if "[F6H-COVERAGE-CLAIMS]" in line and source_id and f"source={source_id}" in line), None)
    task_id = re.search(r"task=([^ ]+)", claims).group(1) if claims and re.search(r"task=([^ ]+)", claims) else ""
    claims_match = re.search(r"claims=(\d+) active=(\d+) changed=(true|false)", claims or "")
    task_events = load(args.raw_dir / "task_state_events.json")
    task_progress = [item for item in task_events if item["task_id"] == task_id]
    before = min(task_progress, key=lambda item: (item["progress"], item["state_revision"]), default=None)
    after = max(task_progress, key=lambda item: (item["progress"], item["state_revision"]), default=None)
    result = {
        "command_id": command_id,
        "written_stamp_sec": stamp(written),
        "kind": kind,
        "source_free": source_free,
        "source_direct_free": source_direct_free,
        "source_occupied": source_occupied,
        "coverage_pending": pending is not None,
        "task_id": task_id or None,
        "claims_count": int(claims_match.group(1)) if claims_match else 0,
        "active_sections_after": int(claims_match.group(2)) if claims_match else 0,
        "claims_changed": claims_match.group(3) == "true" if claims_match else False,
        "progress_before": before["progress"] if before else None,
        "progress_after": after["progress"] if after else None,
        "intervals_before": before["coverage_intervals"] if before else [],
        "intervals_after": after["coverage_intervals"] if after else [],
    }
    result["progress_increased"] = (result["progress_before"] is not None and
                                    result["progress_after"] is not None and
                                    result["progress_after"] > result["progress_before"])
    result["result"] = "PASS" if (
        result["kind"] in ("vista_pared", "view_advance") and
        result["source_free"] > 0 and result["source_direct_free"] > 0 and
        result["source_occupied"] > 0 and result["coverage_pending"] and
        result["claims_count"] > 0 and result["active_sections_after"] > 0 and
        result["claims_changed"] and result["progress_increased"]
    ) else "FAIL"
    (args.processed_dir / "view_wall_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (args.processed_dir / "view_wall_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(result))
        writer.writeheader()
        writer.writerow(result)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
