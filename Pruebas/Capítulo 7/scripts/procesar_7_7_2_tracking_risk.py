#!/usr/bin/env python3
"""Procesa la telemetria cruda reproducible de la prueba 7.7.2."""
import argparse
import csv
import json
from pathlib import Path


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('raw_csv', type=Path)
    parser.add_argument('output_dir', type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with args.raw_csv.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    rows.sort(key=lambda row: as_float(row.get('time')) or -1.0)
    processed = args.output_dir / 'tracking_risk_timeline.csv'
    with processed.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys() if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)
    events = [row for row in rows if row.get('record_kind') == 'risk_event']
    first_poor = next((row for row in events if row.get('event_type') == '1'), None)
    stop_done = next((row for row in events if row.get('event_type') == '2'), None)
    reorient_done = next((row for row in events if row.get('event_type') == '4'), None)
    risk_time = as_float(first_poor.get('time')) if first_poor else None
    stop_time = as_float(stop_done.get('time')) if stop_done else None
    reorient_time = as_float(reorient_done.get('time')) if reorient_done else None
    summary = {
        'samples': len(rows),
        'risk_events': len(events),
        'first_stop_started': first_poor,
        'stop_completed': stop_done,
        'reorientation_completed': reorient_done,
        'latency_first_risk_to_stop_completed_sec': None if risk_time is None or stop_time is None else stop_time-risk_time,
        'latency_stop_to_reorientation_completed_sec': None if stop_time is None or reorient_time is None else reorient_time-stop_time,
        'success_candidate': bool(first_poor and stop_done and stop_done.get('event_success') == '1' and reorient_done and reorient_done.get('event_success') == '1'),
    }
    (args.output_dir / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False) + chr(10), encoding='utf-8')
    try:
        import matplotlib.pyplot as plt
        evidence = [row for row in rows if row.get('record_kind') == 'evidence']
        if evidence:
            x = [as_float(row['time']) for row in evidence]
            fig, axis = plt.subplots(figsize=(11, 5))
            axis.plot(x, [as_float(row['tracking_inliers']) for row in evidence], label='total')
            for key, label in [('left_inliers', 'left'), ('right_inliers', 'right'), ('top_inliers', 'top'), ('bottom_inliers', 'bottom')]:
                axis.plot(x, [as_float(row[key]) for row in evidence], label=label, alpha=0.75)
            for row, label in [(first_poor, 'risk/STOP'), (stop_done, 'STOP terminado'), (reorient_done, 'reorientacion terminada')]:
                if row:
                    axis.axvline(as_float(row['time']), linestyle='--', label=label)
            axis.set_xlabel('Tiempo simulado (s)')
            axis.set_ylabel('Inliers ORB')
            axis.legend()
            axis.grid(True, alpha=0.25)
            fig.tight_layout()
            fig.savefig(args.output_dir / 'tracking_risk_timeline.png', dpi=160)
    except ImportError:
        pass


if __name__ == '__main__':
    main()
