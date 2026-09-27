#!/usr/bin/env python3
"""Genera métricas y figuras SVG de la telemetría pasiva de 8.5.6."""
import argparse
import csv
import html
import json
import re
from collections import defaultdict
from pathlib import Path

SOURCE = re.compile(r"\[([0-9]+\.[0-9]+)\].*\[F6F-DEPTH-SOURCES-WRITTEN\] drone=(\d+).*command=([^ ]+).*kind=([^ ]+).*free=(\d+) direct_free=(\d+) occupied=(\d+)")
CLAIM = re.compile(r"\[([0-9]+\.[0-9]+)\].*\[F6H-COVERAGE-CLAIMS\] task=([^ ]+) drone=(\d+).*claims=(\d+) active=(\d+) changed=(\w+)")
RESERVATION = re.compile(r"\[([0-9]+\.[0-9]+)\].*\[F6J-AUTONOMOUS-RESERVATION-COMMIT\] drone=(\d+).*trajectory_id=([^ ]+) cells=(\d+)")

WIDTH, HEIGHT = 960, 560
LEFT, RIGHT, TOP, BOTTOM = 100, 34, 54, 88
PLOT_W, PLOT_H = WIDTH - LEFT - RIGHT, HEIGHT - TOP - BOTTOM


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def esc(value):
    return html.escape(str(value), quote=True)


def bounds(series):
    values = [value for points in series for point in points for value in point]
    low, high = min(values), max(values)
    if low == high:
        return low - 0.5, high + 0.5
    margin = (high - low) * 0.06
    return low - margin, high + margin


def chart(path, title, xlabel, ylabel, series, step=False, aspect_equal=False):
    xs = [[point[0] for point in points] for _, _, points in series if points]
    ys = [[point[1] for point in points] for _, _, points in series if points]
    if not xs or not ys:
        return
    xmin, xmax = bounds([[(value, 0) for value in values] for values in xs])
    ymin, ymax = bounds([[(0, value) for value in values] for values in ys])
    if aspect_equal:
        span = max(xmax - xmin, ymax - ymin)
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        xmin, xmax, ymin, ymax = cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2
    def px(x): return LEFT + (x - xmin) * PLOT_W / (xmax - xmin)
    def py(y): return TOP + (ymax - y) * PLOT_H / (ymax - ymin)
    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
              '<rect width="100%" height="100%" fill="white"/>',
              f'<text x="{WIDTH/2}" y="28" text-anchor="middle" font-family="sans-serif" font-size="20">{esc(title)}</text>']
    for i in range(6):
        x = LEFT + i * PLOT_W / 5
        y = TOP + i * PLOT_H / 5
        xv = xmin + i * (xmax - xmin) / 5
        yv = ymax - i * (ymax - ymin) / 5
        chunks += [f'<line x1="{x:.1f}" y1="{TOP}" x2="{x:.1f}" y2="{TOP+PLOT_H}" stroke="#e5e7eb"/>',
                   f'<line x1="{LEFT}" y1="{y:.1f}" x2="{LEFT+PLOT_W}" y2="{y:.1f}" stroke="#e5e7eb"/>',
                   f'<text x="{x:.1f}" y="{TOP+PLOT_H+24}" text-anchor="middle" font-family="sans-serif" font-size="13">{xv:.1f}</text>',
                   f'<text x="{LEFT-10}" y="{y+5:.1f}" text-anchor="end" font-family="sans-serif" font-size="13">{yv:.1f}</text>']
    chunks += [f'<line x1="{LEFT}" y1="{TOP+PLOT_H}" x2="{LEFT+PLOT_W}" y2="{TOP+PLOT_H}" stroke="#111827"/>',
               f'<line x1="{LEFT}" y1="{TOP}" x2="{LEFT}" y2="{TOP+PLOT_H}" stroke="#111827"/>',
               f'<text x="{WIDTH/2}" y="{HEIGHT-22}" text-anchor="middle" font-family="sans-serif" font-size="16">{esc(xlabel)}</text>',
               f'<text x="24" y="{HEIGHT/2}" transform="rotate(-90 24 {HEIGHT/2})" text-anchor="middle" font-family="sans-serif" font-size="16">{esc(ylabel)}</text>']
    legend_y = TOP + 8
    for index, (label, color, points) in enumerate(series):
        if not points:
            continue
        ordered = sorted(points)
        coords = []
        for point_index, (x, y) in enumerate(ordered):
            if step and point_index:
                coords.append((px(x), py(ordered[point_index - 1][1])))
            coords.append((px(x), py(y)))
        text = ' '.join(f'{x:.1f},{y:.1f}' for x, y in coords)
        chunks.append(f'<polyline points="{text}" fill="none" stroke="{color}" stroke-width="2.5"/>')
        lx = LEFT + index * 180
        chunks += [f'<line x1="{lx}" y1="{legend_y}" x2="{lx+22}" y2="{legend_y}" stroke="{color}" stroke-width="3"/>',
                   f'<text x="{lx+28}" y="{legend_y+5}" font-family="sans-serif" font-size="14">{esc(label)}</text>']
    chunks.append('</svg>')
    path.write_text('\n'.join(chunks) + '\n', encoding='utf-8')


def event_chart(path, observations):
    styles = {'vista_unknown': ('#71717a', 'VIEW_UNKNOWN'), 'vista_pared': ('#b45309', 'VIEW_WALL'), 'view_advance': ('#15803d', 'VIEW_ADVANCE')}
    origin = min(item['stamp'] for item in observations)
    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="360" viewBox="0 0 {WIDTH} 360">', '<rect width="100%" height="100%" fill="white"/>', '<text x="480" y="28" text-anchor="middle" font-family="sans-serif" font-size="20">Tipos de observación materializados</text>']
    xmax = max(item['stamp'] - origin for item in observations) or 1
    def px(value): return LEFT + value * PLOT_W / xmax
    for drone, y in ((1, 130), (2, 240)):
        chunks += [f'<line x1="{LEFT}" y1="{y}" x2="{LEFT+PLOT_W}" y2="{y}" stroke="#e5e7eb"/>', f'<text x="{LEFT-12}" y="{y+5}" text-anchor="end" font-family="sans-serif" font-size="16">D{drone}</text>']
    for tick in range(6):
        x = LEFT + tick * PLOT_W / 5
        value = tick * xmax / 5
        chunks += [f'<line x1="{x}" y1="70" x2="{x}" y2="270" stroke="#e5e7eb"/>', f'<text x="{x}" y="300" text-anchor="middle" font-family="sans-serif" font-size="13">{value:.0f}</text>']
    used = set()
    for item in observations:
        color, label = styles.get(item['kind'], ('#111827', item['kind']))
        x, y = px(item['stamp'] - origin), 130 if item['drone_id'] == 1 else 240
        chunks.append(f'<circle cx="{x:.1f}" cy="{y}" r="7" fill="{color}"/>')
        if label not in used:
            offset = 115 * len(used)
            chunks += [f'<circle cx="{LEFT+offset}" cy="48" r="6" fill="{color}"/>', f'<text x="{LEFT+offset+12}" y="53" font-family="sans-serif" font-size="14">{esc(label)}</text>']
            used.add(label)
    chunks += ['<text x="480" y="340" text-anchor="middle" font-family="sans-serif" font-size="16">Tiempo desde primera observación materializada (s)</text>', '</svg>']
    path.write_text('\n'.join(chunks) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw-dir', type=Path, required=True)
    parser.add_argument('--reduced-log', type=Path, required=True)
    parser.add_argument('--processed-dir', type=Path, required=True)
    parser.add_argument('--figures-dir', type=Path, required=True)
    args = parser.parse_args()
    args.processed_dir.mkdir(parents=True, exist_ok=True)
    args.figures_dir.mkdir(parents=True, exist_ok=True)
    navigation = load(args.raw_dir / 'navigation_timeline.json')
    tasks = load(args.raw_dir / 'task_state_events.json')
    voxels = load(args.raw_dir / 'voxel_map_timeline.json')
    text = args.reduced_log.read_text(encoding='utf-8')
    observations = [{'stamp': float(m.group(1)), 'drone_id': int(m.group(2)), 'command_id': m.group(3), 'kind': m.group(4), 'free': int(m.group(5)), 'direct_free': int(m.group(6)), 'occupied': int(m.group(7))} for m in SOURCE.finditer(text)]
    claims = [{'stamp': float(m.group(1)), 'task_id': m.group(2), 'drone_id': int(m.group(3)), 'claims': int(m.group(4)), 'active_claims': int(m.group(5)), 'changed': m.group(6) == 'true'} for m in CLAIM.finditer(text)]
    reservations = [{'stamp': float(m.group(1)), 'drone_id': int(m.group(2)), 'trajectory_id': m.group(3), 'cells': int(m.group(4))} for m in RESERVATION.finditer(text)]
    handoff = min((row['elapsed_sec'] for row in tasks if row['assigned_drone_id'] in (1, 2)), default=0.0)
    nav = defaultdict(list)
    for row in navigation:
        if row['global_valid'] and row['elapsed_sec'] >= handoff:
            nav[row['drone_id']].append((row['x'], row['y']))
    chart(args.figures_dir / 'A_trayectorias_globales.svg', 'Trayectorias globales durante la autonomía', 'X global (m)', 'Y global (m)', [('D1', '#2468a2', nav[1]), ('D2', '#d04a32', nav[2])], aspect_equal=True)
    task_series = []
    for task_id in sorted({row['task_id'] for row in tasks}):
        points = [(row['elapsed_sec'], 100 * row['progress']) for row in tasks if row['task_id'] == task_id and row['progress_known']]
        task_series.append((task_id.replace('map_section_', ''), '#15803d' if task_id.endswith('BC') else '#2468a2', points))
    chart(args.figures_dir / 'B_coverage_temporal.svg', 'Evolución de coverage por tarea', 'Tiempo desde inicio (s)', 'Coverage derivado (%)', task_series, step=True)
    chart(args.figures_dir / 'C_reservas_temporales.svg', 'Reservas temporales del planificador', 'Tiempo desde inicio (s)', 'Vóxeles reservados', [('Reservados', '#7057a6', [(row['elapsed_sec'], row['reserved_voxels']) for row in voxels])], step=True)
    event_chart(args.figures_dir / 'D_tipos_observacion.svg', observations)
    metrics = {'handoff_elapsed_sec': handoff, 'observation_count': len(observations), 'coverage_claim_event_count': len(claims), 'reservation_commit_count': len(reservations), 'max_reserved_voxels': max((row['reserved_voxels'] for row in voxels), default=0), 'final_raw_voxels': voxels[-1]['raw_voxels'] if voxels else 0, 'final_map_revision': voxels[-1]['map_revision'] if voxels else 0}
    (args.processed_dir / 'metrics.json').write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    for name, rows in [('observaciones.csv', observations), ('coverage_claims.csv', claims), ('reservas.csv', reservations)]:
        with (args.processed_dir / name).open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]) if rows else ('empty',))
            writer.writeheader()
            writer.writerows(rows)


if __name__ == '__main__':
    main()
