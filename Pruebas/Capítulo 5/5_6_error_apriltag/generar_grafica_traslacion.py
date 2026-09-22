#!/usr/bin/env python3
"""Genera la figura de error de traslacion AprilTag desde un log reducido."""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt


SAMPLE = re.compile(
    r'\[FID-GT-ERROR\] drone_id=(?P<drone_id>\d+) epoch=(?P<epoch>\d+) '
    r'keyframe_id=(?P<keyframe_id>\d+) tag_id=(?P<tag_id>-?\d+) '
    r'valid=(?P<valid>true|false) reason=(?P<reason>\S+) '
    r'kf_stamp_sec=(?P<kf_stamp_sec>[-+0-9.eE]+) '
    r'gt_stamp_sec=(?P<gt_stamp_sec>[-+0-9.eE]+) '
    r'gt_skew_sec=(?P<gt_skew_sec>[-+0-9.eE]+) '
    r'distance_gt_m=(?P<distance_gt_m>[-+0-9.eE]+) '
    r'translation_error_m=(?P<translation_error_m>[-+0-9.eE]+) '
    r'rotation_error_rad=(?P<rotation_error_rad>[-+0-9.eE]+)'
    r'(?: viewing_angle_deg=(?P<viewing_angle_deg>[-+0-9.eE]+))?')


def parse_samples(path):
    samples = {}
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        match = SAMPLE.search(line)
        if not match:
            continue
        row = match.groupdict()
        for key in ('drone_id', 'epoch', 'keyframe_id', 'tag_id'):
            row[key] = int(row[key])
        row['valid'] = row['valid'] == 'true'
        for key in (
            'kf_stamp_sec', 'gt_stamp_sec', 'gt_skew_sec', 'distance_gt_m',
            'translation_error_m', 'rotation_error_rad'):
            row[key] = float(row[key])
        row['viewing_angle_deg'] = (
            float(row['viewing_angle_deg'])
            if row['viewing_angle_deg'] is not None else math.nan)
        if not all(math.isfinite(row[key]) for key in (
                'gt_skew_sec', 'distance_gt_m', 'translation_error_m',
                'rotation_error_rad')):
            raise ValueError(f'muestra no finita: {line}')
        identity = tuple(row[key] for key in ('drone_id', 'epoch', 'keyframe_id', 'tag_id'))
        previous = samples.get(identity)
        if previous and previous != row:
            raise ValueError(f'muestra duplicada inconsistente para {identity}')
        samples[identity] = row
    return sorted(samples.values(), key=lambda row: (
        row['distance_gt_m'], row['kf_stamp_sec'], row['tag_id']))


def write_csv(path, samples):
    fields = [
        'drone_id', 'epoch', 'keyframe_id', 'tag_id', 'valid', 'reason',
        'kf_stamp_sec', 'gt_stamp_sec', 'gt_skew_sec', 'distance_gt_m',
        'translation_error_m', 'rotation_error_rad', 'viewing_angle_deg']
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(samples)


def metrics(samples):
    translation_errors = [row['translation_error_m'] for row in samples]
    rotation_errors = [row['rotation_error_rad'] for row in samples]
    return {
        'sample_count': len(samples),
        'accepted_count': sum(row['valid'] for row in samples),
        'reprojection_rejected_count': sum(
            not row['valid'] and row['reason'] == 'reprojection_error' for row in samples),
        'distance_gt_m_min': min(row['distance_gt_m'] for row in samples),
        'distance_gt_m_max': max(row['distance_gt_m'] for row in samples),
        'translation_error_m_mean': sum(translation_errors) / len(translation_errors),
        'translation_error_m_rmse': math.sqrt(
            sum(error * error for error in translation_errors) / len(translation_errors)),
        'translation_error_m_max': max(translation_errors),
        'rotation_error_rad_mean': sum(rotation_errors) / len(rotation_errors),
        'rotation_error_rad_rmse': math.sqrt(
            sum(error * error for error in rotation_errors) / len(rotation_errors)),
        'rotation_error_rad_max': max(rotation_errors),
        'gt_skew_sec_max': max(row['gt_skew_sec'] for row in samples),
    }


def plot(path, samples):
    fig, axes = plt.subplots(2, 1, figsize=(9.0, 7.0), sharex=True,
                             constrained_layout=True)
    groups = (
        (True, '#1b9e77', 'Aceptada'),
        (False, '#d95f02', 'Rechazada por reproyeccion'),
    )
    for valid, color, label in groups:
        group = [row for row in samples if row['valid'] == valid]
        if not group:
            continue
        x_values = [row['distance_gt_m'] for row in group]
        axes[0].scatter(
            x_values, [row['translation_error_m'] for row in group],
            color=color, edgecolors='black', linewidths=0.35, s=38,
            label=label, zorder=3)
        axes[1].scatter(
            x_values, [row['rotation_error_rad'] for row in group],
            color=color, edgecolors='black', linewidths=0.35, s=38,
            label=label, zorder=3)
    axes[0].set_title('Error AprilTag frente a distancia GT')
    axes[0].set_ylabel('Error de traslacion [m]')
    axes[1].set_xlabel('Distancia GT camara-tag [m]')
    axes[1].set_ylabel('Error de rotacion [rad]')
    for axis in axes:
        axis.grid(True, alpha=0.25, zorder=0)
    axes[0].legend(frameon=False)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    samples = parse_samples(args.input)
    if not samples:
        raise SystemExit('No se encontraron marcadores FID-GT-ERROR en el log reducido.')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / 'datos_traslacion.csv', samples)
    plot(args.output_dir / 'grafica_error_traslacion.png', samples)
    (args.output_dir / 'resumen.json').write_text(
        json.dumps(metrics(samples), indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
