#!/usr/bin/env python3
"""Genera la figura de error AprilTag frente al angulo de vision GT."""

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt

from generar_grafica_traslacion import parse_samples, write_csv


def metrics(samples):
    translation_errors = [row['translation_error_m'] for row in samples]
    rotation_errors = [row['rotation_error_rad'] for row in samples]
    return {
        'sample_count': len(samples),
        'accepted_count': sum(row['valid'] for row in samples),
        'reprojection_rejected_count': sum(
            not row['valid'] and row['reason'] == 'reprojection_error' for row in samples),
        'viewing_angle_deg_min': min(row['viewing_angle_deg'] for row in samples),
        'viewing_angle_deg_max': max(row['viewing_angle_deg'] for row in samples),
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
        x_values = [row['viewing_angle_deg'] for row in group]
        axes[0].scatter(x_values, [row['translation_error_m'] for row in group],
                        color=color, edgecolors='black', linewidths=0.35, s=38,
                        label=label, zorder=3)
        axes[1].scatter(x_values, [row['rotation_error_rad'] for row in group],
                        color=color, edgecolors='black', linewidths=0.35, s=38,
                        label=label, zorder=3)
    axes[0].set_title('Error AprilTag frente al angulo de vision GT')
    axes[0].set_ylabel('Error de traslacion [m]')
    axes[1].set_xlabel('Angulo entre eje optico y tag GT [deg]')
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
    samples = [row for row in parse_samples(args.input)
               if math.isfinite(row['viewing_angle_deg'])]
    if not samples:
        raise SystemExit('No se encontraron muestras FID-GT-ERROR con angulo de vision.')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / 'datos_rotacion.csv', samples)
    plot(args.output_dir / 'grafica_error_rotacion.png', samples)
    (args.output_dir / 'resumen_rotacion.json').write_text(
        json.dumps(metrics(samples), indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
