#!/usr/bin/env python3
"""Diagnostico offline de la cadena lineal ORB de Fase 5H."""

import argparse
import csv
import json
import math
import re
from pathlib import Path

import numpy as np


VECTOR_RE = re.compile(r"([A-Za-z0-9_]+)=\(([^)]*)\)")
SCALAR_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
ROS_STAMP_RE = re.compile(r"\[[A-Z]+\] \[([0-9]+(?:\.[0-9]+)?)\]")


def parse_marker(line):
    values = {key: value.rstrip(',') for key, value in SCALAR_RE.findall(line)}
    for key, raw in VECTOR_RE.findall(line):
        values[key] = np.array([float(item) for item in raw.split(',')], dtype=float)
    return values


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def interpolate(rows, stamp_key, value_keys, stamp):
    stamps = np.array([row[stamp_key] for row in rows])
    if stamp < stamps[0] or stamp > stamps[-1]:
        return None
    index = int(np.searchsorted(stamps, stamp))
    if index == 0:
        return np.array([rows[0][key] for key in value_keys])
    if index == len(rows):
        return np.array([rows[-1][key] for key in value_keys])
    left, right = rows[index - 1], rows[index]
    span = right[stamp_key] - left[stamp_key]
    alpha = 0.0 if span <= 1e-12 else (stamp - left[stamp_key]) / span
    return (1.0 - alpha) * np.array([left[key] for key in value_keys]) + \
        alpha * np.array([right[key] for key in value_keys])


def metrics(errors):
    if not errors:
        return {'count': 0}
    array = np.asarray(errors)
    norms = np.linalg.norm(array, axis=1)
    return {
        'count': len(array),
        'rmse_norm': float(math.sqrt(np.mean(norms ** 2))),
        'mae_norm': float(np.mean(norms)),
        'p95_norm': float(np.percentile(norms, 95)),
        'max_norm': float(np.max(norms)),
        'bias_xyz': np.mean(array, axis=0).tolist(),
        'std_xyz': np.std(array, axis=0).tolist(),
        'rmse_xyz': np.sqrt(np.mean(array ** 2, axis=0)).tolist(),
        'max_abs_xyz': np.max(np.abs(array), axis=0).tolist(),
    }


def ratio_metrics(values):
    if not values:
        return {'count': 0}
    array = np.asarray(values)
    return {
        'count': len(array), 'median': float(np.median(array)),
        'p90': float(np.percentile(array, 90)),
        'p95': float(np.percentile(array, 95)), 'max': float(np.max(array))}


def grouped_metrics(samples, key):
    grouped = {}
    for sample in samples:
        name = sample[key]
        values = grouped.setdefault(name, {'v_mid': [], 'v_two_tk': [], 'v_hat_tk': []})
        for stage in values:
            values[stage].append(sample[stage])
    return {
        name: {stage: metrics(errors) for stage, errors in values.items()}
        for name, values in grouped.items()
    }


def reference_window(arrival_stamp, reference_stamps):
    if not reference_stamps:
        return 'stable'
    delta = min((arrival_stamp - stamp for stamp in reference_stamps), key=abs)
    if -0.5 <= delta < 0.0:
        return 'pre_500ms'
    if 0.0 <= delta <= 0.1:
        return 'immediate_100ms'
    if 0.1 < delta <= 0.5:
        return 'post_500ms'
    return 'stable'


def analyze(log_path, gt_path, drone_id=1):
    measurements, midpoint_dynamic, productive, phases, predictions, translations = [], [], [], [], [], []
    settled_stamp = None
    node_token = f'[dron_{drone_id}.orbslam3_stereo]'
    mux_token = f'[dron_{drone_id}.navigation_state_mux]'
    with log_path.open(encoding='utf-8', errors='replace') as stream:
        for line in stream:
            if '[F5H-LINEAR-MEASUREMENT]' in line and node_token in line:
                measurements.append(parse_marker(line))
            elif '[F5H-MIDPOINT-DYNAMIC]' in line and node_token in line:
                midpoint_dynamic.append(parse_marker(line))
            elif '[F5H-PRODUCTIVE-MEASUREMENT]' in line and node_token in line:
                productive.append(parse_marker(line))
            elif '[F5H-PHASE-MEASUREMENT]' in line and node_token in line:
                phases.append(parse_marker(line))
            elif '[F5H-PRODUCTIVE-PREDICT]' in line and node_token in line:
                predictions.append(parse_marker(line))
            elif '[F5H-DYNAMIC-TRANSLATION]' in line and node_token in line:
                row = parse_marker(line)
                match = ROS_STAMP_RE.search(line)
                if match:
                    row['log_stamp'] = float(match.group(1))
                translations.append(row)
            elif ('[F5H-ORB-SHADOW]' in line and 'settled=true' in line and
                  mux_token in line):
                match = ROS_STAMP_RE.search(line)
                if match and settled_stamp is None:
                    settled_stamp = float(match.group(1))

    with gt_path.open(newline='', encoding='utf-8') as stream:
        gt = [{key: float(value) if key != 'frame_id' else value
               for key, value in row.items()} for row in csv.DictReader(stream)]
    gt.sort(key=lambda row: row['gt_stamp'])
    gt_receive = sorted(gt, key=lambda row: row['gt_receive_stamp'])
    linear_keys = ['gt_linear_world_x', 'gt_linear_world_y', 'gt_linear_world_z']
    q_keys = ['gt_q_x', 'gt_q_y', 'gt_q_z', 'gt_q_w']

    phase_by_stamp = {round(float(row['input_stamp']), 6): row for row in phases}
    rotation = None
    for row in measurements:
        phase = phase_by_stamp.get(round(float(row['input_stamp']), 6))
        gt_q = interpolate(gt, 'gt_stamp', q_keys, float(row['input_stamp']))
        if phase is not None and gt_q is not None and 'raw_q' in phase:
            rotation = quaternion_matrix(gt_q) @ quaternion_matrix(phase['raw_q']).T
            break
    if rotation is None:
        raise RuntimeError('No se pudo estimar la rotacion fija O->GT')

    errors = {
        'v_mid': [], 'v_two_tk': [], 'v_hat_tk': [],
        'v_midpoint_dynamic_tk': [], 'v_dynamic_now': []}
    common_errors = {
        'v_two_tk': [], 'v_hat_tk': [], 'v_midpoint_dynamic_tk': []}
    common_by_motion = {}
    gains_hat, gains_dynamic = [], []
    groups = {}
    samples = []
    measurement_by_arrival = {}
    valid_measurements = 0
    reference_stamps = [
        float(row['arrival_stamp']) for row in measurements
        if row.get('reference_changed') == 'true' and
        (settled_stamp is None or float(row['arrival_stamp']) >= settled_stamp)
    ]
    midpoint_by_stamp = {
        round(float(row['input_stamp']), 6): row for row in midpoint_dynamic
        if row.get('valid') == 'true' and 'v_midpoint_dynamic_tk' in row
    }
    productive_by_stamp = {
        round(float(row['input_stamp']), 6): row for row in productive
        if row.get('linear_source') == 'MIDPOINT_DYNAMIC' and 'v_hat_tk' in row
    }
    for row in measurements:
        if row.get('valid') != 'true' or 'v_mid_current' not in row:
            continue
        arrival_stamp = float(row['arrival_stamp'])
        if settled_stamp is not None and arrival_stamp < settled_stamp:
            continue
        input_stamp = float(row['input_stamp'])
        mid_stamp = float(row['mid_current_stamp'])
        if mid_stamp <= 0.0:
            continue
        gt_mid = interpolate(gt, 'gt_stamp', linear_keys, mid_stamp)
        gt_tk = interpolate(gt, 'gt_stamp', linear_keys, input_stamp)
        if gt_mid is None or gt_tk is None:
            continue
        v_mid = rotation @ row['v_mid_current']
        v_hat = rotation @ row['v_hat_tk']
        e_mid, e_two, e_hat = v_mid - gt_mid, v_mid - gt_tk, v_hat - gt_tk
        errors['v_mid'].append(e_mid)
        errors['v_two_tk'].append(e_two)
        errors['v_hat_tk'].append(e_hat)
        midpoint = midpoint_by_stamp.get(round(input_stamp, 6))
        if midpoint is not None:
            e_midpoint_dynamic = rotation @ midpoint['v_midpoint_dynamic_tk'] - gt_tk
            errors['v_midpoint_dynamic_tk'].append(e_midpoint_dynamic)
            common_errors['v_two_tk'].append(e_two)
            common_errors['v_hat_tk'].append(e_hat)
            common_errors['v_midpoint_dynamic_tk'].append(e_midpoint_dynamic)
            productive_row = productive_by_stamp.get(round(input_stamp, 6))
            if productive_row is not None:
                e_productive = rotation @ productive_row['v_hat_tk'] - gt_tk
                errors.setdefault('v_productive_tk', []).append(e_productive)
                measurement_by_arrival[round(float(productive_row['base_stamp']), 6)] = \
                    np.linalg.norm(e_productive)
            motion = 'moving' if np.linalg.norm(gt_tk) > 0.03 else 'hover'
            motion_errors = common_by_motion.setdefault(
                motion, {name: [] for name in common_errors})
            motion_errors['v_two_tk'].append(e_two)
            motion_errors['v_hat_tk'].append(e_hat)
            motion_errors['v_midpoint_dynamic_tk'].append(e_midpoint_dynamic)
        gains_hat.append(np.linalg.norm(e_hat) / max(np.linalg.norm(e_mid), 1e-6))
        mode = row.get('mode', 'UNKNOWN')
        group = groups.setdefault(mode, {'v_mid': [], 'v_hat_tk': [], 'a_norm': []})
        group['v_mid'].append(e_mid)
        group['v_hat_tk'].append(e_hat)
        group['a_norm'].append(float(np.linalg.norm(row.get('a_hat', np.zeros(3)))))
        samples.append({
            'correction_class': row.get('correction_class', 'UNKNOWN'),
            'reference_window': reference_window(arrival_stamp, reference_stamps),
            'v_mid': e_mid, 'v_two_tk': e_two, 'v_hat_tk': e_hat,
        })
        if round(arrival_stamp, 6) not in measurement_by_arrival:
            measurement_by_arrival[round(arrival_stamp, 6)] = np.linalg.norm(e_hat)
        valid_measurements += 1

    for row in predictions:
        target = float(row['target_stamp'])
        if settled_stamp is not None and target < settled_stamp:
            continue
        gt_now = interpolate(gt_receive, 'gt_receive_stamp', linear_keys, target)
        if gt_now is None or 'v' not in row:
            continue
        e_dynamic = rotation @ row['v'] - gt_now
        errors['v_dynamic_now'].append(e_dynamic)
        base_error = measurement_by_arrival.get(round(float(row['base_stamp']), 6))
        if base_error is not None:
            gains_dynamic.append(np.linalg.norm(e_dynamic) / max(base_error, 1e-6))

    grouped = {}
    for name, values in groups.items():
        grouped[name] = {
            'v_mid': metrics(values['v_mid']),
            'v_hat_tk': metrics(values['v_hat_tk']),
            'a_hat_mean': float(np.mean(values['a_norm'])),
            'a_hat_max': float(np.max(values['a_norm'])),
        }
    hover_acceleration_residuals = [
        float(np.linalg.norm(row['a_O'])) for row in translations
        if 'a_O' in row and
        (settled_stamp is None or row.get('log_stamp', 0.0) >= settled_stamp)
    ]
    return {
        'valid_measurements': valid_measurements,
        'metrics': {name: metrics(value) for name, value in errors.items()},
        'common_sample_metrics': {
            name: metrics(value) for name, value in common_errors.items()},
        'common_sample_metrics_by_motion': {
            motion: {name: metrics(value) for name, value in values.items()}
            for motion, values in common_by_motion.items()},
        'midpoint_dynamic_valid': len(common_errors['v_midpoint_dynamic_tk']),
        'midpoint_dynamic_logged': len(midpoint_dynamic),
        'productive_midpoint_valid': len(errors.get('v_productive_tk', [])),
        'midpoint_dynamic_coverage_ratio': (
            len(common_errors['v_midpoint_dynamic_tk']) / valid_measurements
            if valid_measurements else 0.0),
        'gain_hat': ratio_metrics(gains_hat),
        'gain_dynamic': ratio_metrics(gains_dynamic),
        'by_linear_mode': grouped,
        'by_correction_class': grouped_metrics(samples, 'correction_class'),
        'by_reference_window': grouped_metrics(samples, 'reference_window'),
        'reference_changes': len(reference_stamps),
        'settled_receive_stamp': settled_stamp,
        'hover_acceleration_residual_norm': ratio_metrics(
            hover_acceleration_residuals),
        'rotation_o_to_gt': rotation.tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--gt-csv', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--drone-id', type=int, default=1)
    args = parser.parse_args()
    summary = analyze(args.log, args.gt_csv, args.drone_id)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
