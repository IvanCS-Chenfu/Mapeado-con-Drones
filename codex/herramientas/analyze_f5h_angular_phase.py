#!/usr/bin/env python3

import argparse
import bisect
import csv
import json
import math
import os
import re
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-f5h-phase')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


MARKER_RE = re.compile(r'\[(F5H-PHASE-(?:MEASUREMENT|PUBLISH|CONTROL))\]\s+(.*)')
FIELD_RE = re.compile(r'(\w+)=((?:\([^)]*\))|[^\s]+)')
DRONE_RE = re.compile(r'(\d+)$')


def parse_fields(payload):
    return {match.group(1): match.group(2) for match in FIELD_RE.finditer(payload)}


def as_float(fields, name, default=math.nan):
    try:
        return float(fields.get(name, default))
    except (TypeError, ValueError):
        return default


def as_int(fields, name, default=-1):
    try:
        return int(fields.get(name, default))
    except (TypeError, ValueError):
        return default


def as_bool(fields, name):
    return 1 if fields.get(name) == 'true' else 0


def as_vec(fields, name):
    value = fields.get(name, '')
    if not value.startswith('(') or not value.endswith(')'):
        return np.full(3, np.nan)
    try:
        parts = [float(part) for part in value[1:-1].split(',')]
    except ValueError:
        return np.full(3, np.nan)
    return np.asarray(parts, dtype=float)


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def parse_log(path):
    data = {}
    with path.open('r', encoding='utf-8', errors='replace') as stream:
        for line in stream:
            match = MARKER_RE.search(line)
            if not match:
                continue
            marker, payload = match.groups()
            fields = parse_fields(payload)
            if marker == 'F5H-PHASE-CONTROL':
                namespace = fields.get('namespace', '')
                drone_match = DRONE_RE.search(namespace)
                if not drone_match:
                    continue
                drone = int(drone_match.group(1))
                kind = 'control'
            else:
                drone = as_int(fields, 'drone_id')
                kind = 'measurement' if marker.endswith('MEASUREMENT') else 'publish'
            if drone < 0:
                continue
            data.setdefault(drone, {'measurement': [], 'publish': [], 'control': []})[kind].append(fields)
    for streams in data.values():
        streams['measurement'].sort(key=lambda row: as_float(row, 'receive_stamp'))
        streams['publish'].sort(key=lambda row: as_float(row, 'publish_stamp'))
        streams['control'].sort(key=lambda row: as_float(row, 'control_stamp'))
    return data


def read_gt(path):
    rows = []
    with path.open('r', newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            rows.append({key: float(value) if key != 'frame_id' else value for key, value in row.items()})
    rows.sort(key=lambda row: row['gt_stamp'])
    return rows


def nearest_by_time(rows, stamps, stamp, max_skew, stamp_field):
    if not rows:
        return None
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(rows):
        candidates.append(rows[index])
    if index > 0:
        candidates.append(rows[index - 1])
    result = min(candidates, key=lambda row: abs(row[stamp_field] - stamp))
    return result if abs(result[stamp_field] - stamp) <= max_skew else None


def latest_by_time(rows, stamps, stamp, field):
    index = bisect.bisect_right(stamps, stamp) - 1
    if index < 0:
        return None
    return rows[index]


def add_vec(target, prefix, vector):
    for axis, value in zip(('x', 'y', 'z'), vector):
        target[f'{prefix}_{axis}'] = float(value)


def vector_from_row(row, prefix):
    return np.asarray([row[f'{prefix}_{axis}'] for axis in ('x', 'y', 'z')], dtype=float)


def safe_cosine(first, second):
    denominator = np.linalg.norm(first) * np.linalg.norm(second)
    if denominator < 1e-12:
        return math.nan
    return float(np.dot(first, second) / denominator)


def quaternion_distance(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    if not np.all(np.isfinite(first)) or not np.all(np.isfinite(second)):
        return math.nan
    relative = quaternion_matrix(first) @ quaternion_matrix(second).T
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def build_timeline(drone, streams, gt_rows, max_gt_skew):
    measurements = streams['measurement']
    measurement_stamps = [as_float(row, 'receive_stamp') for row in measurements]
    publishes_by_sample = {as_int(row, 'sample'): row for row in streams['publish']}
    gt_stamps = [row['gt_stamp'] for row in gt_rows]
    gt_receive_stamps = [row['gt_receive_stamp'] for row in gt_rows]
    timeline = []
    alignment = None

    for control in streams['control']:
        stamp = as_float(control, 'control_stamp')
        gt = nearest_by_time(
            gt_rows, gt_receive_stamps, stamp, max_gt_skew,
            'gt_receive_stamp')
        if gt is None:
            continue
        measurement = latest_by_time(
            measurements, measurement_stamps, stamp, 'receive_stamp')
        publish = publishes_by_sample.get(as_int(control, 'sample'))
        r_act = quaternion_matrix(as_vec(control, 'r_act_q'))
        r_des = quaternion_matrix(as_vec(control, 'r_des_q'))
        r_gt_world = quaternion_matrix(np.array([
            gt['gt_q_x'], gt['gt_q_y'], gt['gt_q_z'], gt['gt_q_w']]))
        if alignment is None and as_int(control, 'source') == 1:
            alignment = r_act @ r_gt_world.T
        r_gt_o = (alignment @ r_gt_world) if alignment is not None else r_act

        gt_omega_body = np.array([
            gt['gt_omega_body_x'], gt['gt_omega_body_y'], gt['gt_omega_body_z']])
        omega_body = as_vec(control, 'omega_body')
        omega_des_body = as_vec(control, 'omega_des_body')
        ew = as_vec(control, 'ew')
        tau_er = as_vec(control, 'tau_er')
        tau_ew = as_vec(control, 'tau_ew')
        tau_total = as_vec(control, 'tau_total')
        kw = as_float(control, 'kw')
        ew_gt = gt_omega_body - r_gt_o.T @ r_des @ omega_des_body
        tau_ew_ideal = -kw * ew_gt

        output = {
            'drone_id': drone,
            'time': stamp,
            'state_stamp': as_float(control, 'state_stamp'),
            'state_receive_stamp': as_float(control, 'state_receive_stamp'),
            'source': as_int(control, 'source'),
            'epoch': as_int(control, 'epoch'),
            'sample': as_int(control, 'sample'),
            'tracking_state': as_int(control, 'tracking'),
            'reference_kf': as_int(control, 'ref_kf'),
            'gt_stamp': gt['gt_stamp'],
            'gt_receive_stamp': gt['gt_receive_stamp'],
            'gt_receive_skew_sec': abs(gt['gt_receive_stamp'] - stamp),
            'kr': as_float(control, 'kr'),
            'kw': kw,
            'force': as_float(control, 'force'),
        }
        for name in ('er', 'ew', 'tau_er', 'tau_ew', 'tau_feedforward',
                     'tau_gyro', 'tau_total', 'omega_body', 'omega_o',
                     'omega_des_body'):
            add_vec(output, name, as_vec(control, name))
        add_vec(output, 'gt_omega_body', gt_omega_body)
        add_vec(output, 'ew_gt', ew_gt)
        add_vec(output, 'tau_ew_ideal', tau_ew_ideal)

        if measurement is not None:
            raw_o = as_vec(measurement, 'raw_omega_o')
            motion_o = as_vec(measurement, 'motion_o')
            raw_body = quaternion_matrix(as_vec(measurement, 'raw_q')).T @ raw_o
            motion_body = r_act.T @ motion_o
            add_vec(output, 'raw_omega_body', raw_body)
            add_vec(output, 'motion_omega_body', motion_body)
            output['measurement_input_stamp'] = as_float(measurement, 'input_stamp')
            output['measurement_receive_stamp'] = as_float(measurement, 'receive_stamp')
            output['raw_class'] = measurement.get('raw_class', '')
            output['correction_class'] = measurement.get('correction_class', '')
            output['base_stamp'] = as_float(measurement, 'base_stamp')
            output['base_update_applied'] = as_bool(
                measurement, 'base_update_applied')
            output['base_update_type'] = measurement.get(
                'base_update_type', '')
            output['base_correction_rad'] = as_float(
                measurement, 'base_correction_rad')
            output['visual_base_error_before_rad'] = as_float(
                measurement, 'visual_base_error_before')
            output['visual_base_error_after_rad'] = as_float(
                measurement, 'visual_base_error_after')
            input_gt = nearest_by_time(
                gt_rows, gt_stamps, output['measurement_input_stamp'],
                max_gt_skew, 'gt_stamp')
            if input_gt is not None:
                output['measurement_input_ros_estimate'] = (
                    input_gt['gt_receive_stamp'] +
                    output['measurement_input_stamp'] - input_gt['gt_stamp'])
                output['input_to_receive_sec'] = (
                    output['measurement_receive_stamp'] -
                    output['measurement_input_ros_estimate'])
            else:
                output['measurement_input_ros_estimate'] = math.nan
                output['input_to_receive_sec'] = math.nan
        else:
            add_vec(output, 'raw_omega_body', np.full(3, np.nan))
            add_vec(output, 'motion_omega_body', np.full(3, np.nan))
            output.update({
                'measurement_input_stamp': math.nan,
                'measurement_receive_stamp': math.nan,
                'measurement_input_ros_estimate': math.nan,
                'raw_class': '', 'correction_class': '',
                'base_stamp': math.nan, 'base_update_applied': 0,
                'base_update_type': '', 'base_correction_rad': math.nan,
                'visual_base_error_before_rad': math.nan,
                'visual_base_error_after_rad': math.nan,
                'input_to_receive_sec': math.nan,
            })

        if publish is not None:
            output['publish_stamp'] = as_float(publish, 'publish_stamp')
            output['visual_age_sec'] = (
                stamp - output['measurement_input_ros_estimate']
                if math.isfinite(output['measurement_input_ros_estimate']) else math.nan)
            output['receive_age_sec'] = stamp - as_float(publish, 'receive_stamp')
            output['publish_to_control_receive_sec'] = (
                as_float(control, 'state_receive_stamp') - output['publish_stamp'])
            output['publish_to_control_tick_sec'] = stamp - output['publish_stamp']
            output['local_visual_age_sec'] = as_float(
                publish, 'visual_age_local')
            output['prediction_horizon_sec'] = as_float(
                publish, 'prediction_horizon')
            output['prediction_clamped'] = (
                1 if publish.get('prediction_clamped') == 'true' else 0)
            output['visual_base_error_rad'] = quaternion_distance(
                as_vec(publish, 'visual_q'), as_vec(publish, 'base_q'))
            output['base_predicted_step_rad'] = quaternion_distance(
                as_vec(publish, 'base_q'), as_vec(publish, 'predicted_q'))
        else:
            output.update({
                'publish_stamp': math.nan, 'visual_age_sec': math.nan,
                'receive_age_sec': math.nan,
                'publish_to_control_receive_sec': math.nan,
                'publish_to_control_tick_sec': math.nan,
                'local_visual_age_sec': math.nan,
                'prediction_horizon_sec': math.nan,
                'prediction_clamped': 0,
                'visual_base_error_rad': math.nan,
                'base_predicted_step_rad': math.nan,
            })

        output['p_er_gt'] = float(np.dot(tau_er, gt_omega_body))
        output['p_damping_gt'] = float(np.dot(tau_ew, gt_omega_body))
        output['p_total_gt'] = float(np.dot(tau_total, gt_omega_body))
        output['tau_ew_ideal_alignment'] = safe_cosine(tau_ew, tau_ew_ideal)
        output['tau_ew_ideal_error_norm'] = float(np.linalg.norm(tau_ew - tau_ew_ideal))
        output['gt_omega_norm'] = float(np.linalg.norm(gt_omega_body))
        output['control_omega_norm'] = float(np.linalg.norm(omega_body))
        output['er_norm'] = float(np.linalg.norm(as_vec(control, 'er')))
        output['ew_norm'] = float(np.linalg.norm(ew))
        output['tau_ew_norm'] = float(np.linalg.norm(tau_ew))
        timeline.append(output)
    return timeline


def cross_correlation(time, reference, signal, max_lag_sec=0.5):
    finite = np.isfinite(reference) & np.isfinite(signal)
    if np.count_nonzero(finite) < 20:
        return {'lag_sec': None, 'correlation': None, 'lags': [], 'values': []}
    reference = reference[finite]
    signal = signal[finite]
    sample_time = time[finite]
    dt = float(np.median(np.diff(sample_time)))
    if not math.isfinite(dt) or dt <= 0.0:
        return {'lag_sec': None, 'correlation': None, 'lags': [], 'values': []}
    max_lag = max(1, int(round(max_lag_sec / dt)))
    lags = []
    values = []
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            first, second = reference[:len(reference) - lag or None], signal[lag:]
        else:
            first, second = reference[-lag:], signal[:len(signal) + lag]
        if len(first) < 10 or np.std(first) < 1e-9 or np.std(second) < 1e-9:
            correlation = math.nan
        else:
            correlation = float(np.corrcoef(first, second)[0, 1])
        lags.append(lag * dt)
        values.append(correlation)
    finite_values = np.isfinite(values)
    if not np.any(finite_values):
        return {'lag_sec': None, 'correlation': None, 'lags': lags, 'values': values}
    indices = np.flatnonzero(finite_values)
    best = indices[np.argmax(np.abs(np.asarray(values)[indices]))]
    return {
        'lag_sec': float(lags[best]),
        'correlation': float(values[best]),
        'lags': lags,
        'values': values,
    }


def first_time(rows, predicate, origin):
    for row in rows:
        if predicate(row):
            return row['time'] - origin
    return None


def longest_positive_interval(rows, field):
    longest = 0.0
    start = None
    previous = None
    for row in rows:
        positive = row['source'] == 1 and row[field] > 0.0
        if positive and start is None:
            start = row['time']
        if not positive and start is not None:
            longest = max(longest, (previous or start) - start)
            start = None
        previous = row['time']
    if start is not None and previous is not None:
        longest = max(longest, previous - start)
    return longest


def dominant_frequency(time, values):
    if len(time) < 20:
        return None
    dt = float(np.median(np.diff(time)))
    centered = values - np.mean(values)
    if dt <= 0.0 or np.std(centered) < 1e-9:
        return None
    frequencies = np.fft.rfftfreq(len(centered), dt)
    power = np.abs(np.fft.rfft(centered * np.hanning(len(centered)))) ** 2
    valid = frequencies >= 0.2
    if not np.any(valid):
        return None
    index = np.flatnonzero(valid)[np.argmax(power[valid])]
    return float(frequencies[index])


def power_summary(rows, field, start_offset_sec=0.6):
    if not rows:
        return None
    origin = rows[0]['time']
    selected = [
        row for row in rows
        if row['time'] - origin >= start_offset_sec and math.isfinite(row[field])]
    if len(selected) < 2:
        return None
    net = 0.0
    positive = 0.0
    negative = 0.0
    for previous, current in zip(selected, selected[1:]):
        dt = max(0.0, current['time'] - previous['time'])
        energy = 0.5 * (previous[field] + current[field]) * dt
        net += energy
        if energy > 0.0:
            positive += energy
        else:
            negative += energy
    duration = selected[-1]['time'] - selected[0]['time']
    return {
        'positive_work_ratio': float(np.mean([row[field] > 0.0 for row in selected])),
        'net_energy_j': net,
        'positive_energy_j': positive,
        'negative_energy_j': negative,
        'duration_sec': duration,
        'mean_power_w': (
            float(np.mean([row[field] for row in selected]))),
        'net_energy_per_sec_jps': net / duration if duration > 0.0 else None,
    }


def unique_measurement_rows(rows):
    unique = []
    seen = set()
    for row in rows:
        stamp = row.get('measurement_receive_stamp', math.nan)
        if not math.isfinite(stamp) or stamp in seen:
            continue
        seen.add(stamp)
        unique.append(row)
    return unique


def angular_error_summary(reference, signal, direction_threshold=0.01):
    finite = np.all(np.isfinite(reference), axis=1) & np.all(np.isfinite(signal), axis=1)
    if not np.any(finite):
        return None
    reference = reference[finite]
    signal = signal[finite]
    error_norm = np.linalg.norm(signal - reference, axis=1)
    reference_norm = np.linalg.norm(reference, axis=1)
    signal_norm = np.linalg.norm(signal, axis=1)
    directional = (reference_norm > direction_threshold) & (signal_norm > direction_threshold)
    mismatch_ratio = None
    directional_samples = int(np.sum(directional))
    if directional_samples:
        mismatch_ratio = float(np.mean(
            np.sum(reference[directional] * signal[directional], axis=1) < 0.0))
    return {
        'rmse_radps': float(np.sqrt(np.mean(error_norm ** 2))),
        'mae_radps': float(np.mean(error_norm)),
        'max_error_radps': float(np.max(error_norm)),
        'direction_mismatch_ratio': mismatch_ratio,
        'direction_samples': directional_samples,
        'direction_threshold_radps': direction_threshold,
    }


def analyze_timeline(rows, common_window_sec=None):
    orb = [row for row in rows if row['source'] == 1]
    if len(orb) < 20:
        return {'status': 'DATOS_INSUFICIENTES', 'orb_samples': len(orb)}
    origin = orb[0]['time']
    measurement_orb = unique_measurement_rows(orb)
    time = np.asarray([row['time'] for row in orb])
    gt = np.asarray([[row[f'gt_omega_body_{axis}'] for axis in ('x', 'y', 'z')]
                     for row in orb])
    raw = np.asarray([[row[f'raw_omega_body_{axis}'] for axis in ('x', 'y', 'z')]
                      for row in orb])
    motion = np.asarray([[row[f'motion_omega_body_{axis}'] for axis in ('x', 'y', 'z')]
                         for row in orb])
    control = np.asarray([[row[f'omega_body_{axis}'] for axis in ('x', 'y', 'z')]
                          for row in orb])
    angular_errors = {
        'motion_vs_gt': angular_error_summary(gt, motion),
        'control_vs_gt': angular_error_summary(gt, control),
    }
    dominant_axis = int(np.argmax(np.nanstd(gt, axis=0)))
    frequency = dominant_frequency(time, gt[:, dominant_axis])
    correlations = {}
    for axis_index, axis in enumerate(('x', 'y', 'z')):
        correlations[axis] = {}
        for name, signal in (('raw', raw), ('motion', motion), ('control', control)):
            result = cross_correlation(time, gt[:, axis_index], signal[:, axis_index])
            correlations[axis][name] = {
                'lag_sec': result['lag_sec'],
                'correlation': result['correlation'],
                'phase_deg': (
                    360.0 * frequency * result['lag_sec']
                    if frequency is not None and result['lag_sec'] is not None else None),
            }
    positive = [row for row in orb if row['p_damping_gt'] > 0.0]
    alignments = np.asarray([
        row['tau_ew_ideal_alignment'] for row in orb
        if math.isfinite(row['tau_ew_ideal_alignment'])])
    local_visual_ages = np.asarray([
        row['local_visual_age_sec'] for row in orb
        if math.isfinite(row['local_visual_age_sec'])])
    prediction_horizons = np.asarray([
        row['prediction_horizon_sec'] for row in orb
        if math.isfinite(row['prediction_horizon_sec'])])
    visual_base_errors = np.asarray([
        row['visual_base_error_rad'] for row in orb
        if math.isfinite(row['visual_base_error_rad'])])
    visual_base_errors_before = np.asarray([
        row['visual_base_error_before_rad'] for row in measurement_orb
        if math.isfinite(row['visual_base_error_before_rad'])])
    visual_base_errors_after = np.asarray([
        row['visual_base_error_after_rad'] for row in measurement_orb
        if math.isfinite(row['visual_base_error_after_rad'])])
    base_update_counts = {}
    for row in measurement_orb:
        update_type = row['base_update_type']
        if update_type:
            base_update_counts[update_type] = base_update_counts.get(update_type, 0) + 1
    common_rows = (
        [row for row in orb if row['time'] - origin <= common_window_sec]
        if common_window_sec is not None else [])
    chronology = {
        'orb_start': 0.0,
        'first_gt_growth': first_time(orb, lambda row: row['gt_omega_norm'] > 0.05, origin),
        'first_raw_growth': first_time(orb, lambda row: np.linalg.norm(
            vector_from_row(row, 'raw_omega_body')) > 0.05, origin),
        'first_motion_growth': first_time(orb, lambda row: np.linalg.norm(
            vector_from_row(row, 'motion_omega_body')) > 0.05, origin),
        'first_ew_growth': first_time(orb, lambda row: row['ew_norm'] > 0.05, origin),
        'first_tau_ew_growth': first_time(orb, lambda row: row['tau_ew_norm'] > 0.005, origin),
        'first_positive_damping_power': first_time(
            orb, lambda row: row['p_damping_gt'] > 0.0, origin),
        'first_raw_rejected': first_time(
            orb, lambda row: row['raw_class'] in ('REJECTED', 'SUSPICIOUS'), origin),
        'fallback': first_time(
            [row for row in rows if row['time'] >= origin],
            lambda row: row['source'] != 1, origin),
        'tracking_not_ok': first_time(
            [row for row in rows if row['time'] >= origin],
            lambda row: row['tracking_state'] != 2, origin),
    }
    return {
        'status': 'MEDIDO_PENDIENTE_INTERPRETACION',
        'orb_samples': len(orb),
        'orb_duration_sec': orb[-1]['time'] - orb[0]['time'],
        'dominant_axis': ('x', 'y', 'z')[dominant_axis],
        'dominant_frequency_hz': frequency,
        'cross_correlation': correlations,
        'angular_error': angular_errors,
        'positive_damping_power_ratio': len(positive) / len(orb),
        'longest_positive_damping_interval_sec': longest_positive_interval(orb, 'p_damping_gt'),
        'tau_ew_ideal_alignment_mean': (
            float(np.mean(alignments)) if alignments.size else None),
        'tau_ew_ideal_alignment_negative_ratio': (
            float(np.mean(alignments < 0.0)) if alignments.size else None),
        'max_gt_omega_radps': max(row['gt_omega_norm'] for row in orb),
        'max_control_omega_radps': max(row['control_omega_norm'] for row in orb),
        'max_er_rad': max(row['er_norm'] for row in orb),
        'max_ew_radps': max(row['ew_norm'] for row in orb),
        'max_tau_ew': max(row['tau_ew_norm'] for row in orb),
        'tau_er_work_post_handoff': power_summary(orb, 'p_er_gt'),
        'tau_ew_work_post_handoff': power_summary(orb, 'p_damping_gt'),
        'tau_total_work_post_handoff': power_summary(orb, 'p_total_gt'),
        'common_window_sec': common_window_sec,
        'common_window_samples': len(common_rows),
        'tau_er_work_common_window': (
            power_summary(common_rows, 'p_er_gt') if common_rows else None),
        'tau_ew_work_common_window': (
            power_summary(common_rows, 'p_damping_gt') if common_rows else None),
        'tau_total_work_common_window': (
            power_summary(common_rows, 'p_total_gt') if common_rows else None),
        'mean_local_visual_age_sec': (
            float(np.mean(local_visual_ages)) if local_visual_ages.size else None),
        'max_local_visual_age_sec': (
            float(np.max(local_visual_ages)) if local_visual_ages.size else None),
        'mean_prediction_horizon_sec': (
            float(np.mean(prediction_horizons)) if prediction_horizons.size else None),
        'prediction_clamped_ratio': (
            float(np.mean([row['prediction_clamped'] for row in orb]))
            if prediction_horizons.size else None),
        'max_visual_base_error_rad': (
            float(np.max(visual_base_errors)) if visual_base_errors.size else None),
        'mean_visual_base_error_before_rad': (
            float(np.mean(visual_base_errors_before))
            if visual_base_errors_before.size else None),
        'max_visual_base_error_before_rad': (
            float(np.max(visual_base_errors_before))
            if visual_base_errors_before.size else None),
        'mean_visual_base_error_after_rad': (
            float(np.mean(visual_base_errors_after))
            if visual_base_errors_after.size else None),
        'max_visual_base_error_after_rad': (
            float(np.max(visual_base_errors_after))
            if visual_base_errors_after.size else None),
        'base_update_counts': base_update_counts,
        'unique_orb_measurements': len(measurement_orb),
        'base_update_applied_ratio': (
            float(np.mean([row['base_update_applied'] for row in measurement_orb]))
            if measurement_orb else None),
        'chronology_sec_from_orb': chronology,
    }


def write_timeline(path, rows):
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(output_dir, rows, summary):
    orb = [row for row in rows if row['source'] == 1]
    if not orb:
        return
    time = np.asarray([row['time'] - orb[0]['time'] for row in orb])
    axes_names = ('x', 'y', 'z')

    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for index, axis in enumerate(axes_names):
        for prefix, label in (
            ('gt_omega_body', 'GT'), ('raw_omega_body', 'raw'),
            ('motion_omega_body', 'motion'), ('omega_body', 'control')):
            axes[index].plot(time, [row[f'{prefix}_{axis}'] for row in orb], label=label)
        axes[index].set_ylabel(f'omega {axis} [rad/s]')
        axes[index].grid(True, alpha=0.25)
    axes[0].legend(ncol=4)
    axes[-1].set_xlabel('time from ORB [s]')
    figure.tight_layout()
    figure.savefig(output_dir / 'omega_gt_raw_motion_control.png', dpi=140)
    plt.close(figure)

    figure, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for index, axis in enumerate(axes_names):
        axes[index].plot(time, [row[f'gt_omega_body_{axis}'] for row in orb], label='GT omega')
        axes[index].plot(time, [row[f'ew_{axis}'] for row in orb], label='ew')
        axes[index].plot(time, [row[f'tau_ew_{axis}'] for row in orb], label='tau_ew')
        axes[index].grid(True, alpha=0.25)
        axes[index].set_ylabel(axis)
    axes[0].legend(ncol=3)
    axes[-1].set_xlabel('time from ORB [s]')
    figure.tight_layout()
    figure.savefig(output_dir / 'ew_tau_omega_gt.png', dpi=140)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(12, 4))
    axis.plot(time, [row['p_er_gt'] for row in orb], label='P er GT')
    axis.plot(time, [row['p_damping_gt'] for row in orb], label='P damping GT')
    axis.plot(time, [row['p_total_gt'] for row in orb], label='P total GT', alpha=0.7)
    axis.axhline(0.0, color='black', linewidth=0.8)
    axis.set_xlabel('time from ORB [s]')
    axis.set_ylabel('power [W]')
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / 'angular_power.png', dpi=140)
    plt.close(figure)

    all_time = np.asarray([row['time'] - orb[0]['time'] for row in rows])
    figure, axes = plt.subplots(2, 1, figsize=(12, 5), sharex=True)
    axes[0].step(all_time, [row['source'] for row in rows], where='post')
    axes[1].step(all_time, [row['tracking_state'] for row in rows], where='post')
    axes[0].set_ylabel('source')
    axes[1].set_ylabel('tracking')
    axes[1].set_xlabel('time from ORB [s]')
    figure.tight_layout()
    figure.savefig(output_dir / 'source_tracking.png', dpi=140)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(12, 4))
    for field in ('visual_age_sec', 'local_visual_age_sec',
                  'prediction_horizon_sec', 'receive_age_sec',
                  'publish_to_control_receive_sec', 'publish_to_control_tick_sec'):
        axis.plot(time, [row[field] for row in orb], label=field)
    axis.set_xlabel('time from ORB [s]')
    axis.set_ylabel('age [s]')
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=2)
    figure.tight_layout()
    figure.savefig(output_dir / 'temporal_ages.png', dpi=140)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(12, 4))
    axis.plot(
        time, [row['visual_base_error_before_rad'] for row in orb],
        label='visual-base before')
    axis.plot(
        time, [row['visual_base_error_after_rad'] for row in orb],
        label='visual-base after')
    axis.set_xlabel('time from ORB [s]')
    axis.set_ylabel('orientation error [rad]')
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / 'visual_base_anchor_error.png', dpi=140)
    plt.close(figure)

    dominant_axis = summary.get('dominant_axis')
    if dominant_axis:
        axis_index = axes_names.index(dominant_axis)
        gt = np.asarray([row[f'gt_omega_body_{dominant_axis}'] for row in orb])
        figure, axis = plt.subplots(figsize=(12, 4))
        for prefix, label in (
            ('raw_omega_body', 'raw'), ('motion_omega_body', 'motion'),
            ('omega_body', 'control')):
            signal = np.asarray([row[f'{prefix}_{dominant_axis}'] for row in orb])
            result = cross_correlation(
                np.asarray([row['time'] for row in orb]), gt, signal)
            axis.plot(result['lags'], result['values'], label=label)
        axis.axvline(0.0, color='black', linewidth=0.8)
        axis.set_xlabel('positive lag means estimated signal delayed [s]')
        axis.set_ylabel('correlation')
        axis.grid(True, alpha=0.25)
        axis.legend()
        figure.tight_layout()
        figure.savefig(output_dir / 'cross_correlation.png', dpi=140)
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--metrics-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--max-gt-skew-sec', type=float, default=0.04)
    parser.add_argument('--common-window-sec', type=float)
    args = parser.parse_args()

    streams_by_drone = parse_log(args.log)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = {'source_log': str(args.log), 'drones': {}}
    for drone, streams in sorted(streams_by_drone.items()):
        gt_path = args.metrics_dir / f'drone_{drone}_gt_angular.csv'
        if not gt_path.exists():
            report['drones'][str(drone)] = {
                'status': 'DATOS_INSUFICIENTES', 'reason': f'missing {gt_path}'}
            continue
        drone_dir = args.output_dir / f'drone_{drone}'
        drone_dir.mkdir(parents=True, exist_ok=True)
        timeline = build_timeline(
            drone, streams, read_gt(gt_path), args.max_gt_skew_sec)
        summary = analyze_timeline(timeline, args.common_window_sec)
        write_timeline(drone_dir / 'timeline.csv', timeline)
        plot_results(drone_dir, timeline, summary)
        (drone_dir / 'summary.json').write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding='utf-8')
        report['drones'][str(drone)] = summary
    (args.output_dir / 'summary.json').write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding='utf-8')


if __name__ == '__main__':
    main()
