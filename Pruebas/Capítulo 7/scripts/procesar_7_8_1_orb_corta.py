#!/usr/bin/env python3
"""Procesa una ejecucion de la prueba 7.8.1 con perfil cubico o trapezoidal."""
import argparse
import bisect
import csv
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-chapter7')


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


def read_csv(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def nearest(rows, times, instant, key, max_skew):
    if not rows:
        return None
    index = bisect.bisect_left(times, instant)
    candidates = []
    if index < len(rows):
        candidates.append(rows[index])
    if index:
        candidates.append(rows[index - 1])
    result = min(candidates, key=lambda row: abs(number(row[key]) - instant))
    return result if abs(number(result[key]) - instant) <= max_skew else None


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def metric(values):
    values = [value for value in values if math.isfinite(value)]
    if not values:
        return {'count': 0, 'rmse': None, 'mae': None, 'max': None}
    return {
        'count': len(values),
        'rmse': math.sqrt(sum(value * value for value in values) / len(values)),
        'mae': sum(abs(value) for value in values) / len(values),
        'max': max(abs(value) for value in values),
    }


def vector_norm(values):
    return math.sqrt(sum(value * value for value in values))


def trajectory_segments(rows):
    segments = []
    current = []
    previous_t_act = None
    for row in rows:
        t_act = number(row['t_act'])
        if current and math.isfinite(previous_t_act) and t_act + 0.25 < previous_t_act:
            segments.append(current)
            current = []
        current.append(row)
        previous_t_act = t_act
    if current:
        segments.append(current)
    return segments


def select_orb_segment(rows):
    candidates = []
    for segment in trajectory_segments(rows):
        if len(segment) < 3:
            continue
        endpoint = number(segment[-1]['x_pos'])
        minimum = min(number(row['x_pos']) for row in segment)
        if minimum <= -1.0:
            candidates.append((abs(endpoint + 7.0), -len(segment), segment))
    if not candidates:
        raise RuntimeError('No se encontro el segmento de referencia cuyo destino es x=-7 m')
    return min(candidates, key=lambda item: (item[0], item[1]))[2]


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def finite_derivative(time, values):
    derivative = []
    for index, value in enumerate(values):
        if not math.isfinite(value):
            derivative.append(math.nan)
            continue
        before = max(0, index - 1)
        after = min(len(values) - 1, index + 1)
        delta_t = time[after] - time[before]
        if delta_t <= 0.0 or not math.isfinite(values[before]) or not math.isfinite(values[after]):
            derivative.append(math.nan)
        else:
            derivative.append((values[after] - values[before]) / delta_t)
    return derivative


def plot_gt_vs_trajectory_3x4(output_dir, rows, paths):
    import matplotlib.pyplot as plt
    time = [row['time_sec'] for row in rows]
    gt_acceleration = {
        axis: finite_derivative(time, [row[f'gt_v{axis}'] for row in rows])
        for axis in ('x', 'y', 'z')
    }
    gt_acceleration['yaw'] = finite_derivative(time, [row['gt_omega_z'] for row in rows])
    trajectory = {
        'position': {'x': 'ref_x', 'y': 'ref_y', 'z': 'ref_z', 'yaw': 'ref_yaw'},
        'velocity': {'x': 'ref_vx', 'y': 'ref_vy', 'z': 'ref_vz', 'yaw': 'ref_yaw_vel'},
        'acceleration': {'x': 'ref_ax', 'y': 'ref_ay', 'z': 'ref_az', 'yaw': 'ref_yaw_acc'},
    }
    ground_truth = {
        'position': {'x': 'gt_x', 'y': 'gt_y', 'z': 'gt_z', 'yaw': 'gt_yaw'},
        'velocity': {'x': 'gt_vx', 'y': 'gt_vy', 'z': 'gt_vz', 'yaw': 'gt_omega_z'},
        'acceleration': {'x': None, 'y': None, 'z': None, 'yaw': None},
    }
    titles = {'position': 'Posicion', 'velocity': 'Velocidad', 'acceleration': 'Aceleracion'}
    units = {'position': ('m', 'rad'), 'velocity': ('m/s', 'rad/s'), 'acceleration': ('m/s²', 'rad/s²')}
    axes_names = ('x', 'y', 'z', 'yaw')

    figure, axes = plt.subplots(3, 4, figsize=(18, 10), sharex=False)
    for row_index, quantity in enumerate(('position', 'velocity', 'acceleration')):
        for column, axis_name in enumerate(axes_names):
            axis = axes[row_index, column]
            reference = [row[trajectory[quantity][axis_name]] for row in rows]
            if quantity == 'acceleration':
                gt_values = gt_acceleration[axis_name]
                gt_label = 'GT derivada'
            else:
                gt_values = [row[ground_truth[quantity][axis_name]] for row in rows]
                gt_label = 'GT alineado'
            axis.plot(time, gt_values, label=gt_label, linewidth=1.0)
            axis.plot(time, reference, label='Trayectoria', linewidth=1.0)
            axis.set_title(f'{titles[quantity]} {axis_name}')
            axis.set_ylabel(units[quantity][1] if axis_name == 'yaw' else units[quantity][0])
            axis.grid(True, alpha=0.25)
            if row_index == 2:
                axis.set_xlabel('Tiempo desde inicio ORB [s]')
            if row_index == 0 and column == 0:
                axis.legend(fontsize=8)
    figure.suptitle('7.8.1-A cubica: GT frente a trayectoria')
    figure.tight_layout()
    comparison_path = output_dir / 'gt_vs_trayectoria_3x4.png'
    figure.savefig(comparison_path, dpi=180)
    plt.close(figure)
    paths.append(comparison_path.name)

    figure, axes = plt.subplots(3, 4, figsize=(18, 10), sharex=False)
    for row_index, quantity in enumerate(('position', 'velocity', 'acceleration')):
        for column, axis_name in enumerate(axes_names):
            axis = axes[row_index, column]
            reference = [row[trajectory[quantity][axis_name]] for row in rows]
            if quantity == 'acceleration':
                gt_values = gt_acceleration[axis_name]
            else:
                gt_values = [row[ground_truth[quantity][axis_name]] for row in rows]
            error = [reference_value - gt_value if math.isfinite(reference_value) and math.isfinite(gt_value) else math.nan
                     for reference_value, gt_value in zip(reference, gt_values)]
            axis.plot(time, error, color='#b22222', linewidth=1.0)
            axis.axhline(0.0, color='black', linewidth=0.6)
            axis.set_title(f'Error {titles[quantity].lower()} {axis_name}')
            axis.set_ylabel(units[quantity][1] if axis_name == 'yaw' else units[quantity][0])
            axis.grid(True, alpha=0.25)
            if row_index == 2:
                axis.set_xlabel('Tiempo desde inicio ORB [s]')
    figure.suptitle('7.8.1-A cubica: error trayectoria - GT')
    figure.tight_layout()
    error_path = output_dir / 'errores_gt_vs_trayectoria_3x4.png'
    figure.savefig(error_path, dpi=180)
    plt.close(figure)
    paths.append(error_path.name)


def save_figures(output_dir, rows):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    if not rows:
        return []
    time = [row['time_sec'] for row in rows]
    paths = []

    figure, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    for axis, name in zip(axes[:3], ('x', 'y', 'z')):
        axis.plot(time, [row[f'ref_{name}'] for row in rows], label='referencia')
        axis.plot(time, [row[f'orb_{name}'] for row in rows], label='ORB')
        axis.set_ylabel(f'{name} [m]')
        axis.grid(True, alpha=0.25)
    axes[3].plot(time, [row['ref_yaw'] for row in rows], label='referencia')
    axes[3].plot(time, [row['orb_yaw'] for row in rows], label='ORB')
    axes[3].set_ylabel('yaw [rad]')
    axes[3].set_xlabel('tiempo desde inicio ORB [s]')
    axes[3].grid(True, alpha=0.25)
    axes[0].legend()
    figure.tight_layout()
    path = output_dir / 'figura_a_referencia_orb.png'
    figure.savefig(path, dpi=160)
    plt.close(figure)
    paths.append(path.name)

    figure, axis = plt.subplots(figsize=(12, 5))
    for name in ('x', 'y', 'z'):
        axis.plot(time, [row[f'control_error_{name}'] for row in rows], label=f'e_{name}')
    axis.plot(time, [row['control_error_norm'] for row in rows], label='norma', linewidth=2.0)
    axis.set_xlabel('tiempo desde inicio ORB [s]')
    axis.set_ylabel('error de control [m]')
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    path = output_dir / 'figura_b_error_seguimiento.png'
    figure.savefig(path, dpi=160)
    plt.close(figure)
    paths.append(path.name)

    figure, axis = plt.subplots(figsize=(12, 5))
    axis.plot(time, [row['orb_gt_error_norm'] for row in rows], label='ORB frente a GT alineado')
    for row in rows:
        if row['keyframe_changed']:
            axis.axvline(row['time_sec'], color='#888888', linewidth=0.7, alpha=0.5)
    axis.set_xlabel('tiempo desde inicio ORB [s]')
    axis.set_ylabel('error ORB-GT [m]')
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    path = output_dir / 'figura_c_orb_gt.png'
    figure.savefig(path, dpi=160)
    plt.close(figure)
    paths.append(path.name)

    figure, axis = plt.subplots(figsize=(12, 5))
    axis.plot(time, [row['ref_vx'] for row in rows], label='referencia x')
    axis.plot(time, [row['orb_vx'] for row in rows], label='ORB x')
    axis.plot(time, [row['gt_vx_aligned'] for row in rows], label='GT alineado x')
    axis.set_xlabel('tiempo desde inicio ORB [s]')
    axis.set_ylabel('velocidad x [m/s]')
    axis.grid(True, alpha=0.25)
    axis.legend()
    figure.tight_layout()
    path = output_dir / 'figura_d_velocidad.png'
    figure.savefig(path, dpi=160)
    plt.close(figure)
    paths.append(path.name)

    plot_gt_vs_trajectory_3x4(output_dir, rows, paths)
    return paths


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('raw_dir', type=Path)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--profile', choices=('cubic', 'trapezoidal'), default='cubic')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    navigation = read_csv(args.raw_dir / 'navigation_state.csv')
    gt_pose = read_csv(args.raw_dir / 'gt_pose.csv')
    gt_velocity = read_csv(args.raw_dir / 'gt_velocity.csv')
    trajectory = read_csv(args.raw_dir / 'trajectory_feedback.csv')
    navigation.sort(key=lambda row: number(row['receive_sec']))
    gt_pose.sort(key=lambda row: number(row['stamp_sec']))
    gt_velocity.sort(key=lambda row: number(row['stamp_sec']))
    trajectory.sort(key=lambda row: number(row['receive_sec']))
    reference = select_orb_segment(trajectory)
    reference_times = [number(row['receive_sec']) for row in reference]
    nav_start, nav_end = reference_times[0], reference_times[-1]
    nav_window = [
        row for row in navigation
        if nav_start - 0.10 <= number(row['receive_sec']) <= nav_end + 0.10
    ]
    orb_window = [
        row for row in nav_window
        if integer(row['pose_source']) == 1 and integer(row['local_valid']) == 1
        and integer(row['local_continuity_valid']) == 1 and integer(row['velocity_valid']) == 1
    ]
    if not orb_window:
        raise RuntimeError('No hay NavigationState local valido con pose_source=ORB en el tramo evaluado')

    gt_times = [number(row['receive_sec']) for row in gt_pose]
    gt_velocity_times = [number(row['receive_sec']) for row in gt_velocity]
    ref_times = [number(row['receive_sec']) for row in reference]
    paired = []
    first_pair = None
    for nav in orb_window:
        ref = nearest(reference, ref_times, number(nav['receive_sec']), 'receive_sec', 0.20)
        gt = nearest(gt_pose, gt_times, number(nav['receive_sec']), 'receive_sec', 0.10)
        gt_vel = nearest(gt_velocity, gt_velocity_times, number(nav['receive_sec']), 'receive_sec', 0.10)
        if ref is None or gt is None:
            continue
        if first_pair is None:
            first_pair = (nav, gt)
        paired.append((nav, ref, gt, gt_vel))
    if first_pair is None:
        raise RuntimeError('No hay pares ORB/GT/referencia dentro de los limites temporales')

    nav0, gt0 = first_pair
    yaw_offset = number(nav0['o_yaw']) - number(gt0['yaw'])
    cosine, sine = math.cos(yaw_offset), math.sin(yaw_offset)
    gt0_xy = (number(gt0['x']), number(gt0['y']))
    nav0_xy = (number(nav0['o_x']), number(nav0['o_y']))
    translation = (
        nav0_xy[0] - (cosine * gt0_xy[0] - sine * gt0_xy[1]),
        nav0_xy[1] - (sine * gt0_xy[0] + cosine * gt0_xy[1]),
        number(nav0['o_z']) - number(gt0['z']),
    )

    rows = []
    previous_kf = None
    for nav, ref, gt, gt_vel in paired:
        gt_x = cosine * number(gt['x']) - sine * number(gt['y']) + translation[0]
        gt_y = sine * number(gt['x']) + cosine * number(gt['y']) + translation[1]
        gt_z = number(gt['z']) + translation[2]
        orb = (number(nav['o_x']), number(nav['o_y']), number(nav['o_z']))
        ref_position = (number(ref['x_pos']), number(ref['y_pos']), number(ref['z_pos']))
        control_error = tuple(orb[index] - ref_position[index] for index in range(3))
        gt_error = (orb[0] - gt_x, orb[1] - gt_y, orb[2] - gt_z)
        keyframe = integer(nav['reference_keyframe_id']) if integer(nav['reference_keyframe_valid']) else -1
        keyframe_changed = previous_kf is not None and keyframe != previous_kf
        previous_kf = keyframe
        gt_vx_aligned = math.nan
        gt_vy_aligned = math.nan
        gt_vz_aligned = math.nan
        gt_omega_z = math.nan
        if gt_vel is not None:
            gt_vx_aligned = cosine * number(gt_vel['vel_x']) - sine * number(gt_vel['vel_y'])
            gt_vy_aligned = sine * number(gt_vel['vel_x']) + cosine * number(gt_vel['vel_y'])
            gt_vz_aligned = number(gt_vel['vel_z'])
            gt_omega_z = number(gt_vel['omega_z'])
        rows.append({
            'time_sec': number(nav['receive_sec']) - nav_start,
            'nav_stamp_sec': number(nav['stamp_sec']),
            'sample_sequence': integer(nav['sample_sequence']),
            'map_epoch': integer(nav['map_epoch']),
            'tracking_state': integer(nav['tracking_state']),
            'reference_keyframe_id': keyframe,
            'keyframe_changed': int(keyframe_changed),
            'navigation_age_sec': number(nav['receive_sec']) - number(nav['stamp_sec'])
            if abs(number(nav['receive_sec']) - number(nav['stamp_sec'])) < 1.0 else math.nan,
            'orb_x': orb[0], 'orb_y': orb[1], 'orb_z': orb[2], 'orb_yaw': number(nav['o_yaw']),
            'orb_vx': number(nav['vel_x']), 'orb_vy': number(nav['vel_y']), 'orb_vz': number(nav['vel_z']),
            'gt_x': gt_x, 'gt_y': gt_y, 'gt_z': gt_z, 'gt_yaw': number(gt['yaw']) + yaw_offset,
            'gt_vx': gt_vx_aligned, 'gt_vy': gt_vy_aligned, 'gt_vz': gt_vz_aligned, 'gt_omega_z': gt_omega_z,
            'ref_x': ref_position[0], 'ref_y': ref_position[1], 'ref_z': ref_position[2],
            'ref_yaw': number(ref['yaw_pos']),
            'ref_vx': number(ref['x_vel']), 'ref_vy': number(ref['y_vel']), 'ref_vz': number(ref['z_vel']),
            'ref_yaw_vel': number(ref['yaw_vel']),
            'ref_ax': number(ref['x_acc']), 'ref_ay': number(ref['y_acc']), 'ref_az': number(ref['z_acc']),
            'ref_yaw_acc': number(ref['yaw_acc']),
            'control_error_x': control_error[0], 'control_error_y': control_error[1],
            'control_error_z': control_error[2], 'control_error_norm': vector_norm(control_error),
            'control_yaw_error_rad': wrap(number(nav['o_yaw']) - number(ref['yaw_pos'])),
            'orb_gt_error_x': gt_error[0], 'orb_gt_error_y': gt_error[1],
            'orb_gt_error_z': gt_error[2], 'orb_gt_error_norm': vector_norm(gt_error),
            'orb_gt_yaw_error_rad': wrap(number(nav['o_yaw']) - (number(gt['yaw']) + yaw_offset)),
            'gt_vx_aligned': gt_vx_aligned,
            'velocity_error_norm': vector_norm((
                number(nav['vel_x']) - gt_vx_aligned,
                number(nav['vel_y']) - gt_vy_aligned,
                number(nav['vel_z']) - gt_vz_aligned,
            )) if gt_vel else math.nan,
        })
    write_csv(args.output_dir / 'samples.csv', rows)
    figures = save_figures(args.output_dir, rows)
    source_counts = {}
    for row in nav_window:
        source = integer(row['pose_source'])
        source_counts[str(source)] = source_counts.get(str(source), 0) + 1
    total_source_samples = sum(source_counts.values())
    tracking_bad = sum(integer(row['tracking_state']) not in (2, 5) for row in nav_window)
    fallback_count = source_counts.get('3', 0)
    keyframe_changes = sum(row['keyframe_changed'] for row in rows)
    epoch_changes = sum(
        rows[index]['map_epoch'] != rows[index - 1]['map_epoch']
        for index in range(1, len(rows)))
    summary = {
        'profile': args.profile,
        'reference_segment_samples': len(reference),
        'evaluation_samples': len(rows),
        'reference_interval_sec': [nav_start, nav_end],
        'alignment_gt_to_o': {
            'first_orb_sample_sequence': integer(nav0['sample_sequence']),
            'yaw_offset_rad': yaw_offset,
            'translation_m': translation,
        },
        'tracking': {
            'position_m': metric([row['control_error_norm'] for row in rows]),
            'position_x_m': metric([row['control_error_x'] for row in rows]),
            'position_y_m': metric([row['control_error_y'] for row in rows]),
            'position_z_m': metric([row['control_error_z'] for row in rows]),
            'velocity_mps': metric([row['velocity_error_norm'] for row in rows]),
            'yaw_rad': metric([row['control_yaw_error_rad'] for row in rows]),
        },
        'orb_vs_gt_aligned': {
            'position_m': metric([row['orb_gt_error_norm'] for row in rows]),
            'yaw_rad': metric([row['orb_gt_yaw_error_rad'] for row in rows]),
            'final_position_error_m': rows[-1]['orb_gt_error_norm'],
        },
        'visual_state': {
            'keyframe_changes': keyframe_changes,
            'tracking_not_ok_samples': tracking_bad,
            'orb_source_ratio': source_counts.get('1', 0) / total_source_samples if total_source_samples else None,
            'fallback_samples': fallback_count,
            'navigation_age_sec': metric([row['navigation_age_sec'] for row in rows]),
            'epoch_changes': epoch_changes,
            'source_counts': source_counts,
        },
        'final': {
            'reference_x': rows[-1]['ref_x'],
            'orb_x': rows[-1]['orb_x'],
            'control_position_error_m': rows[-1]['control_error_norm'],
        },
        'figures': figures,
        'success_candidate': bool(rows and fallback_count == 0 and source_counts.get('1', 0) == total_source_samples),
    }
    (args.output_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
