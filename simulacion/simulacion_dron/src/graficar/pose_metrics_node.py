#!/usr/bin/env python3

import csv
import json
import math
import os
from collections import deque
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib-f5f')
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from orbslam3_msgs.msg import NavigationState  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_matrix(pose):
    x, y, z, w = (
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return transform


def rotation_error_rad(first, second):
    relative = first[:3, :3].T @ second[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def yaw(transform):
    return math.atan2(transform[1, 0], transform[0, 0])


def wrapped_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def metric_summary(values):
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if finite.size == 0:
        return {'count': 0, 'rmse': None, 'mae': None, 'p95': None, 'max': None}
    return {
        'count': int(finite.size),
        'rmse': float(np.sqrt(np.mean(finite * finite))),
        'mae': float(np.mean(np.abs(finite))),
        'p95': float(np.percentile(np.abs(finite), 95)),
        'max': float(np.max(np.abs(finite))),
    }


def temporal_summary(interarrival_values):
    finite = np.asarray([
        value for value in interarrival_values
        if math.isfinite(value) and value > 0.0
    ], dtype=float)
    if finite.size == 0:
        return {'frequency_hz': None, 'jitter_sec': metric_summary([])}
    median = float(np.median(finite))
    return {
        'frequency_hz': float(1.0 / np.mean(finite)),
        'jitter_sec': metric_summary((finite - median).tolist()),
    }


class DroneMetrics:
    def __init__(self):
        self.gt = deque(maxlen=400)
        self.gt_velocity = deque(maxlen=400)
        self.rows = []
        self.alignments = {}
        self.last_stamp = None
        self.last_reference = None
        self.last_revision = None
        self.clock_offsets = {}
        self.reference_switches = 0
        self.revision_changes = 0
        self.unpaired = 0


class PoseMetricsNode(Node):
    def __init__(self):
        super().__init__('phase5_pose_metrics')
        self.declare_parameter('drone_count', 2)
        self.declare_parameter('namespace_base', 'dron')
        self.declare_parameter('output_dir', '/tmp/fase5_pose_metrics')
        self.declare_parameter('max_skew_sec', 0.10)
        self.declare_parameter('report_period_sec', 15.0)
        self.drone_count = int(self.get_parameter('drone_count').value)
        self.namespace_base = str(self.get_parameter('namespace_base').value)
        self.output_dir = Path(str(self.get_parameter('output_dir').value))
        self.max_skew = float(self.get_parameter('max_skew_sec').value)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.data = {drone: DroneMetrics() for drone in range(1, self.drone_count + 1)}
        self.input_subscriptions = []
        for drone in self.data:
            prefix = f'/{self.namespace_base}_{drone}'
            self.input_subscriptions.append(self.create_subscription(
                PoseStamped, f'{prefix}/sensor/GT/pose',
                lambda message, drone=drone: self.on_gt(drone, message), 40))
            self.input_subscriptions.append(self.create_subscription(
                TwistStamped, f'{prefix}/sensor/GT/vel',
                lambda message, drone=drone: self.on_gt_velocity(
                    drone, message), 40))
            self.input_subscriptions.append(self.create_subscription(
                NavigationState, f'{prefix}/orbslam/navigation_state',
                lambda message, drone=drone: self.on_navigation(drone, message), 40))
        period = max(2.0, float(self.get_parameter('report_period_sec').value))
        self.timer = self.create_timer(period, self.write_reports)
        self.get_logger().info(
            f'[F5F-METRICS-READY] drones={self.drone_count} output={self.output_dir} '
            f'max_skew_sec={self.max_skew:.3f}')

    def on_gt(self, drone, message):
        self.data[drone].gt.append((stamp_seconds(message.header.stamp), message.pose))

    def on_gt_velocity(self, drone, message):
        self.data[drone].gt_velocity.append((
            stamp_seconds(message.header.stamp), message.twist))

    def on_navigation(self, drone, message):
        state = self.data[drone]
        stamp = stamp_seconds(message.header.stamp)
        if not state.gt:
            state.unpaired += 1
            return
        gt_stamp, gt_pose = min(state.gt, key=lambda sample: abs(sample[0] - stamp))
        skew = abs(gt_stamp - stamp)
        if skew > self.max_skew:
            state.unpaired += 1
            return
        gt = pose_matrix(gt_pose)
        o_pose = pose_matrix(message.o_t_body)
        w_pose = pose_matrix(message.w_t_body)
        epoch = int(message.map_epoch)
        if message.local_valid and epoch not in state.alignments:
            state.alignments[epoch] = gt @ np.linalg.inv(o_pose)
        aligned_o = state.alignments.get(epoch, np.eye(4)) @ o_pose
        o_valid = bool(message.local_valid and epoch in state.alignments)
        w_observable = message.global_status != NavigationState.GLOBAL_STATUS_INVALID
        o_position_error = float(np.linalg.norm(aligned_o[:3, 3] - gt[:3, 3])) \
            if o_valid else math.nan
        w_position_error = float(np.linalg.norm(w_pose[:3, 3] - gt[:3, 3])) \
            if message.global_valid else math.nan
        o_angle_error = rotation_error_rad(aligned_o, gt) if o_valid else math.nan
        w_angle_error = rotation_error_rad(w_pose, gt) if message.global_valid else math.nan
        interarrival = math.nan if state.last_stamp is None else stamp - state.last_stamp
        raw_latency = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if epoch not in state.clock_offsets:
            state.clock_offsets[epoch] = raw_latency
        latency = raw_latency - state.clock_offsets[epoch]
        reference = int(message.reference_keyframe_id) \
            if message.reference_keyframe_valid else -1
        revision = int(message.pose_revision)
        linear_velocity_error = math.nan
        angular_velocity_error = math.nan
        if message.velocity_valid and state.gt_velocity and o_valid:
            velocity_stamp, gt_velocity = min(
                state.gt_velocity, key=lambda sample: abs(sample[0] - stamp))
            if abs(velocity_stamp - stamp) <= self.max_skew:
                alignment_rotation = state.alignments[epoch][:3, :3]
                estimated_linear_world = alignment_rotation @ np.array([
                    message.velocity.linear.x, message.velocity.linear.y,
                    message.velocity.linear.z])
                estimated_angular_world = alignment_rotation @ np.array([
                    message.velocity.angular.x, message.velocity.angular.y,
                    message.velocity.angular.z])
                gt_linear = np.array([
                    gt_velocity.linear.x, gt_velocity.linear.y,
                    gt_velocity.linear.z])
                gt_angular = np.array([
                    gt_velocity.angular.x, gt_velocity.angular.y,
                    gt_velocity.angular.z])
                linear_velocity_error = float(np.linalg.norm(
                    estimated_linear_world - gt_linear))
                angular_velocity_error = float(np.linalg.norm(
                    estimated_angular_world - gt_angular))
        if state.last_reference is not None and reference != state.last_reference:
            state.reference_switches += 1
        if state.last_revision is not None and revision != state.last_revision:
            state.revision_changes += 1
        state.last_reference = reference
        state.last_revision = revision
        state.last_stamp = stamp
        state.rows.append({
            'stamp': stamp, 'gt_stamp': gt_stamp, 'skew_sec': skew,
            'latency_sec': latency, 'interarrival_sec': interarrival,
            'epoch': epoch, 'tracking_state': int(message.tracking_state),
            'pose_source': int(message.pose_source),
            'global_status': int(message.global_status),
            'local_valid': int(message.local_valid),
            'global_valid': int(message.global_valid),
            'reference_keyframe_id': reference, 'pose_revision': revision,
            'o_x': aligned_o[0, 3] if o_valid else math.nan,
            'o_y': aligned_o[1, 3] if o_valid else math.nan,
            'o_z': aligned_o[2, 3] if o_valid else math.nan,
            'w_x': w_pose[0, 3] if w_observable else math.nan,
            'w_y': w_pose[1, 3] if w_observable else math.nan,
            'w_z': w_pose[2, 3] if w_observable else math.nan,
            'gt_x': gt[0, 3], 'gt_y': gt[1, 3], 'gt_z': gt[2, 3],
            'o_position_error_m': o_position_error,
            'w_position_error_m': w_position_error,
            'o_angle_error_rad': o_angle_error,
            'w_angle_error_rad': w_angle_error,
            'o_yaw_error_rad': wrapped_angle(yaw(aligned_o) - yaw(gt))
            if o_valid else math.nan,
            'w_yaw_error_rad': wrapped_angle(yaw(w_pose) - yaw(gt))
            if message.global_valid else math.nan,
            'linear_velocity_error_mps': linear_velocity_error,
            'angular_velocity_error_radps': angular_velocity_error,
        })
        if len(state.rows) % 100 == 0:
            self.get_logger().info(
                f'[F5F-POSE-METRIC] drone_id={drone} paired={len(state.rows)} '
                f'unpaired={state.unpaired} epoch={epoch} ref_kf={reference} '
                f'revision={revision} global_status={message.global_status} '
                f'skew_ms={skew * 1000.0:.3f}')

    def build_summary(self, drone, state):
        rows = state.rows
        orb_samples = sum(
            row['pose_source'] == NavigationState.POSE_SOURCE_ORB
            for row in rows)
        fallback_samples = sum(
            row['pose_source'] == NavigationState.POSE_SOURCE_GT_FALLBACK
            for row in rows)
        source_samples = orb_samples + fallback_samples
        interarrival_values = [row['interarrival_sec'] for row in rows]
        temporal = temporal_summary(interarrival_values)
        return {
            'drone_id': drone,
            'paired_samples': len(rows), 'unpaired_samples': state.unpaired,
            'epochs_aligned': sorted(state.alignments.keys()),
            'reference_switches': state.reference_switches,
            'revision_changes': state.revision_changes,
            'authoritative_samples': sum(row['global_valid'] for row in rows),
            'orb_samples': orb_samples,
            'gt_fallback_samples': fallback_samples,
            'gt_fallback_ratio': (
                fallback_samples / source_samples if source_samples else None),
            'linear_velocity_error_mps': metric_summary([
                row['linear_velocity_error_mps'] for row in rows]),
            'angular_velocity_error_radps': metric_summary([
                row['angular_velocity_error_radps'] for row in rows]),
            'o_position_m': metric_summary([row['o_position_error_m'] for row in rows]),
            'w_position_m': metric_summary([row['w_position_error_m'] for row in rows]),
            'o_angle_rad': metric_summary([row['o_angle_error_rad'] for row in rows]),
            'w_angle_rad': metric_summary([row['w_angle_error_rad'] for row in rows]),
            'o_yaw_rad': metric_summary([row['o_yaw_error_rad'] for row in rows]),
            'w_yaw_rad': metric_summary([row['w_yaw_error_rad'] for row in rows]),
            'frequency_hz': temporal['frequency_hz'],
            'interarrival_sec': metric_summary(interarrival_values),
            'jitter_sec': temporal['jitter_sec'],
            'latency_definition': 'relative_to_first_sample_per_epoch',
            'latency_sec': metric_summary([row['latency_sec'] for row in rows]),
            'max_skew_sec': max((row['skew_sec'] for row in rows), default=None),
        }

    def write_drone_plot(self, drone, rows):
        if not rows:
            return
        time = np.asarray([row['stamp'] - rows[0]['stamp'] for row in rows])
        figure, axes = plt.subplots(5, 1, figsize=(13, 15), sharex=True)
        for axis_index, coordinate in enumerate(('x', 'y', 'z')):
            axes[axis_index].plot(
                time, [row[f'gt_{coordinate}'] for row in rows],
                label='GT', color='black', linewidth=1.2)
            axes[axis_index].plot(
                time, [row[f'o_{coordinate}'] for row in rows],
                label='O aligned', color='#1976d2', linewidth=0.9)
            axes[axis_index].plot(
                time, [row[f'w_{coordinate}'] for row in rows],
                label='W', color='#d32f2f', linewidth=0.9)
            axes[axis_index].set_ylabel(f'{coordinate} [m]')
            axes[axis_index].grid(True, alpha=0.25)
        axes[0].legend(loc='upper right', ncol=3)
        axes[3].plot(
            time, [row['o_position_error_m'] for row in rows], label='O error')
        axes[3].plot(
            time, [row['w_position_error_m'] for row in rows], label='W error')
        axes[3].set_ylabel('error [m]')
        axes[3].legend(loc='upper right')
        axes[3].grid(True, alpha=0.25)
        axes[4].step(
            time, [row['pose_revision'] for row in rows], where='post',
            label='pose revision')
        axes[4].step(
            time, [row['global_status'] for row in rows], where='post',
            label='global status')
        axes[4].set_ylabel('state')
        axes[4].set_xlabel('time [s]')
        axes[4].legend(loc='upper right')
        axes[4].grid(True, alpha=0.25)
        figure.suptitle(f'Fase 5F - dron {drone}: O / W / GT')
        figure.tight_layout()
        figure.savefig(self.output_dir / f'drone_{drone}_o_w_gt.png', dpi=140)
        plt.close(figure)

    def write_reports(self):
        summaries = []
        for drone, state in self.data.items():
            if state.rows:
                csv_path = self.output_dir / f'drone_{drone}_samples.csv'
                with csv_path.open('w', newline='', encoding='utf-8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=state.rows[0].keys())
                    writer.writeheader()
                    writer.writerows(state.rows)
                self.write_drone_plot(drone, state.rows)
            summaries.append(self.build_summary(drone, state))
        report = {'drones': summaries}
        temporary = self.output_dir / 'summary.json.tmp'
        temporary.write_text(
            json.dumps(report, indent=2, sort_keys=True), encoding='utf-8')
        temporary.replace(self.output_dir / 'summary.json')
        counts = ','.join(
            f'{item["drone_id"]}:{item["paired_samples"]}/'
            f'{item["authoritative_samples"]}/'
            f'{item["orb_samples"]}/{item["gt_fallback_samples"]}'
            for item in summaries)
        if rclpy.ok():
            self.get_logger().info(
                '[F5H-REPORT] paired/authoritative/orb/fallback='
                f'{counts}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseMetricsNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.write_reports()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
