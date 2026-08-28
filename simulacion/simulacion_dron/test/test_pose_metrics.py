import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / 'src' / 'graficar' / 'pose_metrics_node.py')
SPEC = importlib.util.spec_from_file_location('pose_metrics_node', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_pose(x, y, z, yaw_rad=0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(yaw_rad * 0.5),
            w=math.cos(yaw_rad * 0.5)))


def test_fixed_alignment_maps_local_pose_to_ground_truth():
    local = MODULE.pose_matrix(make_pose(2.0, 0.0, 0.0))
    ground_truth = MODULE.pose_matrix(make_pose(12.0, 5.0, 0.0))
    alignment = ground_truth @ MODULE.np.linalg.inv(local)
    next_local = MODULE.pose_matrix(make_pose(3.0, 0.0, 0.0))
    aligned = alignment @ next_local
    assert MODULE.np.allclose(aligned[:3, 3], [13.0, 5.0, 0.0])


def test_metric_summary_reports_required_statistics():
    summary = MODULE.metric_summary([1.0, 2.0, 3.0, math.nan])
    assert summary['count'] == 3
    assert math.isclose(summary['mae'], 2.0)
    assert math.isclose(summary['rmse'], math.sqrt(14.0 / 3.0))
    assert summary['p95'] >= 2.8
    assert summary['max'] == 3.0


def test_temporal_summary_reports_frequency_and_jitter():
    summary = MODULE.temporal_summary([math.nan, 0.05, 0.05, 0.06])
    assert math.isclose(summary['frequency_hz'], 18.75)
    assert summary['jitter_sec']['count'] == 3
    assert math.isclose(summary['jitter_sec']['max'], 0.01)


def test_rotation_error_and_wrapping_are_bounded():
    first = MODULE.pose_matrix(make_pose(0.0, 0.0, 0.0, math.radians(179.0)))
    second = MODULE.pose_matrix(make_pose(0.0, 0.0, 0.0, math.radians(-179.0)))
    assert math.isclose(
        MODULE.rotation_error_rad(first, second), math.radians(2.0), abs_tol=1e-7)
    assert math.isclose(
        MODULE.wrapped_angle(math.radians(358.0)), math.radians(-2.0), abs_tol=1e-7)


def test_node_constructs_without_shadowing_rclpy_properties(tmp_path):
    MODULE.rclpy.init()
    node = None
    try:
        node = MODULE.PoseMetricsNode()
        assert len(node.input_subscriptions) == 6
    finally:
        if node is not None:
            node.destroy_node()
        MODULE.rclpy.shutdown()


def test_main_tolerates_external_rclpy_shutdown(monkeypatch):
    events = []

    class FakeNode:
        def write_reports(self):
            events.append('report')

        def destroy_node(self):
            events.append('destroy')

    monkeypatch.setattr(MODULE.rclpy, 'init', lambda args=None: events.append('init'))
    monkeypatch.setattr(MODULE, 'PoseMetricsNode', FakeNode)
    monkeypatch.setattr(
        MODULE.rclpy, 'spin',
        lambda node: (_ for _ in ()).throw(MODULE.ExternalShutdownException()))
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(
        MODULE.rclpy, 'shutdown', lambda: events.append('shutdown'))

    MODULE.main()

    assert events == ['init', 'report', 'destroy']


def test_main_tolerates_invalid_context_wait_set_race(monkeypatch):
    events = []

    class FakeNode:
        def write_reports(self):
            events.append('report')

        def destroy_node(self):
            events.append('destroy')

    monkeypatch.setattr(MODULE.rclpy, 'init', lambda args=None: events.append('init'))
    monkeypatch.setattr(MODULE, 'PoseMetricsNode', FakeNode)
    monkeypatch.setattr(
        MODULE.rclpy, 'spin',
        lambda node: (_ for _ in ()).throw(RuntimeError('invalid context')))
    monkeypatch.setattr(MODULE.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(
        MODULE.rclpy, 'shutdown', lambda: events.append('shutdown'))

    MODULE.main()

    assert events == ['init', 'report', 'destroy']
