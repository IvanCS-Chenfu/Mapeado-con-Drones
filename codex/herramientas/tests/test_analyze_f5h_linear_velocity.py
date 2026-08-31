import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / 'analyze_f5h_linear_velocity.py'
SPEC = importlib.util.spec_from_file_location('linear_analyzer', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_interpolate_uses_requested_timestamp_domain():
    rows = [
        {'physical': 1.0, 'receive': 10.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
        {'physical': 3.0, 'receive': 14.0, 'x': 2.0, 'y': 4.0, 'z': 6.0},
    ]
    physical = MODULE.interpolate(rows, 'physical', ['x', 'y', 'z'], 2.0)
    receive = MODULE.interpolate(rows, 'receive', ['x', 'y', 'z'], 12.0)
    assert np.allclose(physical, [1.0, 2.0, 3.0])
    assert np.allclose(receive, [1.0, 2.0, 3.0])


def test_parse_marker_preserves_parallel_two_and_three_sample_values():
    row = MODULE.parse_marker(
        '[F5H-LINEAR-MEASUREMENT] mode=THREE_SAMPLE_PREDICTED '
        'v_mid_current=(1.0,2.0,3.0) v_hat_tk=(4.0,5.0,6.0)')
    assert np.allclose(row['v_mid_current'], [1.0, 2.0, 3.0])
    assert np.allclose(row['v_hat_tk'], [4.0, 5.0, 6.0])


def test_metrics_reports_bias_and_rmse_per_axis():
    result = MODULE.metrics([np.array([1.0, 0.0, -1.0]), np.array([1.0, 0.0, 1.0])])
    assert result['count'] == 2
    assert np.allclose(result['bias_xyz'], [1.0, 0.0, 0.0])
    assert np.allclose(result['rmse_xyz'], [1.0, 0.0, 1.0])


def test_reference_window_classifies_change_neighborhood():
    changes = [10.0]
    assert MODULE.reference_window(9.8, changes) == 'pre_500ms'
    assert MODULE.reference_window(10.05, changes) == 'immediate_100ms'
    assert MODULE.reference_window(10.3, changes) == 'post_500ms'
    assert MODULE.reference_window(11.0, changes) == 'stable'


def test_parse_dynamic_translation_keeps_acceleration_components():
    row = MODULE.parse_marker(
        '[F5H-DYNAMIC-TRANSLATION] g_O=(0,-9.81,0) '
        'a_thrust_O=(0,9.80,0) a_O=(0,-0.01,0)')
    assert np.allclose(row['g_O'], [0.0, -9.81, 0.0])
    assert np.allclose(row['a_O'], [0.0, -0.01, 0.0])


def test_parse_midpoint_dynamic_keeps_shadow_velocity_and_coverage():
    row = MODULE.parse_marker(
        '[F5H-MIDPOINT-DYNAMIC] valid=true torque_coverage=FULL '
        'thrust_coverage=FULL v_midpoint_dynamic_tk=(1.0,2.0,3.0)')
    assert row['valid'] == 'true'
    assert row['torque_coverage'] == 'FULL'
    assert np.allclose(row['v_midpoint_dynamic_tk'], [1.0, 2.0, 3.0])


def test_analyzer_defaults_to_drone_one_namespace():
    assert MODULE.analyze.__defaults__ == (1,)


def test_temporal_error_summary_uses_handoff_and_stops_at_fallback():
    rows = [
        {'stamp': 10.0, 'error': np.array([0.01, 0.0, 0.0])},
        {'stamp': 12.0, 'error': np.array([0.2, 0.0, 0.0])},
        {'stamp': 16.0, 'error': np.array([2.0, 0.0, 0.0])},
    ]
    result = MODULE.temporal_error_summary(rows, 10.0, 15.0)
    assert result['fallback_sec'] == 5.0
    assert result['first_error_gt_0.1_sec'] == 2.0
    assert result['first_error_gt_1_sec'] == 6.0
    assert result['windows']['0-5']['count'] == 2
