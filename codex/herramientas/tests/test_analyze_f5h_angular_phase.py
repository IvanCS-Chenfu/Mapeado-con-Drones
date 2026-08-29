import importlib.util
import math
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / 'analyze_f5h_angular_phase.py'
SPEC = importlib.util.spec_from_file_location('analyze_f5h_angular_phase', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_quaternion_matrix_uses_xyzw_convention():
    rotation = MODULE.quaternion_matrix([
        0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])
    assert np.allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-9)


def test_cross_correlation_reports_positive_signal_delay():
    time = np.arange(400, dtype=float) * 0.02
    reference = np.sin(2.0 * math.pi * 0.7 * time) + 0.2 * np.sin(2.0 * math.pi * 1.3 * time)
    delay_samples = 5
    signal = np.concatenate((np.zeros(delay_samples), reference[:-delay_samples]))
    result = MODULE.cross_correlation(time, reference, signal, max_lag_sec=0.3)
    assert math.isclose(result['lag_sec'], 0.1, abs_tol=0.021)
    assert result['correlation'] > 0.99


def test_log_parser_separates_drone_and_stream(tmp_path):
    log_path = tmp_path / 'phase.log'
    log_path.write_text(
        '[F5H-PHASE-MEASUREMENT] drone_id=2 input_stamp=1 receive_stamp=2\n'
        '[F5H-PHASE-PUBLISH] drone_id=2 publish_stamp=3 sample=4\n'
        '[F5H-PHASE-CONTROL] namespace=/dron_2 control_stamp=4 sample=4\n',
        encoding='utf-8')
    parsed = MODULE.parse_log(log_path)
    assert len(parsed[2]['measurement']) == 1
    assert len(parsed[2]['publish']) == 1
    assert len(parsed[2]['control']) == 1


def test_quaternion_distance_reports_relative_rotation():
    identity = [0.0, 0.0, 0.0, 1.0]
    yaw_90 = [0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]
    assert math.isclose(
        MODULE.quaternion_distance(identity, yaw_90), math.pi / 2.0,
        abs_tol=1e-9)


def test_power_summary_integrates_post_handoff_energy():
    rows = [
        {'time': 0.0, 'power': 100.0},
        {'time': 0.6, 'power': 1.0},
        {'time': 0.7, 'power': 1.0},
        {'time': 0.8, 'power': -1.0},
    ]
    summary = MODULE.power_summary(rows, 'power')
    assert math.isclose(summary['positive_work_ratio'], 2.0 / 3.0)
    assert math.isclose(summary['net_energy_j'], 0.1, abs_tol=1e-9)
    assert math.isclose(summary['duration_sec'], 0.2, abs_tol=1e-9)
    assert math.isclose(summary['net_energy_per_sec_jps'], 0.5, abs_tol=1e-9)


def test_boolean_parser_accepts_only_explicit_true():
    assert MODULE.as_bool({'value': 'true'}, 'value') == 1
    assert MODULE.as_bool({'value': 'false'}, 'value') == 0
    assert MODULE.as_bool({}, 'value') == 0


def test_unique_measurement_rows_deduplicates_control_reuse():
    rows = [
        {'measurement_receive_stamp': 1.0, 'sample': 1},
        {'measurement_receive_stamp': 1.0, 'sample': 2},
        {'measurement_receive_stamp': 1.1, 'sample': 3},
    ]
    unique = MODULE.unique_measurement_rows(rows)
    assert [row['sample'] for row in unique] == [1, 3]


def test_angular_error_summary_reports_magnitude_and_direction():
    reference = np.asarray([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    signal = np.asarray([[0.5, 0.0, 0.0], [0.0, -1.0, 0.0]])
    summary = MODULE.angular_error_summary(reference, signal)
    assert math.isclose(summary['mae_radps'], 1.75)
    assert math.isclose(summary['rmse_radps'], math.sqrt((0.25 + 9.0) / 2.0))
    assert math.isclose(summary['max_error_radps'], 3.0)
    assert math.isclose(summary['direction_mismatch_ratio'], 0.5)
    assert summary['direction_samples'] == 2
