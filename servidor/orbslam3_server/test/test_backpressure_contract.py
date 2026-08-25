from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'src/global_map_server.cpp'
).read_text(encoding='utf-8')


def test_hard_failures_are_observable_but_do_not_latch_backpressure():
    assert 'secondary_blocking_failure_' not in SOURCE
    assert 'blocking_failure=' not in SOURCE
    assert '[F3L-HARD-FAILURE]' in SOURCE
    assert 'reason=%s action=continue' in SOURCE


def test_backpressure_only_uses_queues_and_active_optimization():
    expected = (
        'backpressure_->Active() || secondary_pressure_active_ ||\n'
        '        optimization_active_.load()'
    )
    assert expected in SOURCE
