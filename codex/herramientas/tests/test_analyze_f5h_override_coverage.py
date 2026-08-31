#!/usr/bin/env python3

from pathlib import Path
import importlib.util


MODULE_PATH = Path(__file__).parents[1] / "analyze_f5h_override_coverage.py"
SPEC = importlib.util.spec_from_file_location("override_coverage", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_denominator_and_miss_run():
    prefix = "[dron_2.navigation_state_mux]: [F5H-CHANNEL-OVERRIDE] "
    lines = [
        prefix + "navigation_source=GT_FALLBACK override_requested=true valid=false applied=false",
        prefix + "navigation_source=ORB override_requested=true valid=true applied=true "
        "gt_alignment_skew=0.010 gt_interpolated=false gt_causal_propagation=true",
        prefix + "navigation_source=ORB override_requested=true valid=false applied=false "
        "gt_alignment_skew=-1 gt_interpolated=false gt_causal_propagation=false",
        prefix + "navigation_source=ORB override_requested=true valid=true applied=true "
        "gt_alignment_skew=0.030 gt_interpolated=true gt_causal_propagation=false",
    ]
    summary = MODULE.summarize(lines, 2)
    assert summary["override_requested_count"] == 3
    assert summary["override_applied_count"] == 2
    assert summary["max_consecutive_override_misses"] == 1
    assert summary["gt_alignment_skew_max"] == 0.03


if __name__ == "__main__":
    test_denominator_and_miss_run()
    print("1/1 tests passed")
