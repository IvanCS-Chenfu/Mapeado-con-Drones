import csv
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "analyze_orb_visual_evidence.py"
SPEC = importlib.util.spec_from_file_location("orb_visual", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_quantile_interpolates_and_clamps():
    assert MODULE.quantile([3.0, 1.0], 0.5) == 2.0
    assert MODULE.quantile([3.0, 1.0], -1.0) == 1.0
    assert MODULE.quantile([3.0, 1.0], 2.0) == 3.0


def test_summarize_preserves_tracking_and_empirical_statistics(tmp_path):
    path = tmp_path / "drone_1_orb_visual_evidence.csv"
    fieldnames = ["image_stamp_sec", "tracking_state", *MODULE.FIELDS]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for stamp, tracking, inliers in ((1.0, 1, 0), (2.0, 2, 20), (3.0, 2, 40)):
            row = {field: 1.0 for field in MODULE.FIELDS}
            row.update({
                "image_stamp_sec": stamp,
                "tracking_state": tracking,
                "n_tracking_inliers": inliers,
            })
            writer.writerow(row)
    result = MODULE.summarize(path)
    assert result["frames"] == 3
    assert result["duration_sec"] == 2.0
    assert result["tracking_counts"] == {"1": 1, "2": 2}
    assert result["metrics"]["n_tracking_inliers"]["median"] == 20.0
    assert result["tracking_ok_frames"] == 2
    assert result["tracking_ok_metrics"]["n_tracking_inliers"]["median"] == 30.0
