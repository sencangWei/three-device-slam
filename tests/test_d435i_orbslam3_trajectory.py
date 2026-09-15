from pathlib import Path

import pytest

from scripts.analyze_d435i_orbslam3_trajectory import trajectory_metrics


def test_trajectory_metrics_cover_closure_and_static_tail(tmp_path: Path) -> None:
    path = tmp_path / "trajectory.txt"
    path.write_text(
        "10000000000 0 0 0 0 0 0 1\n"
        "95000000000 0.02 0 0 0 0 0 1\n"
        "100000000000 0.001 0 0 0 0 0 1\n"
    )
    metrics = trajectory_metrics(path, input_start_ns=0, input_end_ns=100000000000)
    assert metrics["initialization_s"] == pytest.approx(10.0)
    assert metrics["end_lag_s"] == pytest.approx(0.0)
    assert metrics["endpoint_translation_m"] == pytest.approx(0.001)
    assert metrics["final_static_max_radius_m"] == pytest.approx(0.019)


def test_trajectory_requires_final_static_coverage(tmp_path: Path) -> None:
    path = tmp_path / "trajectory.txt"
    path.write_text(
        "10000000000 0 0 0 0 0 0 1\n"
        "90000000000 0.1 0 0 0 0 0 1\n"
    )
    with pytest.raises(ValueError, match="final static"):
        trajectory_metrics(path, input_start_ns=0, input_end_ns=100000000000)
