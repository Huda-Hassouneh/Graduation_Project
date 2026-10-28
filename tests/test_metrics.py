"""Metric definitions checked against hand-computed values on synthetic logs."""

import numpy as np
import pytest

from graduation_project.experiments.runner import RunResult, log_columns
from graduation_project.metrics.tracking import tracking_metrics


def synthetic_run(n: int = 101) -> RunResult:
    cols = log_columns(4)
    data = np.zeros((n, len(cols)))
    c = {name: i for i, name in enumerate(cols)}
    t = np.linspace(0.0, 10.0, n)
    data[:, c["t"]] = t
    data[:, c["q_w"]] = 1.0
    data[:, c["p_x"]] = t  # 1 m/s along x -> 10 m path
    data[:, c["ref_p_x"]] = t
    data[:, c["ref_p_z"]] = np.where(t >= 5.0, 0.2, 0.0)  # 0.2 m vertical error in second half
    data[:, c["pos_err"]] = np.abs(data[:, c["ref_p_z"]])
    data[:, c["att_err"]] = np.deg2rad(2.0)
    data[:, c["energy"]] = 100.0 * t  # 100 W constant
    data[:, c["alloc_saturated"]] = (np.arange(n) % 10 == 0).astype(float)
    return RunResult(cols, data, "completed")


def test_metrics_full_window() -> None:
    m = tracking_metrics(synthetic_run())
    frac = np.mean(np.linspace(0, 10, 101) >= 5.0)
    assert m["pos_rmse_m"] == pytest.approx(0.2 * np.sqrt(frac))
    assert m["pos_max_err_m"] == pytest.approx(0.2)
    assert m["pos_rmse_xy_m"] == pytest.approx(0.0)
    assert m["att_err_mean_deg"] == pytest.approx(2.0)
    assert m["energy_J"] == pytest.approx(1000.0)
    assert m["mean_power_W"] == pytest.approx(100.0)
    assert m["distance_m"] == pytest.approx(10.0)
    assert m["energy_per_m_J"] == pytest.approx(100.0)
    assert m["saturation_ratio"] == pytest.approx(11 / 101)
    assert m["tilt_max_deg"] == pytest.approx(0.0)


def test_metrics_window_start() -> None:
    m = tracking_metrics(synthetic_run(), t_start=5.0)
    assert m["pos_rmse_m"] == pytest.approx(0.2)
    assert m["energy_J"] == pytest.approx(500.0)
    assert m["n_samples"] == 51


def test_metrics_empty_window_rejected() -> None:
    with pytest.raises(ValueError):
        tracking_metrics(synthetic_run(), t_start=11.0)
