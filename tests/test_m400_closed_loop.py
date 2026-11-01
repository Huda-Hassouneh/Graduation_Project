"""End-to-end M400 missions (Python physics, ESKF feedback): take-off, hover, 3-axis tracking, landing,
and detection/recovery of a mass change. Slow (~1-2 min); deselect with -m "not slow"."""

import numpy as np
import pytest

from graduation_project.experiments.mission_scenarios import load_scenario
from graduation_project.experiments.mission_runner import run_mission
from graduation_project.framework.core import FrameworkConfig
from graduation_project.metrics.mission import mission_metrics
from graduation_project.planning.mission import PHASES

pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def nominal_run():
    return run_mission(load_scenario("nominal", seed=11), "pid", FrameworkConfig())


def test_takeoff_hover_track_and_land(nominal_run) -> None:
    r = nominal_run
    assert r.status == "completed", r.info
    z, phase = r.col("p_z"), r.col("phase").astype(int)
    z_rest = r.info["z_rest"]
    assert z[0] == pytest.approx(z_rest, abs=0.01)  # starts resting on the pad
    to = phase == PHASES.index("takeoff")
    assert z[to].max() > z_rest + 2.0  # took off
    cov = phase == PHASES.index("coverage")
    for axis in ("x", "z"):  # moved along the facade and vertically between passes
        assert np.ptp(r.col(f"p_{axis}")[cov]) > 5.0
    assert np.ptp(r.col("p_y")) > 4.5  # pad (y = 0) to the stand-off line (y = 5)
    assert np.all(np.abs(np.degrees(r.col("yaw"))[cov] - 90.0) < 3.0)  # rotated to face the facade and held it
    assert r.info["landed_at"] is not None and r.info["max_ground_impact_mps"] < 1.0  # soft landing
    assert abs(r.col("p_x")[-1]) < 0.3 and abs(r.col("p_y")[-1]) < 0.3  # back on the pad
    m = mission_metrics(r)
    assert m["pos_rmse_m"] < 0.12 and m["est_err_rmse_m"] < 0.05
    assert m["min_clearance_m"] > 0.5 and m["safety_violation_any"] == 0.0
    assert m["coverage_fraction"] == pytest.approx(1.0, abs=0.01)
    assert m["false_alarm"] == 0.0 and r.info["n_probes"] == 0  # nominal flight: no diagnosis, no probes


def test_hover_accuracy_is_consistent_with_published_rtk_figure(nominal_run) -> None:
    """DJI publishes +-0.1 m RTK hover accuracy (closed loop, real controller). Same order of magnitude only."""
    r = nominal_run
    hold = (r.col("phase").astype(int) == PHASES.index("takeoff")) & (np.abs(r.col("ref_v_z")) < 1e-6)
    if np.any(hold):
        assert np.max(r.col("pos_err")[hold]) < 0.2


def test_mass_change_detected_classified_and_model_updated() -> None:
    scn = load_scenario("mass_change", seed=12)
    r = run_mission(scn, "pid", FrameworkConfig())
    m = mission_metrics(r)
    assert r.status == "completed"
    assert m["detected"] == 1.0 and m["detection_delay_s"] < 5.0
    assert m["classified_correct"] == 1.0
    ups = [e for e in r.events if e["event"] == "update_model" and "thrust gain" in e["detail"]]
    expected = 1.0 / (1.0 + (scn.truth_events[0].value - 1.0))
    assert ups and ups[-1]["value"] == pytest.approx(expected, abs=0.03)  # theta = m_nom / m_true
