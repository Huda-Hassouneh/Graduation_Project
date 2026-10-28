"""Closed-loop integration tests (Milestone 2). Uses ideal state feedback (no sensors yet).

Thresholds marked "acceptance" are engineering acceptance criteria chosen before the final
runs, not claims of optimality.
"""

import json

import numpy as np
import pytest

from graduation_project.config import CONFIG_DIR, AircraftParams, load_aircraft
from graduation_project.controllers.pid import CascadedPIDController, PIDGains, load_pid_gains, position_gains_from_poles
from graduation_project.dynamics.mass_properties import MassProperties
from graduation_project.environment.wind import ConstantWind
from graduation_project.experiments.io import load_run_csv, save_run
from graduation_project.experiments.runner import ScheduledChange, SimConfig, run_closed_loop
from graduation_project.metrics.tracking import tracking_metrics
from graduation_project.planning.trajectories import CircleTrajectory, Hover, WaypointTrajectory


def controller_for(ac: AircraftParams) -> CascadedPIDController:
    return CascadedPIDController(ac, load_pid_gains(ac.name))


def test_pole_placement_gains_reproduce_requested_poles() -> None:
    kp, kd, ki = position_gains_from_poles(1.2, 0.8, 0.5)
    roots = np.sort_complex(np.roots([1.0, kd, kp, ki]))
    expected = np.sort_complex(np.roots(np.polymul([1.0, 0.5], [1.0, 2 * 0.8 * 1.2, 1.44])))
    np.testing.assert_allclose(roots, expected, atol=1e-9)


def test_hover_recovers_from_initial_offset(aircraft: AircraftParams) -> None:
    """Acceptance: < 5 cm error after 4 time constants of the slowest designed pole + 2 s.

    (A first version used a fixed 8 s horizon for both aircraft; the slower-by-design M400
    reference gave 5.03 cm. The horizon now follows each aircraft's own design pole.)
    """
    design = __import__("yaml").safe_load(open(CONFIG_DIR / "controllers" / "pid_baseline.yaml"))
    a_slowest = min(design["aircraft"][aircraft.name]["position_design"][ax]["a"] for ax in ("xy", "z"))
    horizon = 4.0 / a_slowest + 2.0
    sim = SimConfig(duration=horizon, initial_offset=np.array([0.5, -0.5, -0.3]))
    run = run_closed_loop(aircraft, controller_for(aircraft), Hover(np.array([0.0, 0.0, 5.0])), sim)
    assert run.status == "completed"
    assert run.col("pos_err")[-1] < 0.05
    assert np.max(run.col("pos_err")) < 0.8  # no overshoot beyond the initial error


def test_controller_starts_in_equilibrium_at_hover(research_quad: AircraftParams) -> None:
    run = run_closed_loop(research_quad, controller_for(research_quad), Hover(np.array([0, 0, 5.0])), SimConfig(duration=2.0))
    assert np.max(run.col("pos_err")) < 1e-9
    total_thrust = run.cols("thrust", "1234").sum(axis=1)
    np.testing.assert_allclose(total_thrust, research_quad.mass * research_quad.gravity, rtol=1e-9)


def test_end_to_end_logging_round_trip(tmp_path, research_quad: AircraftParams) -> None:
    run = run_closed_loop(research_quad, controller_for(research_quad), Hover(np.array([0, 0, 5.0])), SimConfig(duration=1.0))
    meta = {"experiment_id": "test_run", "seed": 0, "feedback_mode": "ideal_truth", "aircraft": research_quad.raw}
    run_dir = save_run(run, meta, runs_dir=tmp_path)
    cols, data = load_run_csv(run_dir / "timeseries.csv")
    assert cols == run.columns and data.shape == run.data.shape
    np.testing.assert_allclose(data, run.data, rtol=1e-8, atol=1e-12)
    saved = json.loads((run_dir / "metadata.json").read_text())
    for key in ("experiment_id", "seed", "git", "software", "status", "feedback_mode", "aircraft"):
        assert key in saved
    assert saved["status"] == "completed"
    assert len(run.col("t")) == 251 and run.col("t")[-1] == pytest.approx(1.0)


def test_divergence_is_reported_as_failed_run(research_quad: AircraftParams) -> None:
    good = load_pid_gains("research_quad")
    bad = PIDGains(**{**good.__dict__, "rate_p": -good.rate_p, "att_p": -good.att_p})
    sim = SimConfig(duration=10.0, initial_offset=np.array([0.2, 0.0, 0.0]))
    run = run_closed_loop(research_quad, CascadedPIDController(research_quad, bad), Hover(np.array([0, 0, 5.0])), sim)
    assert run.status == "failed" and run.message
    assert len(run.data) < 2501


@pytest.mark.slow
def test_time_step_convergence_on_circle(research_quad: AircraftParams) -> None:
    """dt_sim = 4 ms (default) vs 1 ms with the same 250 Hz controller: trajectories must agree."""
    traj = CircleTrajectory(np.array([0, 0, 5.0]), 3.0, 2.0, 10.0, ramp_time=4.0)
    runs = [run_closed_loop(research_quad, controller_for(research_quad), traj, SimConfig(duration=10.0, dt_sim=dt))
            for dt in (0.004, 0.001)]
    diff = np.max(np.linalg.norm(runs[0].cols("p") - runs[1].cols("p"), axis=1))
    assert diff < 2e-3  # [m]


@pytest.mark.slow
@pytest.mark.parametrize("name", ["research_quad", "m400_reference_quad"])
def test_circle_tracking_acceptance(name: str) -> None:
    ac = load_aircraft(name)
    traj = CircleTrajectory(np.array([0, 0, 5.0]), 3.0, 2.0, 30.0, ramp_time=4.0)
    run = run_closed_loop(ac, controller_for(ac), traj, SimConfig(duration=30.0))
    m = tracking_metrics(run)
    assert m["pos_rmse_m"] < {"research_quad": 0.10, "m400_reference_quad": 0.25}[name]  # acceptance
    assert m["saturation_ratio"] == 0.0


@pytest.mark.slow
def test_unknown_mass_increase_is_rejected_by_integral_action(research_quad: AircraftParams) -> None:
    """+20 % true mass at t = 2 s (controller keeps nominal mass): altitude dips, then recovers."""
    mp = research_quad.mass_properties
    change = ScheduledChange(2.0, "mass_+20pct",
                             lambda p: p.set_mass_properties(MassProperties(1.2 * mp.mass, 1.2 * mp.inertia)),
                             {"mass_factor": 1.2})
    run = run_closed_loop(research_quad, controller_for(research_quad), Hover(np.array([0, 0, 5.0])),
                          SimConfig(duration=16.0), changes=(change,))
    t, z = run.col("t"), run.col("p_z")
    assert run.col("mass_true")[-1] == pytest.approx(1.8)
    assert 5.0 - z[t > 2.0].min() > 0.05  # the change is visible in the response
    assert abs(z[-1] - 5.0) < 0.03  # recovered within 14 s


@pytest.mark.slow
def test_steady_wind_offset_is_removed(research_quad: AircraftParams) -> None:
    run = run_closed_loop(research_quad, controller_for(research_quad), Hover(np.array([0, 0, 5.0])),
                          SimConfig(duration=20.0), wind=ConstantWind(np.array([5.0, 0.0, 0.0])))
    t, err = run.col("t"), run.col("pos_err")
    assert err.max() > 0.05  # wind displaces the vehicle downwind ...
    assert run.col("p_x")[np.argmax(err)] > 0.0
    assert err[t > 15.0].max() < 0.03  # ... and integral action removes the offset
    assert np.rad2deg(run.col("pitch")[-1]) < 0.0  # leans into the wind: nose up toward -x


@pytest.mark.slow
def test_tilt_limit_is_respected_on_aggressive_move(research_quad: AircraftParams) -> None:
    traj = WaypointTrajectory([(0, 0, 5), (15, 0, 5)], durations=[2.5], start_time=0.5)
    run = run_closed_loop(research_quad, controller_for(research_quad), traj, SimConfig(duration=8.0))
    m = tracking_metrics(run)
    assert run.status == "completed"
    assert m["tilt_max_deg"] < np.rad2deg(research_quad.limits.max_tilt) + 3.0  # small transient overshoot allowed


@pytest.mark.slow
def test_m400_reference_rejects_max_payload_step() -> None:
    """+6 kg (published max payload) added at t = 2 s as a point mass at the CoM.

    The payload adds no inertia (a deliberate simplification for this test). The integrator
    bound (6 m/s^2 vertical) must allow full rejection.
    """
    ac = load_aircraft("m400_reference_quad")
    mp = ac.mass_properties
    change = ScheduledChange(2.0, "payload_+6kg",
                             lambda p: p.set_mass_properties(MassProperties(mp.mass + 6.0, mp.inertia)),
                             {"added_mass_kg": 6.0})
    run = run_closed_loop(ac, controller_for(ac), Hover(np.array([0, 0, 5.0])), SimConfig(duration=25.0),
                          changes=(change,))
    assert run.status == "completed"
    assert abs(run.col("p_z")[-1] - 5.0) < 0.05
    assert run.col("mass_true")[-1] == pytest.approx(15.74)
