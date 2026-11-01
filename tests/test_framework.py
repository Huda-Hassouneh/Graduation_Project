"""Research framework units: mission planning, probes, energy manager, diagnosis, active diagnosis, recovery,
and the information boundary (the framework must not access simulation truth)."""

import ast
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from graduation_project.active_diagnosis.planner import ActiveConfig, ActiveDiagnosisPlanner
from graduation_project.active_diagnosis.probes import Probe
from graduation_project.aircraft_spec import load_spec
from graduation_project.config import REPO_ROOT, load_aircraft
from graduation_project.diagnosis.diagnoser import HYPOTHESES, UNDETERMINED_MSG, DiagnosisReport, Diagnoser
from graduation_project.energy.battery import BatteryParams
from graduation_project.energy.manager import EnergyManager
from graduation_project.estimation.eskf import EkfDiagnostics, SensorStats
from graduation_project.experiments.mission_runner import facade_plane
from graduation_project.planning.mission import (
    QUINTIC_A,
    QUINTIC_V,
    CoverageConfig,
    MissionManager,
    MotionLimits,
    build_coverage_mission,
    segment_duration,
)
from graduation_project.recovery.supervisor import RecoverySupervisor
from graduation_project.sysid.online import OnlineIdentifier, SysIdSnapshot

Z_REST = 0.3236


@pytest.fixture
def mission():
    f = facade_plane()
    return MissionManager(build_coverage_mission(f, CoverageConfig(), Z_REST), f, CoverageConfig(), MotionLimits(), Z_REST,
                          min_standoff=1.353)


def _max_speed_acc(m: MissionManager, t0=0.0, t1=None, dt=0.01):
    t1 = m.end_time if t1 is None else t1
    v, a, z = [], [], []
    for t in np.arange(t0, t1, dt):
        r = m.reference(t)
        v.append(r.velocity)
        a.append(r.acceleration)
        z.append(r.position[2])
    return np.array(v), np.array(a), np.array(z)


def test_coverage_plan_geometry_and_hard_limits(mission) -> None:
    f, lim = mission.facade, mission.limits
    cov = [s for s in mission.segments if s.coverage]
    assert len(cov) == 3
    for s in cov:  # passes at the 3 m stand-off, inside the facade extent, horizontal
        assert f.distance(s.p0) == pytest.approx(3.0) and f.distance(s.p1) == pytest.approx(3.0)
        assert s.p0[2] == pytest.approx(s.p1[2])
    assert mission.segments[0].phase == "ground" and mission.segments[-1].phase == "landing"
    assert mission.segments[-1].p1[2] < Z_REST  # touchdown target below resting height
    v, a, _ = _max_speed_acc(mission)
    assert np.max(np.linalg.norm(v[:, :2], axis=1)) <= lim.transit_speed + 1e-6
    assert np.max(v[:, 2]) <= lim.climb_speed + 1e-6 and np.min(v[:, 2]) >= -lim.descent_speed - 1e-6
    assert np.max(np.abs(a)) <= lim.max_accel + 1e-6
    assert segment_duration(np.zeros(3), np.array([12.0, 0, 0]), "cruise", lim) == pytest.approx(
        max(QUINTIC_V * 12 / lim.cruise_speed, np.sqrt(QUINTIC_A * 12 / lim.max_accel)))


def test_reference_is_continuous_through_replanning(mission) -> None:
    t_change = 30.0
    before = mission.reference(t_change)
    mission.set_speed_scale(t_change, 0.6)
    mission.shift_standoff(t_change + 0.02, 1.0)
    mission.abort(t_change + 0.04, "test")
    ts = np.arange(t_change - 1.0, mission.end_time, 0.004)
    p = np.array([mission.reference(t).position for t in ts])
    assert np.max(np.linalg.norm(np.diff(p, axis=0), axis=1)) < 0.02  # no jumps (<= 2 m/s * 4 ms + margin)
    assert np.linalg.norm(mission.reference(t_change).position - before.position) < 1e-9
    assert mission.segments[-1].p1[2] < Z_REST and mission.aborted


def test_replanned_waypoints_respect_minimum_standoff(mission) -> None:
    mission.shift_standoff(20.0, -2.5)  # an unsafe request toward the facade
    for s in mission.segments:
        if s.phase in ("coverage", "transit") and mission.facade.in_front(s.p1 - mission.facade.distance(s.p1) * mission.facade.normal):
            assert mission.facade.distance(s.p1) >= mission.min_standoff - 1e-9


def test_probe_is_bounded_and_smooth(mission) -> None:
    pr = Probe("vertical", 0.1, 0.5, 6.0, np.array([0, 0, 1.0]))
    taus = np.linspace(0, 6.0, 3001)
    off = np.array([pr.offset(t) for t in taus])
    assert np.max(np.abs(off[:, 0])) <= pr.amplitude + 1e-12
    assert np.max(np.abs(off[:, 1])) <= pr.peak_velocity + 1e-9 and np.max(np.abs(off[:, 2])) <= pr.peak_acceleration + 1e-9
    np.testing.assert_allclose(off[0], 0, atol=1e-12)
    np.testing.assert_allclose(off[-1], 0, atol=1e-9)
    end0 = mission.end_time
    t0 = mission.insert_probe(20.0, pr)
    assert mission.end_time == pytest.approx(end0 + 6.0) and t0 >= 20.0
    assert mission.probe_active(t0 + 1.0) and mission.tag(t0 + 1.0) == "diagnostic"


@pytest.fixture(scope="module")
def nominal_parts():
    spec = load_spec("dji_matrice_400")
    ac = load_aircraft("dji_matrice_400")
    return spec, ac, BatteryParams.from_spec(spec)


def test_energy_manager_is_conservative_and_buckets(nominal_parts) -> None:
    _, _, bp = nominal_parts
    em = EnergyManager(bp, hover_power_nominal=1050.0)
    em.update(0.0, np.array([52.0, 20.0, 0.80]), "nominal")
    for k in range(1, 101):
        em.update(k * 0.1, np.array([52.0, 20.0, 0.80]), "diagnostic" if k > 50 else "nominal")
    assert em.consumed_j == pytest.approx(1040.0 * 10.0, rel=1e-6)
    assert em.buckets["diagnostic"] == pytest.approx(1040.0 * 5.0, rel=1e-6)
    r_full = em.remaining_j()
    em.update(10.1, np.array([52.0, 20.0, 0.60]), "nominal")  # BMS reports less than counted: trust the minimum
    assert em.remaining_j() < r_full
    assert em.energy_for(100.0, thrust_gain=0.8) > em.energy_for(100.0, thrust_gain=1.0)
    assert em.margin_after(1e6) < 0.0


def _ekf_diag(t=10.0):
    stats = {n: SensorStats(accepted=10, last_arrival_t=t, last_accept_t=t) for n in ("gnss_pos", "gnss_vel", "baro", "heading", "range")}
    return EkfDiagnostics(t, np.full(3, 0.02), np.full(3, 0.02), np.full(3, 0.01), np.zeros(3), np.zeros(3), stats)


def _snap(**kw):
    base = dict(t=0.0, thrust_gain=1.0, thrust_gain_std=0.001, lag=0.06, lag_std=0.05, lag_valid=False,
                drag_coeff=np.array([0.011, 0.011]), drag_coeff_std=np.array([0.02, 0.02]), wind_accel=np.zeros(2),
                gnss_delay=0.0, gnss_delay_std=0.1, gnss_delay_r2=0.0)
    base.update(kw)
    return SysIdSnapshot(**base)


def _diagnoser(ac):
    return Diagnoser(ac.mass, ac.mass_properties.inertia, ac.rotor.tau_motor, 0.011)


def _run_diag(d, steps, snap, f_body=np.array([0, 0, 9.81]), wrench=np.array([95.5, 0, 0, 0]), t0=10.0, track=0.05):
    rep = None
    for k in range(steps):
        t = t0 + k * 0.02
        rep = d.update(t, np.array([1.0, 0, 0, 0]), f_body, np.zeros(3), wrench, track, snap, _ekf_diag(t), True)
    return rep


def test_nominal_signals_raise_no_alarm(nominal_parts) -> None:
    rep = _run_diag(_diagnoser(nominal_parts[1]), 500, _snap())
    assert not rep.anomaly and rep.cause is None and rep.message == "nominal"


def test_thrust_gain_change_is_diagnosed_without_torque(nominal_parts) -> None:
    rep = _run_diag(_diagnoser(nominal_parts[1]), 400, _snap(thrust_gain=0.85))
    assert rep.anomaly and rep.determined and rep.cause == "mass_or_thrust_change"
    assert "gain" in rep.persistent


def test_conflicting_evidence_is_reported_as_undetermined(nominal_parts) -> None:
    d = _diagnoser(nominal_parts[1])
    # thrust-gain change AND large tracking error AND GNSS inconsistency at once
    rep = None
    for k in range(400):
        t = 10.0 + k * 0.02
        diag = _ekf_diag(t)  # sensors keep arriving ...
        diag.stats["gnss_pos"].nis_ema = 6.0  # ... but GNSS innovations are inconsistent
        rep = d.update(t, np.array([1.0, 0, 0, 0]), np.array([0, 0, 9.81]), np.zeros(3), np.array([95.5, 0, 0, 0]),
                       0.05, _snap(thrust_gain=0.93), diag, True)
    assert rep.anomaly and not rep.determined and rep.cause is None and rep.message == UNDETERMINED_MSG
    assert set(rep.probabilities) == set(HYPOTHESES)


def test_sensor_dropout_detected_from_staleness(nominal_parts) -> None:
    d = _diagnoser(nominal_parts[1])
    rep = None
    for k in range(200):
        t = 10.0 + k * 0.02
        diag = _ekf_diag(t)
        for n in ("gnss_pos", "gnss_vel"):
            diag.stats[n].last_arrival_t = 10.0  # stops arriving
        rep = d.update(t, np.array([1.0, 0, 0, 0]), np.array([0, 0, 9.81]), np.zeros(3), np.array([95.5, 0, 0, 0]),
                       0.05, _snap(), diag, True)
    assert rep.determined and rep.cause == "sensor_dropout"


def _report(probs, anomaly=True):
    p = dict.fromkeys(HYPOTHESES, 0.0)
    p.update(probs)
    return DiagnosisReport(0.0, anomaly, 0.0, p, None, False, max(p.values()), {}, (), UNDETERMINED_MSG)


def test_active_diagnosis_respects_hard_constraints(nominal_parts) -> None:
    _, ac, bp = nominal_parts
    em = EnergyManager(bp, 1050.0)
    em.update(0.0, np.array([52.0, 20.0, 0.9]), "nominal")
    sysid = OnlineIdentifier(ac.mass, ac.rotor.tau_motor, 0.107)
    f = facade_plane()
    tmax = ac.rotor.thrust_max * 4
    planner = ActiveDiagnosisPlanner(ac.mass, ac.gravity, tmax, 0.853)
    rep = _report({"response_delay": 0.5, "unexplained": 0.3})
    ok = planner.evaluate(30.0, rep, sysid, em, np.array([6.0, 5.0, 6.0]), np.zeros(3), np.full(3, 0.02), "coverage", f, 25.0, 1.0)
    assert ok.accepted and ok.probe.kind == "vertical" and ok.chosen.ig >= planner.cfg.ig_min
    near = ActiveDiagnosisPlanner(ac.mass, ac.gravity, tmax, 0.853)  # 1.2 m from the wall: clearance 0.29 m < 0.5 m
    dec = near.evaluate(30.0, rep, sysid, em, np.array([6.0, 6.8, 6.0]), np.zeros(3), np.full(3, 0.02), "coverage", f, 25.0, 1.0)
    assert not dec.accepted and all("facade clearance" in c.reasons for c in dec.candidates)
    low = EnergyManager(bp, 1050.0)
    low.update(0.0, np.array([45.0, 20.0, 0.07]), "nominal")  # nearly empty: probe would eat the reserve
    dec = ActiveDiagnosisPlanner(ac.mass, ac.gravity, tmax, 0.853).evaluate(
        30.0, rep, sysid, low, np.array([6.0, 5.0, 6.0]), np.zeros(3), np.full(3, 0.02), "coverage", f, 25.0, 1.0)
    assert not dec.accepted and any("energy" in r for c in dec.candidates for r in c.reasons)
    dec = ActiveDiagnosisPlanner(ac.mass, ac.gravity, tmax, 0.853).evaluate(
        30.0, _report({}, anomaly=False), sysid, em, np.array([6.0, 5.0, 6.0]), np.zeros(3), np.full(3, 0.02), "coverage", f, 25.0, 1.0)
    assert not dec.accepted  # no hypothesis needs information: probe has no value


def test_energy_awareness_changes_probe_utility_only(nominal_parts) -> None:
    _, ac, bp = nominal_parts
    em = EnergyManager(bp, 1050.0)
    em.update(0.0, np.array([52.0, 20.0, 0.9]), "nominal")
    sysid = OnlineIdentifier(ac.mass, ac.rotor.tau_motor, 0.107)
    rep = _report({"response_delay": 0.6})
    args = (30.0, rep, sysid, em, np.array([6.0, 5.0, 6.0]), np.zeros(3), np.full(3, 0.02), "coverage", facade_plane(), 25.0, 1.0)
    aware = ActiveDiagnosisPlanner(ac.mass, ac.gravity, ac.rotor.thrust_max * 4, 0.853).evaluate(*args)
    blind = ActiveDiagnosisPlanner(ac.mass, ac.gravity, ac.rotor.thrust_max * 4, 0.853, energy_aware=False).evaluate(*args)
    a = {c.kind: c for c in aware.candidates}
    b = {c.kind: c for c in blind.candidates}
    assert a["vertical"].ig == pytest.approx(b["vertical"].ig) and a["vertical"].utility < b["vertical"].utility


def _supervisor(ac, **kw):
    return RecoverySupervisor(ac.rotor.thrust_max * 4, 30.0, ac.mass, ac.gravity, 0.853, **kw)


def _decide(sup, rep, em, pos=np.array([6.0, 5.0, 6.0]), std=np.full(3, 0.02), gain=1.0, t=50.0, remaining=60.0):
    return sup.decide(t, rep, gain, 0.001, np.zeros(3), np.zeros(2), em, pos, std, facade_plane(), remaining, 25.0, 40.0,
                      False, "coverage")


def test_recovery_safety_rules_override(nominal_parts) -> None:
    _, ac, bp = nominal_parts
    em = EnergyManager(bp, 1050.0)
    em.update(0.0, np.array([52.0, 20.0, 0.9]), "nominal")
    acts = _decide(_supervisor(ac), None, em, std=np.full(3, 1.5))
    assert [a.kind for a in acts] == ["safe_termination"]
    acts = _decide(_supervisor(ac), None, em, pos=np.array([6.0, 6.6, 6.0]))  # 1.4 m from the wall
    assert acts and acts[0].kind == "modify_trajectory"
    acts = _decide(_supervisor(ac, replanning=False), None, em, pos=np.array([6.0, 6.6, 6.0]))
    assert [a.kind for a in acts] == ["safe_termination"]


def test_recovery_energy_and_model_update(nominal_parts) -> None:
    _, ac, bp = nominal_parts
    low = EnergyManager(bp, 1050.0)
    low.update(0.0, np.array([45.0, 20.0, 0.20]), "nominal")  # enough for the exit, not for 400 s more
    acts = _decide(_supervisor(ac), None, low, remaining=400.0)
    assert acts and acts[0].kind == "replan_remaining"
    acts = _decide(_supervisor(ac, energy_aware=False), None, low, remaining=400.0)
    assert not acts or acts[0].kind != "replan_remaining"
    empty = EnergyManager(bp, 1050.0)
    empty.update(0.0, np.array([45.0, 20.0, 0.11]), "nominal")  # not even the exit is affordable
    assert [a.kind for a in _decide(_supervisor(ac), None, empty, remaining=400.0)] == ["safe_termination"]
    em = EnergyManager(bp, 1050.0)
    em.update(0.0, np.array([52.0, 20.0, 0.9]), "nominal")
    rep = replace(_report({"mass_or_thrust_change": 0.9}), cause="mass_or_thrust_change", determined=True)
    acts = _decide(_supervisor(ac), rep, em, gain=0.85)
    assert any(a.kind == "update_model" and a.value == pytest.approx(0.85) for a in acts)
    acts = _decide(_supervisor(ac, adaptation=False), rep, em, gain=0.85)
    assert not any(a.kind == "update_model" for a in acts)
    acts = _decide(_supervisor(ac), rep, em, gain=0.40)  # implied hover throttle too high
    assert [a.kind for a in acts][-1] == "abort"


FRAMEWORK_MODULES = ["framework/core.py", "diagnosis/diagnoser.py", "active_diagnosis/planner.py", "recovery/supervisor.py",
                     "sysid/online.py", "sysid/rls.py", "energy/manager.py"]
FORBIDDEN = {"graduation_project.dynamics.multirotor", "graduation_project.energy.battery.Battery",
             "graduation_project.experiments.mission_scenarios", "graduation_project.experiments.mission_runner",
             "graduation_project.sensors.models.SensorSuite", "graduation_project.environment.wind"}


@pytest.mark.parametrize("rel", FRAMEWORK_MODULES)
def test_framework_has_no_access_to_truth(rel) -> None:
    """Static check of the information boundary: the research framework never imports the plant, the
    simulated battery, the sensor simulator, the wind model or the scenario fault schedule."""
    tree = ast.parse((REPO_ROOT / "src" / "graduation_project" / rel).read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            for alias in node.names:
                full = f"{node.module}.{alias.name}"
                assert node.module not in FORBIDDEN and full not in FORBIDDEN, f"{rel} imports {full}"
        if isinstance(node, ast.Import):
            for alias in node.names:
                assert alias.name not in FORBIDDEN, f"{rel} imports {alias.name}"
    src = (REPO_ROOT / "src" / "graduation_project" / rel).read_text()
    assert ".truth" not in src and "fault_class" not in src and "truth_events" not in src
