"""Sensors, ESKF, online identification: noise statistics, fault injection, consistency, identifiability."""

import numpy as np
import pytest

from graduation_project.aircraft_spec import load_spec
from graduation_project.config import load_aircraft
from graduation_project.controllers.pid import CascadedPIDController, load_pid_gains
from graduation_project.dynamics.multirotor import MultirotorPlant
from graduation_project.estimation.eskf import ErrorStateEKF
from graduation_project.planning.trajectories import Hover
from graduation_project.sensors.models import SensorFault, SensorParams, SensorSuite, TruthSample
from graduation_project.sysid.online import DragWindGrid, LagPosterior
from graduation_project.sysid.rls import RecursiveLeastSquares


@pytest.fixture(scope="module")
def setup():
    spec = load_spec("dji_matrice_400")
    return spec, load_aircraft("dji_matrice_400"), SensorParams.from_spec(spec)


def _truth(t, p, v=np.zeros(3), q=np.array([1.0, 0, 0, 0])):
    return TruthSample(t, np.asarray(p, float), np.asarray(v, float), q)


def test_gnss_noise_latency_and_faults(setup) -> None:
    _, _, sp = setup
    faults = (SensorFault("gnss_pos", "dropout", 2.0, 3.0), SensorFault("gnss_pos", "bias", 4.0, 5.0, [1.0, 0, 0]),
              SensorFault("gnss_pos", "invalid", 6.0, 7.0, 5.0, 1.0))
    s = SensorSuite(sp, seed=1, faults=faults)
    got = []
    for k in range(int(8.0 / 0.004)):
        t = k * 0.004
        for m in s.poll(t, _truth(t, [1.0, 2.0, 3.0])):
            if m.sensor == "gnss_pos":
                got.append((t, m.t, m.value))
    arr = np.array([g[2] for g in got if g[1] < 2.0])
    np.testing.assert_allclose(arr.std(axis=0), sp.gnss_pos_std, rtol=0.35)
    assert all(abs((ta - tm) - sp.gnss_latency) < 0.005 for ta, tm, _ in got)  # arrival = validity + latency
    assert not any(2.0 <= tm < 3.0 for _, tm, _ in got)  # dropout
    bias = np.array([v for _, tm, v in got if 4.0 <= tm < 5.0])
    assert bias[:, 0].mean() == pytest.approx(2.0, abs=0.05)
    inv = [v for _, tm, v in got if 6.0 <= tm < 7.0]
    assert any(np.isnan(v).all() for v in inv) and any(np.all(np.isfinite(v)) and abs(v[0] - 1) > 3 for v in inv)


def test_unreported_delay_returns_old_content(setup) -> None:
    _, _, sp = setup
    s = SensorSuite(sp, seed=2, faults=(SensorFault("gnss_pos", "delay", 0.0, 1e9, 0.5),))
    vals = []
    for k in range(int(4.0 / 0.004)):
        t = k * 0.004
        for m in s.poll(t, _truth(t, [t, 0, 0], [1, 0, 0])):  # x = t
            if m.sensor == "gnss_pos" and m.t > 1.0:
                vals.append(m.value[0] - m.t)
    assert np.mean(vals) == pytest.approx(-0.5, abs=0.02)


def test_eskf_hover_closed_loop_is_consistent(setup) -> None:
    spec, ac, sp = setup
    pl = MultirotorPlant(ac)
    pl.state = pl.hover_state(np.array([0, 0, 5.0]))
    sens = SensorSuite(sp, seed=4)
    ekf = ErrorStateEKF(sp, ac.gravity, p=np.array([0, 0, 5.0]))
    ctl = CascadedPIDController(ac, load_pid_gains("dji_matrice_400"))
    err = []
    for k in range(int(20 / 0.004)):
        t = k * 0.004
        s = pl.state
        imu = sens.imu(t, pl.specific_force_body(s.to_vector(), np.zeros(3)), s.angular_velocity)
        if k:
            ekf.predict(imu, 0.004)
        for m in sens.poll(t, _truth(t, s.position.copy(), s.velocity.copy(), s.quaternion.copy())):
            ekf.update(m, t)
        pl.step(0.004, ctl.update(ekf.estimate(), Hover(np.array([0, 0, 5.0])).sample(t)).omega_cmd)
        if t > 5:
            err.append(ekf.p - s.position)
    err = np.array(err)
    assert np.sqrt(np.mean(np.sum(err**2, axis=1))) < 0.05
    for name in ("gnss_pos", "gnss_vel", "heading", "baro"):
        st = ekf.stats[name]
        assert st.rejected_gate <= 0.01 * max(st.accepted, 1), name
        assert 0.1 < st.nis_ema < 3.0, name  # chi-square consistency (EMA of NIS / dof)
    assert np.all(ekf.diagnostics().position_std < 0.05)


def test_rls_converges_and_reports_identifiability() -> None:
    rls = RecursiveLeastSquares([0.0, 0.0], [10.0, 10.0], noise_std=0.1, forgetting=0.999)
    rng = np.random.default_rng(0)
    for _ in range(2000):
        phi = rng.standard_normal(2)
        rls.update(phi, phi @ [2.0, -1.0] + 0.1 * rng.standard_normal())
    np.testing.assert_allclose(rls.theta, [2.0, -1.0], atol=0.02)
    assert np.all(rls.std < 0.01)
    ig_exciting = rls.information_gain(rng.standard_normal((50, 2)))
    assert ig_exciting > rls.information_gain(np.zeros((50, 2))) == pytest.approx(0.0, abs=1e-9)
    flat = RecursiveLeastSquares([0.0], [1.0], noise_std=0.1, forgetting=0.99, p_max=np.array([1.0]))
    for _ in range(2000):
        flat.update(np.array([0.0]), 0.0)  # no excitation: uncertainty returns to its bound
    assert flat.std[0] == pytest.approx(1.0, rel=1e-6)


def _lag_data(tau_true, gain, excite, rng, n=1500, dt=0.02):
    lp = LagPosterior((0.03, 0.06, 0.1, 0.16, 0.25, 0.4), dt, 0.06, 6.0, 0.06)
    y = 9.8
    for k in range(n):
        u = 9.8 + (2.0 * np.sin(2 * np.pi * 0.5 * k * dt) if excite else 0.0)
        y += (1 - np.exp(-dt / tau_true)) * (u - y)
        lp.update(u, gain * y + 0.06 * rng.standard_normal(), gain)
    return lp


def test_lag_identifiable_only_with_excitation_and_insensitive_to_gain() -> None:
    rng = np.random.default_rng(1)
    hover = _lag_data(0.16, 1.0, False, rng)
    assert not hover.valid()  # no excitation -> lag not identifiable
    ex = _lag_data(0.16, 1.0, True, rng)
    assert ex.valid() and ex.mean == pytest.approx(0.16, rel=0.25)
    heavy = _lag_data(0.06, 0.8, True, rng)  # 20 % gain change must not look like a lag change
    assert heavy.mean == pytest.approx(0.06, rel=0.35)


def test_drag_wind_separation_needs_speed_variation() -> None:
    rng = np.random.default_rng(0)

    def run(c, w, speeds):
        g = DragWindGrid(0.011, 0.06, 15.0, 0.02)
        for v in speeds:
            g.update(v, -c * (v - w) * abs(v - w) + 0.06 * rng.standard_normal())
        return g

    t = np.arange(0, 60, 0.02)
    varying = 1.5 * np.sin(2 * np.pi * t / 24)
    g = run(0.033, 3.5, varying)
    assert g.c == pytest.approx(0.033, rel=0.2) and g.wind == pytest.approx(3.5, abs=0.7)
    g_const = run(0.033, 3.5, np.zeros_like(t))
    assert not g_const.c_std < 0.35 * 0.011  # constant speed: drag and wind not separable
