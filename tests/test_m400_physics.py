"""Physics extensions: ground contact, arming, command delay, density scaling, battery, atmosphere, wind."""

import numpy as np
import pytest

from graduation_project.aircraft_spec import load_spec
from graduation_project.config import load_aircraft
from graduation_project.dynamics.multirotor import MultirotorPlant
from graduation_project.energy.battery import Battery, BatteryParams
from graduation_project.environment.atmosphere import RHO0, Site, isa, pressure_altitude
from graduation_project.environment.wind import CompositeWind, DiscreteGust, DrydenTurbulence, MeanWindProfile
from graduation_project.experiments.mission_runner import contact_params


@pytest.fixture(scope="module")
def m400():
    return load_spec("dji_matrice_400"), load_aircraft("dji_matrice_400")


def _ground_plant(m400):
    spec, ac = m400
    pl = MultirotorPlant(ac, contact=contact_params(spec, ac))
    pl.state = pl.ground_state(np.zeros(2), yaw=0.7)
    pl.armed = False
    return pl


def test_rests_on_ground_without_drift(m400) -> None:
    pl = _ground_plant(m400)
    p0 = pl.state.position.copy()
    for _ in range(1250):
        pl.step(0.004, np.zeros(4))
    np.testing.assert_allclose(pl.state.position, p0, atol=1e-6)
    assert np.linalg.norm(pl.state.velocity) < 1e-6 and np.linalg.norm(pl.state.angular_velocity) < 1e-6


def test_lift_off_and_touchdown_energy_dissipation(m400) -> None:
    pl = _ground_plant(m400)
    pl.armed = True
    w = np.full(4, pl.params.hover_speed * 1.1)
    for _ in range(500):
        pl.step(0.004, w)
    assert pl.state.position[2] > 1.0  # thrust above weight lifts off
    pl.armed = False  # motors stop: free fall then contact
    v_max = 0.0
    for _ in range(2500):
        pl.step(0.004, np.zeros(4))
        v_max = max(v_max, abs(pl.state.velocity[2]))
    assert abs(pl.state.velocity[2]) < 0.05 * v_max  # damped contact dissipates the impact energy
    assert pl.state.position[2] > 0.0  # never below ground


def test_disarmed_rotors_spin_down(m400) -> None:
    pl = MultirotorPlant(m400[1])
    pl.armed = False
    for _ in range(500):
        pl.step(0.004, np.full(4, 300.0))
    assert np.all(pl.state.rotor_speeds < 1e-6 * 300 + 1e-6)


def test_command_delay_is_integer_steps(m400) -> None:
    ac = m400[1]
    a, b = MultirotorPlant(ac), MultirotorPlant(ac)
    b.truth.command_delay_s = 0.04  # 10 steps
    cmd = np.full(4, ac.hover_speed * 1.2)
    for k in range(10):
        a.step(0.004, cmd)
        b.step(0.004, cmd if k == 0 else cmd)
    assert np.allclose(a.state.rotor_speeds, b.state.rotor_speeds)  # constant command: delay invisible
    c = MultirotorPlant(ac)
    c.truth.command_delay_s = 0.04
    hover = np.full(4, ac.hover_speed)
    for _ in range(11):
        c.step(0.004, hover)
    for _ in range(10):
        c.step(0.004, cmd)  # new command still in the delay line
    np.testing.assert_allclose(c.state.rotor_speeds, hover, rtol=1e-9)
    c.step(0.004, cmd)
    assert np.all(c.state.rotor_speeds > hover)


def test_density_scales_thrust(m400) -> None:
    ac = m400[1]
    pl = MultirotorPlant(ac)
    w0 = pl.wrench(pl.state.to_vector(), np.zeros(3)).thrusts.sum()
    pl.truth.air_density_ratio = 0.8
    assert pl.wrench(pl.state.to_vector(), np.zeros(3)).thrusts.sum() == pytest.approx(0.8 * w0)


def test_isa_reference_values() -> None:
    assert isa(0).density == pytest.approx(1.225, rel=1e-4)
    assert isa(1000).pressure == pytest.approx(89874.6, rel=1e-4)  # ISO 2533 table value
    assert isa(1000).temperature == pytest.approx(281.65)
    assert pressure_altitude(isa(1234.0).pressure) == pytest.approx(1234.0, abs=1e-6)
    assert Site(0.0, 0.0).density_ratio == pytest.approx(1.0, abs=1e-6)
    assert isa(0, 20).density < RHO0


def test_battery_consistency_with_published_values(m400) -> None:
    bp = BatteryParams.from_spec(m400[0])
    assert bp.ocv(1.0) == pytest.approx(54.6)
    assert bp.nominal_energy_wh == pytest.approx(977.0, rel=0.02)
    b = Battery(bp, 1.0)
    p = 1050.7
    i, limited = b.current_for_power(p)
    assert not limited and i * (bp.ocv(1.0) - i * bp.r0) == pytest.approx(p, rel=1e-9)
    e_out, t = 0.0, 0
    while not b.state.depleted:
        b.step(p, 1.0)
        t += 1
    assert 45 * 60 < t < 56 * 60  # consistency with the calibration target, not a validation
    assert b.state.energy_out_j == pytest.approx(p * t, rel=1e-3)


def test_battery_power_limit_and_aging(m400) -> None:
    bp = BatteryParams.from_spec(m400[0])
    b = Battery(bp, 0.5, capacity_scale=0.8, r0_scale=2.0)
    p_max = bp.ocv(0.5) ** 2 / (4 * 2.0 * bp.r0)
    _, limited = b.current_for_power(1.01 * p_max)
    assert limited
    b2 = Battery(bp, 0.5)
    for _ in range(60):
        b.step(1000.0, 1.0)
        b2.step(1000.0, 1.0)
    assert b.state.soc < b2.state.soc and b.state.voltage < b2.state.voltage


def test_wind_components() -> None:
    prof = MeanWindProfile(5.0, 90.0, z_ref=10.0, alpha=0.2)
    np.testing.assert_allclose(prof.velocity(0.0, np.array([0, 0, 10.0])), [0.0, 5.0, 0.0], atol=1e-12)
    assert prof.speed_at(2.5) < prof.speed_at(10.0)
    g = DiscreteGust(10.0, 4.0, np.array([0.0, 6.0, 0.0]))
    assert np.allclose(g.velocity(12.0, None), [0, 6, 0]) and np.allclose(g.velocity(9.9, None), 0)
    a = DrydenTurbulence.low_altitude(5.0, 10.0, 0.0, seed=3, dt=0.01)
    b = DrydenTurbulence.low_altitude(5.0, 10.0, 0.0, seed=3, dt=0.01)
    xa = np.array([a.velocity(k * 0.01, None) for k in range(20000)])
    xb = np.array([b.velocity(k * 0.01, None) for k in range(20000)])
    np.testing.assert_array_equal(xa, xb)  # seeded: reproducible
    np.testing.assert_allclose(xa.std(axis=0), a.sigma, rtol=0.25)  # stationary variance
    assert np.allclose(a.velocity(5.0, None), a.velocity(5.0, None))  # re-query is stable
    c = CompositeWind([prof, g])
    np.testing.assert_allclose(c.velocity(12.0, np.array([0, 0, 10.0])), [0.0, 11.0, 0.0], atol=1e-9)
