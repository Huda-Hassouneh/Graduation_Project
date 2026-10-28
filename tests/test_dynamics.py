"""Rigid-body and propulsion physics tests (Milestone 1). Tolerances are stated per test."""

import numpy as np
import pytest

from graduation_project.actuators.rotor import RotorHealth
from graduation_project.config import AircraftParams
from graduation_project.dynamics.frames import euler_zyx_from_quat, quat_to_rotmat
from graduation_project.dynamics.mass_properties import MassProperties
from graduation_project.dynamics.multirotor import MultirotorPlant, PlantState, StateValidationError


def unpowered_plant(ac: AircraftParams) -> MultirotorPlant:
    """Rotors produce no force/torque and drag is disabled: a pure rigid body under gravity."""
    plant = MultirotorPlant(ac)
    plant.set_rotor_health(RotorHealth(np.zeros(plant.n)))
    plant.truth.drag_scale = 0.0
    return plant


def test_hover_equilibrium_thrust_equals_weight(aircraft: AircraftParams) -> None:
    plant = MultirotorPlant(aircraft)
    w = plant.wrench(plant.state.to_vector(), np.zeros(3))
    weight = aircraft.mass * aircraft.gravity
    assert abs(w.thrusts.sum() - weight) / weight < 1e-12
    xdot = plant.derivative(plant.state.to_vector(), plant.state.rotor_speeds, np.zeros(3))
    assert np.max(np.abs(xdot[3:6])) < 1e-12  # linear acceleration [m/s^2]
    assert np.max(np.abs(xdot[10:13])) < 1e-12  # angular acceleration [rad/s^2]


def test_open_loop_hover_does_not_drift(aircraft: AircraftParams) -> None:
    plant = MultirotorPlant(aircraft)
    cmd = np.full(plant.n, aircraft.hover_speed)
    for _ in range(1000):  # 2 s at 2 ms
        plant.step(0.002, cmd)
    assert np.linalg.norm(plant.state.position) < 1e-9  # [m]
    assert np.linalg.norm(plant.state.angular_velocity) < 1e-12  # [rad/s]


def test_free_fall_matches_analytic_parabola(research_quad: AircraftParams) -> None:
    plant = unpowered_plant(research_quad)
    v0 = np.array([1.0, -2.0, 3.0])
    plant.state.velocity = v0.copy()
    t, dt = 0.0, 0.001
    for _ in range(500):
        plant.step(dt, np.zeros(plant.n))
        t += dt
    g = research_quad.gravity
    expected = v0 * t - 0.5 * g * t**2 * np.array([0, 0, 1])
    np.testing.assert_allclose(plant.state.position, expected, atol=1e-9)


@pytest.mark.slow
def test_torque_free_rotation_conserves_energy_and_world_momentum(research_quad: AircraftParams) -> None:
    plant = unpowered_plant(research_quad)
    J = research_quad.mass_properties.inertia
    assert len(set(np.round(np.diag(J), 6))) == 3  # genuinely asymmetric body
    plant.state.angular_velocity = np.array([1.0, 0.1, 2.0])

    def invariants(s: PlantState) -> tuple[np.ndarray, float]:
        w = s.angular_velocity
        return quat_to_rotmat(s.quaternion) @ (J @ w), 0.5 * w @ J @ w

    L0, E0 = invariants(plant.state)
    for _ in range(10000):  # 10 s at 1 ms
        plant.step(0.001, np.zeros(plant.n))  # marked slow: ~10k RK4 steps
    L1, E1 = invariants(plant.state)
    assert np.linalg.norm(L1 - L0) / np.linalg.norm(L0) < 1e-6
    assert abs(E1 - E0) / E0 < 1e-6


def test_constant_external_yaw_torque_gives_linear_rate(research_quad: AircraftParams) -> None:
    plant = unpowered_plant(research_quad)
    plant.truth.external_torque_b = np.array([0.0, 0.0, 0.01])
    for _ in range(1000):
        plant.step(0.001, np.zeros(plant.n))
    Jzz = research_quad.mass_properties.inertia[2, 2]
    assert plant.state.angular_velocity[2] == pytest.approx(0.01 / Jzz * 1.0, rel=1e-9)


@pytest.mark.parametrize(
    "rotors, axis, sign",
    [
        ((2, 3), 1, +1),  # rear rotors (3 RL, 4 RR) stronger -> +pitch torque (nose down in FLU)
        ((0, 1), 1, -1),  # front rotors stronger -> nose up
        ((0, 3), 0, -1),  # right rotors (1 FR, 4 RR) stronger -> -roll (right side up)
        ((1, 2), 0, +1),  # left rotors stronger -> +roll
        ((0, 2), 2, -1),  # CCW rotors (1, 3) faster -> reaction torque clockwise (-z)
        ((1, 3), 2, +1),  # CW rotors faster -> +yaw
    ],
)
def test_torque_sign_conventions(aircraft: AircraftParams, rotors: tuple, axis: int, sign: int) -> None:
    plant = MultirotorPlant(aircraft)
    x = plant.state.to_vector()
    x[13 + np.array(rotors)] *= 1.05
    w = plant.wrench(x, np.zeros(3))
    others = [i for i in range(3) if i != axis]
    assert np.sign(w.torque_prop_b[axis]) == sign
    assert np.all(np.abs(w.torque_prop_b[others]) < 1e-9 * abs(w.torque_prop_b[axis]) + 1e-12)
    assert w.force_prop_b[2] > aircraft.mass * aircraft.gravity  # thrust points along +z_B


def test_positive_pitch_torque_tilts_nose_down_and_accelerates_forward(research_quad: AircraftParams) -> None:
    plant = MultirotorPlant(research_quad)
    plant.truth.external_torque_b = np.array([0.0, 0.002, 0.0])
    for _ in range(300):
        plant.step(0.001, np.full(4, research_quad.hover_speed))
    roll, pitch, yaw = euler_zyx_from_quat(plant.state.quaternion)
    assert pitch > 0 and plant.state.velocity[0] > 0  # thrust tilted forward -> +x_W acceleration


def test_motor_lag_is_first_order(research_quad: AircraftParams) -> None:
    plant = MultirotorPlant(research_quad)
    w0 = plant.state.rotor_speeds[0]
    target = 1.2 * w0
    tau = research_quad.rotor.tau_motor
    dt = 0.0005
    for _ in range(int(round(tau / dt))):
        plant.step(dt, np.full(4, target))
    expected = target + (w0 - target) * np.exp(-1.0)
    np.testing.assert_allclose(plant.state.rotor_speeds, expected, rtol=1e-8)


def test_increased_lag_factor_slows_motor(research_quad: AircraftParams) -> None:
    fast, slow = MultirotorPlant(research_quad), MultirotorPlant(research_quad)
    slow.set_rotor_health(RotorHealth(np.ones(4), np.full(4, 2.0)))
    cmd = np.full(4, 1.2 * research_quad.hover_speed)
    for _ in range(20):
        fast.step(0.001, cmd)
        slow.step(0.001, cmd)
    assert np.all(slow.state.rotor_speeds < fast.state.rotor_speeds)


def test_rotor_speed_saturates_at_limits(research_quad: AircraftParams) -> None:
    plant = MultirotorPlant(research_quad)
    plant.truth.drag_scale = 0.0
    for _ in range(1000):
        plant.step(0.001, np.full(4, 10.0 * research_quad.rotor.omega_max))
    np.testing.assert_allclose(plant.state.rotor_speeds, research_quad.rotor.omega_max, rtol=1e-6)
    assert np.all(plant.wrench(plant.state.to_vector(), np.zeros(3)).thrusts <= research_quad.rotor.thrust_max * (1 + 1e-6))


def test_mass_increase_produces_expected_sink_rate(aircraft: AircraftParams) -> None:
    plant = MultirotorPlant(aircraft)
    mp = aircraft.mass_properties
    heavier = MassProperties(1.2 * mp.mass, 1.2 * mp.inertia)
    plant.set_mass_properties(heavier)
    xdot = plant.derivative(plant.state.to_vector(), plant.state.rotor_speeds, np.zeros(3))
    expected_az = mp.mass * aircraft.gravity / heavier.mass - aircraft.gravity
    assert xdot[5] == pytest.approx(expected_az, rel=1e-12)


def test_reduced_rotor_effectiveness_creates_torque_imbalance(research_quad: AircraftParams) -> None:
    plant = MultirotorPlant(research_quad)
    plant.set_rotor_health(RotorHealth(np.array([0.8, 1.0, 1.0, 1.0])))
    w = plant.wrench(plant.state.to_vector(), np.zeros(3))
    assert w.thrusts[0] == pytest.approx(0.8 * w.thrusts[1])
    assert w.torque_prop_b[0] > 0 and w.torque_prop_b[1] > 0  # FR weak: left side up, nose down


@pytest.mark.slow
def test_rk4_converges_at_fourth_order(research_quad: AircraftParams) -> None:
    """Open-loop manoeuvre; error vs. a fine reference should shrink ~16x per halving of dt."""
    cmd = research_quad.hover_speed * np.array([1.03, 0.98, 1.01, 0.99])

    def run(dt: float) -> np.ndarray:
        plant = MultirotorPlant(research_quad)
        plant.state.angular_velocity = np.array([0.3, -0.2, 0.5])
        for _ in range(int(round(0.8 / dt))):
            plant.step(dt, cmd)
        return plant.state.to_vector()

    ref = run(0.000125)
    e1 = np.linalg.norm(run(0.004) - ref)
    e2 = np.linalg.norm(run(0.002) - ref)
    assert e1 / e2 > 10.0
    assert e2 < 1e-6


def test_wind_produces_drag_toward_wind_direction(aircraft: AircraftParams) -> None:
    plant = MultirotorPlant(aircraft)
    w = plant.wrench(plant.state.to_vector(), np.array([5.0, 0.0, 0.0]))
    assert w.force_drag_w[0] > 0 and abs(w.force_drag_w[1]) < 1e-12


def test_state_validation_rejects_divergence(research_quad: AircraftParams) -> None:
    plant = unpowered_plant(research_quad)
    plant.state.angular_velocity = np.array([0.0, 0.0, 60.0])
    with pytest.raises(StateValidationError):
        plant.step(0.001, np.zeros(4))


def test_integrator_force_path_matches_logging_wrench(aircraft: AircraftParams) -> None:
    """The lean integrator path and the logging path implement the same equations."""
    plant = MultirotorPlant(aircraft)
    plant.set_rotor_health(RotorHealth(np.array([0.9, 1.0, 0.7, 1.0])))
    plant.truth.external_force_w = np.array([0.3, -0.2, 0.1])
    rng = np.random.default_rng(3)
    x = plant.state.to_vector()
    x[3:6] = rng.normal(size=3)
    x[6:10] = rng.normal(size=4)
    x[13:] *= rng.uniform(0.8, 1.2, size=4)
    wind = np.array([2.0, 1.0, -0.5])
    f, tau = plant._net_force_torque(x, wind)
    w = plant.wrench(x, wind)
    np.testing.assert_allclose(f, w.force_total_w, atol=1e-12)
    np.testing.assert_allclose(tau, w.torque_total_b, atol=1e-12)
