"""Control allocation: exactness, symmetry, and saturation priority (thrust > roll/pitch > yaw)."""

import numpy as np

from graduation_project.actuators.allocation import ControlAllocator
from graduation_project.actuators.rotor import RotorHealth, rotor_wrench_body
from graduation_project.config import AircraftParams


def test_allocation_matrix_is_invertible_and_exact(aircraft: AircraftParams) -> None:
    alloc = ControlAllocator(aircraft.rotor)
    np.testing.assert_allclose(alloc.B @ alloc.B_pinv, np.eye(4), atol=1e-9)
    u = np.array([aircraft.mass * aircraft.gravity, 0.05, -0.03, 0.01])
    res = alloc.allocate(u)
    assert not res.saturated
    np.testing.assert_allclose(res.achieved_wrench, u, atol=1e-9)


def test_allocated_speeds_reproduce_wrench_in_plant_model(aircraft: AircraftParams) -> None:
    alloc = ControlAllocator(aircraft.rotor)
    u = np.array([1.1 * aircraft.mass * aircraft.gravity, 0.02, 0.04, -0.01])
    omega = alloc.speed_commands(alloc.allocate(u).thrusts)
    f, tau = rotor_wrench_body(omega, aircraft.rotor, RotorHealth.nominal(4))
    np.testing.assert_allclose(np.concatenate(([f[2]], tau)), u, rtol=1e-9, atol=1e-12)


def test_hover_wrench_gives_equal_thrusts(aircraft: AircraftParams) -> None:
    alloc = ControlAllocator(aircraft.rotor)
    res = alloc.allocate(np.array([aircraft.mass * aircraft.gravity, 0, 0, 0]))
    np.testing.assert_allclose(res.thrusts, aircraft.hover_thrust_per_rotor, rtol=1e-9)


def test_excessive_yaw_is_sacrificed_before_thrust(aircraft: AircraftParams) -> None:
    alloc = ControlAllocator(aircraft.rotor)
    T = aircraft.mass * aircraft.gravity
    res = alloc.allocate(np.array([T, 0.0, 0.0, 100.0]))
    assert res.saturated and 0.0 < res.yaw_scale < 1.0
    assert abs(res.achieved_wrench[0] - T) / T < 1e-6
    assert np.all(res.thrusts >= alloc.t_min - 1e-9) and np.all(res.thrusts <= alloc.t_max + 1e-9)


def test_infeasible_thrust_is_clipped_within_limits(aircraft: AircraftParams) -> None:
    alloc = ControlAllocator(aircraft.rotor)
    res = alloc.allocate(np.array([10.0 * aircraft.mass * aircraft.gravity, 1.0, 1.0, 1.0]))
    assert res.saturated
    np.testing.assert_allclose(res.thrusts, alloc.t_max)
