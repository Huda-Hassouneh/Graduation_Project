"""Reference trajectories: boundary conditions, derivative consistency, continuity, raster geometry."""

import numpy as np
import pytest

from graduation_project.planning.trajectories import (
    CircleTrajectory,
    QuinticSegment,
    WaypointTrajectory,
    facade_raster_waypoints,
)


def finite_difference_check(traj, times, h=1e-5, tol_v=1e-6, tol_a=1e-4):
    for t in times:
        pm, pp = traj.sample(t - h).position, traj.sample(t + h).position
        vm, vp = traj.sample(t - h).velocity, traj.sample(t + h).velocity
        s = traj.sample(t)
        np.testing.assert_allclose((pp - pm) / (2 * h), s.velocity, atol=tol_v)
        np.testing.assert_allclose((vp - vm) / (2 * h), s.acceleration, atol=tol_a)


def test_quintic_boundary_conditions() -> None:
    seg = QuinticSegment(np.zeros(3), np.array([3.0, -1.0, 2.0]), 2.0, v1=np.array([1.0, 0, 0]))
    p0, v0, a0 = seg.sample_pva(0.0)
    p1, v1, a1 = seg.sample_pva(2.0)
    np.testing.assert_allclose(p0, 0, atol=1e-12)
    np.testing.assert_allclose(v0, 0, atol=1e-12)
    np.testing.assert_allclose(a0, 0, atol=1e-12)
    np.testing.assert_allclose(p1, [3, -1, 2], atol=1e-12)
    np.testing.assert_allclose(v1, [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(a1, 0, atol=1e-12)


def test_waypoint_trajectory_derivatives_and_speed_limit() -> None:
    traj = WaypointTrajectory([(0, 0, 5), (4, 0, 5), (4, 3, 7)], max_speed=1.5, start_time=1.0)
    ts = np.linspace(0.0, traj.duration + 1.0, 400)
    finite_difference_check(traj, ts[(np.abs(ts - 1.0) > 1e-3)])
    speeds = [np.linalg.norm(traj.sample(t).velocity) for t in ts]
    assert max(speeds) <= 1.5 + 1e-9
    np.testing.assert_allclose(traj.sample(traj.duration + 5).position, [4, 3, 7])


def test_waypoint_trajectory_is_continuous_at_segment_joins() -> None:
    traj = WaypointTrajectory([(0, 0, 0), (1, 0, 0), (1, 1, 0)], durations=[2.0, 2.0])
    a, b = traj.sample(2.0 - 1e-9), traj.sample(2.0 + 1e-9)
    np.testing.assert_allclose(a.position, b.position, atol=1e-8)
    np.testing.assert_allclose(a.velocity, b.velocity, atol=1e-7)
    np.testing.assert_allclose(a.acceleration, b.acceleration, atol=1e-6)


@pytest.mark.parametrize("climb", [0.0, 0.1])
def test_circle_derivatives_and_steady_speed(climb: float) -> None:
    traj = CircleTrajectory(np.array([1.0, 2.0, 5.0]), 3.0, 2.0, 20.0, ramp_time=4.0, climb_per_rad=climb)
    finite_difference_check(traj, np.linspace(0.01, 20.0, 300))
    s0 = traj.sample(0.0)
    np.testing.assert_allclose(s0.position, [4.0, 2.0, 5.0])
    np.testing.assert_allclose(s0.velocity, 0.0, atol=1e-15)
    v = traj.sample(10.0).velocity
    assert np.linalg.norm(v[:2]) == pytest.approx(2.0, rel=1e-12)


def test_circle_ramp_is_c2_continuous_at_ramp_end() -> None:
    traj = CircleTrajectory(np.zeros(3), 2.0, 1.0, 10.0, ramp_time=3.0)
    a, b = traj.sample(3.0 - 1e-9), traj.sample(3.0 + 1e-9)
    np.testing.assert_allclose(a.acceleration, b.acceleration, atol=1e-6)


def test_facade_raster_keeps_standoff_and_alternates() -> None:
    wps = facade_raster_waypoints((0, 8, 0), (1, 0, 0), (0, -1, 0), 12.0, 3.0, 9.0, standoff=3.0, n_passes=3)
    assert len(wps) == 6
    for w in wps:
        assert w[1] == pytest.approx(5.0)  # 3 m in front of the wall plane y = 8
    np.testing.assert_allclose([w[2] for w in wps], [3, 3, 6, 6, 9, 9])
    assert wps[1][0] == pytest.approx(12.0) and wps[2][0] == pytest.approx(12.0)


def test_facade_raster_rejects_non_horizontal_normal() -> None:
    with pytest.raises(ValueError):
        facade_raster_waypoints((0, 0, 0), (1, 0, 0), (0, 0, 1), 1.0, 0.0, 1.0, 1.0, 2)
