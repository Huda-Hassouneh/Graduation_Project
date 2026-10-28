"""Smooth reference trajectories providing position, velocity, acceleration and yaw references.

All references are in world ENU [m, m/s, m/s^2]; yaw [rad] is measured from +x_W (east)
counter-clockwise about +z_W. Every trajectory is at least C^2 so acceleration
feed-forward is continuous (step-like moves use quintic minimum-jerk segments).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

import numpy as np

from graduation_project.dynamics.frames import FloatArray


@dataclass(frozen=True)
class RefPoint:
    position: FloatArray
    velocity: FloatArray
    acceleration: FloatArray
    yaw: float = 0.0
    yaw_rate: float = 0.0


class Trajectory(Protocol):
    duration: float

    def sample(self, t: float) -> RefPoint: ...


def _vec(x: Sequence[float] | FloatArray) -> FloatArray:
    v = np.asarray(x, dtype=float)
    if v.shape != (3,):
        raise ValueError("expected a 3-vector")
    return v


@dataclass(frozen=True)
class Hover:
    position: FloatArray
    yaw: float = 0.0
    duration: float = np.inf

    def sample(self, t: float) -> RefPoint:
        return RefPoint(_vec(self.position).copy(), np.zeros(3), np.zeros(3), self.yaw, 0.0)


class QuinticSegment:
    """Per-axis quintic polynomial matching position/velocity/acceleration at both ends."""

    def __init__(
        self,
        p0: FloatArray,
        p1: FloatArray,
        duration: float,
        v0: FloatArray | None = None,
        v1: FloatArray | None = None,
        a0: FloatArray | None = None,
        a1: FloatArray | None = None,
    ) -> None:
        if duration <= 0.0:
            raise ValueError("segment duration must be positive")
        z = np.zeros(3)
        p0, p1 = _vec(p0), _vec(p1)
        v0 = z if v0 is None else _vec(v0)
        v1 = z if v1 is None else _vec(v1)
        a0 = z if a0 is None else _vec(a0)
        a1 = z if a1 is None else _vec(a1)
        T = duration
        M = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 2, 0, 0, 0],
                [1, T, T**2, T**3, T**4, T**5],
                [0, 1, 2 * T, 3 * T**2, 4 * T**3, 5 * T**4],
                [0, 0, 2, 6 * T, 12 * T**2, 20 * T**3],
            ]
        )
        self.coeffs = np.linalg.solve(M, np.vstack([p0, v0, a0, p1, v1, a1]))  # (6, 3)
        self.duration = T

    def sample_pva(self, t: float) -> tuple[FloatArray, FloatArray, FloatArray]:
        t = float(np.clip(t, 0.0, self.duration))
        c = self.coeffs
        tp = np.array([1, t, t**2, t**3, t**4, t**5])
        dtp = np.array([0, 1, 2 * t, 3 * t**2, 4 * t**3, 5 * t**4])
        ddtp = np.array([0, 0, 2, 6 * t, 12 * t**2, 20 * t**3])
        return tp @ c, dtp @ c, ddtp @ c


class WaypointTrajectory:
    """Stop-and-go sequence of quintic segments (zero velocity/acceleration at each waypoint).

    Segment durations are given, or chosen from ``max_speed`` using the quintic peak speed
    ``v_peak = 1.875 * distance / T``. Holds the final waypoint after the last segment.
    """

    QUINTIC_PEAK_SPEED_FACTOR = 1.875

    def __init__(
        self,
        waypoints: Sequence[Sequence[float]],
        durations: Sequence[float] | None = None,
        max_speed: float | None = None,
        min_segment_time: float = 1.0,
        yaw: float = 0.0,
        start_time: float = 0.0,
    ) -> None:
        pts = [_vec(w) for w in waypoints]
        if len(pts) < 2:
            raise ValueError("need at least two waypoints")
        if durations is None:
            if max_speed is None or max_speed <= 0:
                raise ValueError("give durations or a positive max_speed")
            durations = [
                max(min_segment_time, self.QUINTIC_PEAK_SPEED_FACTOR * np.linalg.norm(b - a) / max_speed)
                for a, b in zip(pts[:-1], pts[1:])
            ]
        if len(durations) != len(pts) - 1:
            raise ValueError("need one duration per segment")
        self.waypoints = pts
        self.segments = [QuinticSegment(a, b, T) for a, b, T in zip(pts[:-1], pts[1:], durations)]
        self.start_times = start_time + np.concatenate(([0.0], np.cumsum(durations)[:-1]))
        self.start_time = start_time
        self.duration = start_time + float(np.sum(durations))
        self.yaw = yaw

    def sample(self, t: float) -> RefPoint:
        if t <= self.start_time:
            return RefPoint(self.waypoints[0].copy(), np.zeros(3), np.zeros(3), self.yaw)
        if t >= self.duration:
            return RefPoint(self.waypoints[-1].copy(), np.zeros(3), np.zeros(3), self.yaw)
        i = int(np.searchsorted(self.start_times, t, side="right") - 1)
        p, v, a = self.segments[i].sample_pva(t - self.start_times[i])
        return RefPoint(p, v, a, self.yaw)


def _smoothstep_phase(t: float, rate: float, ramp: float) -> tuple[float, float, float]:
    """Phase, rate and acceleration with a quintic smoothstep ramp from 0 to ``rate`` over ``ramp`` s."""
    if ramp <= 0.0:
        return rate * t, rate, 0.0
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= ramp:
        return rate * ramp * 0.5 + rate * (t - ramp), rate, 0.0
    s = t / ramp
    phi = rate * ramp * (2.5 * s**4 - 3.0 * s**5 + s**6)
    dphi = rate * (10 * s**3 - 15 * s**4 + 6 * s**5)
    ddphi = rate * (30 * s**2 - 60 * s**3 + 30 * s**4) / ramp
    return phi, dphi, ddphi


@dataclass(frozen=True)
class CircleTrajectory:
    """Horizontal circle (optionally a helix) entered from rest with a smooth speed ramp.

    ``p(t) = c + r [cos phi, sin phi, 0] + [0, 0, climb_per_rad * phi]``; starts at ``c + [r, 0, 0]``.
    """

    center: FloatArray
    radius: float
    speed: float  # tangential speed after the ramp [m/s]
    duration: float
    ramp_time: float = 3.0
    climb_per_rad: float = 0.0  # [m/rad]; non-zero gives a helix
    yaw: float = 0.0

    def sample(self, t: float) -> RefPoint:
        c = _vec(self.center)
        phi, dphi, ddphi = _smoothstep_phase(t, self.speed / self.radius, self.ramp_time)
        r = self.radius
        cs, sn = np.cos(phi), np.sin(phi)
        h = self.climb_per_rad
        p = c + np.array([r * cs, r * sn, h * phi])
        v = np.array([-r * sn * dphi, r * cs * dphi, h * dphi])
        a = np.array(
            [-r * cs * dphi**2 - r * sn * ddphi, -r * sn * dphi**2 + r * cs * ddphi, h * ddphi]
        )
        return RefPoint(p, v, a, self.yaw)


def facade_raster_waypoints(
    facade_origin: Sequence[float],
    facade_tangent: Sequence[float],
    facade_normal: Sequence[float],
    width: float,
    z_min: float,
    z_max: float,
    standoff: float,
    n_passes: int,
    vertical: bool = False,
) -> list[FloatArray]:
    """Boustrophedon coverage waypoints parallel to a vertical facade plane.

    ``facade_origin`` is a point on the wall at ground level, ``facade_tangent`` a horizontal
    direction along the wall and ``facade_normal`` the outward wall normal (both horizontal,
    world frame). Waypoints lie ``standoff`` metres in front of the wall. With ``vertical=False``
    passes are horizontal rows from ``z_min`` to ``z_max``; with ``vertical=True`` they are columns.
    """
    o = _vec(facade_origin)
    t_hat = _vec(facade_tangent) / np.linalg.norm(facade_tangent)
    n_hat = _vec(facade_normal) / np.linalg.norm(facade_normal)
    if abs(t_hat[2]) > 1e-9 or abs(n_hat[2]) > 1e-9 or abs(t_hat @ n_hat) > 1e-9:
        raise ValueError("tangent and normal must be horizontal and orthogonal")
    if n_passes < 1 or standoff <= 0.0 or z_max <= z_min or width <= 0.0:
        raise ValueError("invalid raster parameters")
    base = o + standoff * n_hat
    ez = np.array([0.0, 0.0, 1.0])
    pts: list[FloatArray] = []
    levels = np.linspace(z_min, z_max, n_passes) if not vertical else np.linspace(0.0, width, n_passes)
    for k, level in enumerate(levels):
        if vertical:
            a, b = base + level * t_hat + z_min * ez, base + level * t_hat + z_max * ez
        else:
            a, b = base + level * ez, base + width * t_hat + level * ez
        pts.extend([a, b] if k % 2 == 0 else [b, a])
    return pts
