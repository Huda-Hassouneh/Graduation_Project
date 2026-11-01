"""Baseline tracking scenarios (Milestone 2). Each returns a trajectory, a SimConfig and a description.

The provisional facade is the vertical plane y = 8 m (wall face toward -y), spanning
x in [0, 12] m. It is a geometric stand-in until the Milestone 3 benchmark world defines
the facade; only reference paths use it here (no collision model yet).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from graduation_project.experiments.runner import SimConfig
from graduation_project.planning.trajectories import (
    CircleTrajectory,
    Hover,
    Trajectory,
    WaypointTrajectory,
    facade_raster_waypoints,
)

FACADE_ORIGIN = (0.0, 8.0, 0.0)
FACADE_TANGENT = (1.0, 0.0, 0.0)
FACADE_NORMAL = (0.0, -1.0, 0.0)
FACADE_WIDTH_M = 12.0  # facade spans x in [0, FACADE_WIDTH_M] along FACADE_TANGENT
SCAN_Z_MIN_M = 3.0
SCAN_Z_MAX_M = 9.0
STANDOFF_M = 3.0


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    trajectory: Trajectory
    sim: SimConfig
    eval_start_s: float  # start of the metric window [s]


def hover_offset() -> Scenario:
    return Scenario(
        "hover_offset",
        "Hold (0,0,5) m starting 0.5 m east/north and 0.3 m low (initial-condition recovery).",
        Hover(np.array([0.0, 0.0, 5.0])),
        SimConfig(duration=15.0, initial_offset=np.array([0.5, 0.5, -0.3])),
        eval_start_s=10.0,
    )


def smooth_step() -> Scenario:
    return Scenario(
        "smooth_step",
        "Quintic 3 m east + 1 m up repositioning in 3 s from hover, then hold.",
        WaypointTrajectory([(0, 0, 5), (3, 0, 6)], durations=[3.0], start_time=1.0),
        SimConfig(duration=12.0),
        eval_start_s=0.0,
    )


def straight_line() -> Scenario:
    return Scenario(
        "straight_line",
        "10 m diagonal line at <= 2 m/s peak speed.",
        WaypointTrajectory([(0, 0, 5), (8, 6, 5)], max_speed=2.0, start_time=1.0),
        SimConfig(duration=12.0),
        eval_start_s=0.0,
    )


def circle() -> Scenario:
    return Scenario(
        "circle",
        "Horizontal circle r = 3 m at 2 m/s, entered with a 4 s smooth ramp.",
        CircleTrajectory(np.array([0.0, 0.0, 5.0]), radius=3.0, speed=2.0, duration=30.0, ramp_time=4.0),
        SimConfig(duration=30.0),
        eval_start_s=0.0,
    )


def helix_3d() -> Scenario:
    return Scenario(
        "helix_3d",
        "Climbing helix r = 3 m, 1.5 m/s, 0.5 m rise per revolution.",
        CircleTrajectory(np.array([0.0, 0.0, 3.0]), radius=3.0, speed=1.5, duration=30.0, ramp_time=4.0,
                         climb_per_rad=0.5 / (2 * np.pi)),
        SimConfig(duration=30.0),
        eval_start_s=0.0,
    )


def _raster(vertical: bool, passes: int, repeats: int = 1) -> WaypointTrajectory:
    wps = facade_raster_waypoints(FACADE_ORIGIN, FACADE_TANGENT, FACADE_NORMAL, width=FACADE_WIDTH_M,
                                  z_min=SCAN_Z_MIN_M, z_max=SCAN_Z_MAX_M, standoff=STANDOFF_M, n_passes=passes, vertical=vertical)
    seq = list(wps)
    for _ in range(repeats - 1):
        seq += list(reversed(seq[-len(wps):]))[1:]
    return WaypointTrajectory(seq, max_speed=1.5, start_time=1.0)


def facade_parallel() -> Scenario:
    traj = _raster(vertical=False, passes=3)
    return Scenario("facade_parallel", "Horizontal raster, 3 rows, 3 m stand-off from the provisional facade.",
                    traj, SimConfig(duration=traj.duration + 3.0), eval_start_s=0.0)


def vertical_scan() -> Scenario:
    traj = _raster(vertical=True, passes=4)
    return Scenario("vertical_scan", "Vertical column scan (4 columns, 3-9 m), 3 m stand-off.",
                    traj, SimConfig(duration=traj.duration + 3.0), eval_start_s=0.0)


def repeated_coverage() -> Scenario:
    traj = _raster(vertical=False, passes=3, repeats=2)
    return Scenario("repeated_coverage", "Horizontal raster flown forward then back (2 coverage passes).",
                    traj, SimConfig(duration=traj.duration + 3.0), eval_start_s=0.0)


SCENARIOS: dict[str, Callable[[], Scenario]] = {
    f.__name__: f
    for f in (hover_offset, smooth_step, straight_line, circle, helix_3d, facade_parallel, vertical_scan, repeated_coverage)
}
