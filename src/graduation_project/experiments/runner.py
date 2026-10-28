"""Fixed-step closed-loop simulation: trajectory -> controller -> plant (+ wind, + truth changes).

Timing: the plant is integrated with ``dt_sim``; the controller runs every ``decimation``
plant steps (``dt_control = decimation * dt_sim``) and its rotor-speed command is held
(zero-order hold) between updates. One log row is written per control update, *before*
the command computed at that instant is applied. Energy is integrated at every plant step.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from graduation_project.config import AircraftParams
from graduation_project.controllers.pid import CascadedPIDController
from graduation_project.dynamics.frames import FloatArray, euler_zyx_from_quat, quat_from_axis_angle, rotation_angle_between
from graduation_project.dynamics.multirotor import MultirotorPlant, StateValidationError
from graduation_project.energy.power import EnergyIntegrator, electrical_power, shaft_power
from graduation_project.environment.wind import NO_WIND, WindField
from graduation_project.planning.trajectories import Trajectory
from graduation_project.sensors.state_estimate import ideal_state_feedback


@dataclass(frozen=True)
class SimConfig:
    duration: float  # [s]
    dt_sim: float = 0.004  # [s]; convergence vs. smaller steps: tests/test_closed_loop.py
    control_rate_hz: float = 250.0
    initial_offset: FloatArray = field(default_factory=lambda: np.zeros(3))  # added to ref(0) [m]
    initial_yaw: float | None = None  # [rad]; defaults to ref yaw at t = 0

    @property
    def decimation(self) -> int:
        k = 1.0 / (self.control_rate_hz * self.dt_sim)
        if abs(k - round(k)) > 1e-9 or round(k) < 1:
            raise ValueError("control period must be an integer multiple of dt_sim")
        return int(round(k))


@dataclass(frozen=True)
class ScheduledChange:
    """A ground-truth dynamics change applied at time ``t`` (label/params go to metadata)."""

    t: float
    label: str
    apply: Callable[[MultirotorPlant], None]
    params: dict = field(default_factory=dict)


@dataclass
class RunResult:
    columns: list[str]
    data: FloatArray  # (rows, columns)
    status: str  # "completed" or "failed"
    message: str = ""
    warnings: list[str] = field(default_factory=list)

    def col(self, name: str) -> FloatArray:
        return self.data[:, self.columns.index(name)]

    def cols(self, prefix: str, names: str = "xyz") -> FloatArray:
        return np.column_stack([self.col(f"{prefix}_{c}") for c in names])


def log_columns(n_rotors: int) -> list[str]:
    xyz = lambda p: [f"{p}_{c}" for c in "xyz"]  # noqa: E731
    rot = lambda p: [f"{p}_{i + 1}" for i in range(n_rotors)]  # noqa: E731
    return (
        ["t"]
        + xyz("p") + xyz("v") + ["q_w", "q_x", "q_y", "q_z"] + ["roll", "pitch", "yaw"] + xyz("w")
        + xyz("ref_p") + xyz("ref_v") + xyz("ref_a") + ["ref_yaw"]
        + ["qdes_w", "qdes_x", "qdes_y", "qdes_z", "att_err"] + xyz("wdes")
        + ["pos_err"] + ["T_cmd", "tau_cmd_x", "tau_cmd_y", "tau_cmd_z", "alloc_saturated"]
        + rot("omega_cmd") + rot("omega") + rot("thrust")
        + ["power_el", "energy"] + xyz("wind") + ["mass_true"]
    )


def run_closed_loop(
    aircraft: AircraftParams,
    controller: CascadedPIDController,
    trajectory: Trajectory,
    sim: SimConfig,
    wind: WindField = NO_WIND,
    changes: tuple[ScheduledChange, ...] = (),
) -> RunResult:
    """Simulate. ``aircraft`` defines the *true* plant; the controller holds its own nominal model."""
    ref0 = trajectory.sample(0.0)
    yaw0 = ref0.yaw if sim.initial_yaw is None else sim.initial_yaw
    plant = MultirotorPlant(aircraft)
    plant.state = plant.hover_state(ref0.position + sim.initial_offset, quat_from_axis_angle([0, 0, 1], yaw0))
    controller.reset()

    n = plant.n
    columns = log_columns(n)
    n_ctrl = int(np.floor(sim.duration * sim.control_rate_hz + 1e-9)) + 1
    data = np.full((n_ctrl, len(columns)), np.nan)
    energy = EnergyIntegrator()
    pending = sorted(changes, key=lambda c: c.t)
    status, message = "completed", ""
    omega_cmd = plant.state.rotor_speeds.copy()

    def power_now() -> float:
        return electrical_power(
            shaft_power(plant.state.rotor_speeds, aircraft.rotor.k_m, plant.truth.health.effectiveness),
            aircraft.power,
        )

    energy.update(0.0, power_now())
    t = 0.0
    k = 0
    row = 0
    try:
        while row < n_ctrl:
            while pending and pending[0].t <= t + 1e-12:
                pending.pop(0).apply(plant)
            s = plant.state
            wind_now = wind.velocity(t, s.position)
            if k % sim.decimation == 0:
                ref = trajectory.sample(t)
                out = controller.update(ideal_state_feedback(t, s), ref)
                omega_cmd = out.omega_cmd
                w = plant.wrench(s.to_vector(), wind_now)
                data[row] = np.concatenate(
                    [
                        [t], s.position, s.velocity, s.quaternion, euler_zyx_from_quat(s.quaternion), s.angular_velocity,
                        ref.position, ref.velocity, ref.acceleration, [ref.yaw],
                        out.q_des, [rotation_angle_between(s.quaternion, out.q_des)], out.omega_des,
                        [np.linalg.norm(ref.position - s.position)], out.wrench_cmd, [float(out.allocation.saturated)],
                        omega_cmd, s.rotor_speeds, w.thrusts,
                        [power_now(), energy.energy], wind_now, [plant.truth.mass_properties.mass],
                    ]
                )
                row += 1
                if row >= n_ctrl:
                    break
            plant.step(sim.dt_sim, omega_cmd, wind_now)
            k += 1
            t = k * sim.dt_sim
            energy.update(t, power_now())
    except StateValidationError as exc:
        status, message = "failed", str(exc)
        data = data[:row]
    result = RunResult(columns, data, status, message)
    if status == "completed" and np.nanmean(result.col("alloc_saturated")) > 0.05:
        result.warnings.append("allocator saturated in more than 5% of control steps")
    return result
