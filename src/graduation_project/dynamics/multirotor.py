"""Six-degree-of-freedom multirotor plant (ground truth). Equations: docs/model_equations.md.

State vector ``x`` (length 13 + n):

    x[0:3]    p_W      position of the CoM in world ENU [m]
    x[3:6]    v_W      velocity of the CoM in world ENU [m/s]
    x[6:10]   q        unit quaternion body->world, scalar first
    x[10:13]  omega_B  body angular velocity in FLU [rad/s]
    x[13:]    Omega    rotor speeds [rad/s]

Dynamics::

    p_dot     = v
    v_dot     = (R F_prop_B + F_drag_W + F_ext_W) / m - g e_z
    q_dot     = 0.5 q (x) [0, omega_B]
    omega_dot = J^-1 (tau_prop_B + tau_ext_B - omega_B x J omega_B)
    Omega_dot = (sat(Omega_cmd) - Omega) / tau_motor

Integration: classical fixed-step RK4 with the rotor-speed command and the wind velocity held
constant over a step (zero-order hold); the quaternion is renormalised after every step.
There is no ground-contact model; experiments start airborne.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np

from graduation_project.actuators.rotor import (
    RotorHealth,
    motor_speed_derivative,
    rotor_drag_torques,
    rotor_thrusts,
    rotor_wrench_body,
)
from graduation_project.config import AircraftParams
from graduation_project.dynamics.frames import (
    FloatArray,
    QUAT_IDENTITY,
    cross3,
    quat_derivative,
    quat_normalize,
    quat_to_rotmat,
)
from graduation_project.dynamics.mass_properties import MassProperties
from graduation_project.environment.aero import drag_force_world

E_Z = np.array([0.0, 0.0, 1.0])


class StateValidationError(RuntimeError):
    """Raised when the simulated state is non-physical or numerically invalid."""


@dataclass
class PlantState:
    position: FloatArray
    velocity: FloatArray
    quaternion: FloatArray
    angular_velocity: FloatArray
    rotor_speeds: FloatArray

    def to_vector(self) -> FloatArray:
        return np.concatenate(
            [self.position, self.velocity, self.quaternion, self.angular_velocity, self.rotor_speeds]
        ).astype(float)

    @classmethod
    def from_vector(cls, x: FloatArray) -> "PlantState":
        x = np.asarray(x, dtype=float)
        return cls(x[0:3].copy(), x[3:6].copy(), x[6:10].copy(), x[10:13].copy(), x[13:].copy())

    def copy(self) -> "PlantState":
        return PlantState.from_vector(self.to_vector())


@dataclass
class TruthModifiers:
    """Ground-truth dynamics changes (injected by experiments; hidden from controllers)."""

    mass_properties: MassProperties
    health: RotorHealth
    drag_scale: float = 1.0
    external_force_w: FloatArray = field(default_factory=lambda: np.zeros(3))  # [N]
    external_torque_b: FloatArray = field(default_factory=lambda: np.zeros(3))  # [N m]


@dataclass(frozen=True)
class Wrench:
    """Forces/torques acting on the body at one instant (for logging and tests)."""

    thrusts: FloatArray  # (n,) [N]
    drag_torques: FloatArray  # (n,) [N m]
    force_prop_b: FloatArray  # [N]
    torque_prop_b: FloatArray  # [N m]
    force_drag_w: FloatArray  # [N]
    force_total_w: FloatArray  # including gravity [N]
    torque_total_b: FloatArray  # [N m]


@dataclass(frozen=True)
class ValidationLimits:
    max_angular_rate: float = 50.0  # [rad/s]
    max_speed: float = 100.0  # [m/s]


class MultirotorPlant:
    def __init__(
        self,
        params: AircraftParams,
        initial_state: PlantState | None = None,
        validation: ValidationLimits = ValidationLimits(),
    ) -> None:
        self.params = params
        self.n = params.rotor.n_rotors
        self.truth = TruthModifiers(
            mass_properties=params.mass_properties, health=RotorHealth.nominal(self.n)
        )
        self.validation = validation
        self.state = initial_state.copy() if initial_state is not None else self.hover_state(np.zeros(3))
        self.t = 0.0

    # ------------------------------------------------------------------ construction helpers
    def hover_state(self, position: FloatArray, quaternion: FloatArray = QUAT_IDENTITY) -> PlantState:
        """State at rest with every rotor at the nominal hover speed."""
        return PlantState(
            np.asarray(position, dtype=float).copy(),
            np.zeros(3),
            np.asarray(quaternion, dtype=float).copy(),
            np.zeros(3),
            np.full(self.n, self.params.hover_speed),
        )

    # ------------------------------------------------------------------ truth modifications
    def set_mass_properties(self, mp: MassProperties) -> None:
        self.truth.mass_properties = mp

    def set_rotor_health(self, health: RotorHealth) -> None:
        if health.effectiveness.shape != (self.n,):
            raise ValueError("health vector size mismatch")
        self.truth.health = health

    # ------------------------------------------------------------------ dynamics
    def wrench(self, x: FloatArray, wind_w: FloatArray) -> Wrench:
        p = self.params
        mp = self.truth.mass_properties
        q = x[6:10] / np.linalg.norm(x[6:10])
        R = quat_to_rotmat(q)
        omega_rotors = x[13:]
        f_b, tau_b = rotor_wrench_body(omega_rotors, p.rotor, self.truth.health)
        f_drag_w = drag_force_world(x[3:6], wind_w, R, p.drag, self.truth.drag_scale)
        f_total_w = R @ f_b + f_drag_w + self.truth.external_force_w - mp.mass * p.gravity * E_Z
        tau_total_b = tau_b + self.truth.external_torque_b
        return Wrench(
            thrusts=rotor_thrusts(omega_rotors, p.rotor, self.truth.health),
            drag_torques=rotor_drag_torques(omega_rotors, p.rotor, self.truth.health),
            force_prop_b=f_b,
            torque_prop_b=tau_b,
            force_drag_w=f_drag_w,
            force_total_w=f_total_w,
            torque_total_b=tau_total_b,
        )

    def _net_force_torque(self, x: FloatArray, wind_w: FloatArray) -> tuple[FloatArray, FloatArray]:
        """Lean version of :meth:`wrench` used inside the integrator (same equations)."""
        p = self.params
        rotor = p.rotor
        eff = self.truth.health.effectiveness
        q = x[6:10] / np.linalg.norm(x[6:10])
        R = quat_to_rotmat(q)
        w2 = x[13:] ** 2
        thrusts = eff * rotor.k_f * w2
        drag_q = eff * rotor.k_m * w2
        force_w = R[:, 2] * thrusts.sum()
        force_w += drag_force_world(x[3:6], wind_w, R, p.drag, self.truth.drag_scale)
        force_w += self.truth.external_force_w
        force_w[2] -= self.truth.mass_properties.mass * p.gravity
        torque_b = np.array(
            [
                rotor.positions[:, 1] @ thrusts,
                -(rotor.positions[:, 0] @ thrusts),
                -(rotor.spin_dirs @ drag_q),
            ]
        )
        return force_w, torque_b + self.truth.external_torque_b

    def derivative(self, x: FloatArray, omega_cmd: FloatArray, wind_w: FloatArray) -> FloatArray:
        mp = self.truth.mass_properties
        force_w, torque_b = self._net_force_torque(x, wind_w)
        omega_b = x[10:13]
        xdot = np.empty_like(x)
        xdot[0:3] = x[3:6]
        xdot[3:6] = force_w / mp.mass
        xdot[6:10] = quat_derivative(x[6:10], omega_b)
        xdot[10:13] = mp.inertia_inv @ (torque_b - cross3(omega_b, mp.inertia @ omega_b))
        xdot[13:] = motor_speed_derivative(x[13:], omega_cmd, self.params.rotor, self.truth.health)
        return xdot

    def step(self, dt: float, omega_cmd: FloatArray, wind_w: FloatArray | None = None) -> PlantState:
        """Advance the true state by ``dt`` [s] with RK4 and validate it."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        omega_cmd = np.asarray(omega_cmd, dtype=float)
        if omega_cmd.shape != (self.n,) or not np.all(np.isfinite(omega_cmd)):
            raise ValueError(f"invalid rotor-speed command {omega_cmd}")
        wind = np.zeros(3) if wind_w is None else np.asarray(wind_w, dtype=float)
        x = self.state.to_vector()
        k1 = self.derivative(x, omega_cmd, wind)
        k2 = self.derivative(x + 0.5 * dt * k1, omega_cmd, wind)
        k3 = self.derivative(x + 0.5 * dt * k2, omega_cmd, wind)
        k4 = self.derivative(x + dt * k3, omega_cmd, wind)
        x_new = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self._validate(x_new)
        x_new[6:10] = quat_normalize(x_new[6:10])
        self.state = PlantState.from_vector(x_new)
        self.t += dt
        return self.state

    def _validate(self, x: FloatArray) -> None:
        lim = self.validation
        if not np.all(np.isfinite(x)):
            raise StateValidationError(f"non-finite state at t={self.t:.4f}s")
        if abs(np.linalg.norm(x[6:10]) - 1.0) > 1e-3:
            raise StateValidationError("quaternion norm drifted by more than 1e-3 within one step")
        if np.linalg.norm(x[10:13]) > lim.max_angular_rate:
            raise StateValidationError(f"angular rate exceeds {lim.max_angular_rate} rad/s at t={self.t:.4f}s")
        if np.linalg.norm(x[3:6]) > lim.max_speed:
            raise StateValidationError(f"speed exceeds {lim.max_speed} m/s at t={self.t:.4f}s")

    def snapshot_truth(self) -> TruthModifiers:
        return copy.deepcopy(self.truth)
