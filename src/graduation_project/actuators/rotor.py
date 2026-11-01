"""Rotor propulsion model: geometry, first-order motor-speed lag, thrust and reaction torque.

Model (docs/model_equations.md, section 3):

* Motor speed ``Omega_i`` [rad/s] follows ``dOmega_i/dt = (sat(Omega_cmd_i) - Omega_i) / tau_i``,
  where ``sat`` clamps to ``[omega_min, omega_max]``. ``tau_i = tau_motor * lag_factor_i``.
* Thrust ``T_i = eta_i * k_f * Omega_i**2`` along ``+z_B``.
* Drag (reaction) torque magnitude ``Q_i = eta_i * k_m * Omega_i**2``. A rotor with spin
  direction ``s_i = +1`` turns counter-clockwise seen from above (angular velocity along ``+z_B``)
  and exerts ``-s_i * Q_i * z_B`` on the airframe.
* ``eta_i`` in [0, 1] is the rotor effectiveness (1 = nominal); it is a *truth* parameter used to
  inject actuator degradation and is not known to the controller.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from graduation_project.dynamics.frames import FloatArray

E_Z = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class RotorConfig:
    """Static description of an ``n``-rotor propulsion system (body FLU frame, SI units)."""

    positions: FloatArray  # (n, 3) rotor hub positions relative to the CoM [m]
    spin_dirs: FloatArray  # (n,) +1 = CCW seen from above, -1 = CW
    k_f: float  # thrust coefficient [N/(rad/s)^2]
    k_m: float  # drag-torque coefficient [N m/(rad/s)^2]
    tau_motor: float  # motor-speed time constant [s]
    omega_min: float  # minimum (idle) rotor speed [rad/s]
    omega_max: float  # maximum rotor speed [rad/s]

    def __post_init__(self) -> None:
        pos = np.asarray(self.positions, dtype=float)
        spins = np.asarray(self.spin_dirs, dtype=float)
        if pos.ndim != 2 or pos.shape[1] != 3:
            raise ValueError("positions must have shape (n, 3)")
        if spins.shape != (pos.shape[0],) or not np.all(np.isin(spins, (-1.0, 1.0))):
            raise ValueError("spin_dirs must be +/-1 for every rotor")
        if not (self.k_f > 0 and self.k_m > 0 and self.tau_motor > 0):
            raise ValueError("k_f, k_m and tau_motor must be positive")
        if not 0.0 <= self.omega_min < self.omega_max:
            raise ValueError("require 0 <= omega_min < omega_max")
        object.__setattr__(self, "positions", pos)
        object.__setattr__(self, "spin_dirs", spins)

    @property
    def n_rotors(self) -> int:
        return self.positions.shape[0]

    @property
    def thrust_min(self) -> float:
        return self.k_f * self.omega_min**2

    @property
    def thrust_max(self) -> float:
        return self.k_f * self.omega_max**2


@dataclass
class RotorHealth:
    """Ground-truth actuator condition (never read by controllers or estimators)."""

    effectiveness: FloatArray  # (n,) multiplies k_f and k_m, in [0, 1]
    lag_factor: FloatArray = field(default=None)  # type: ignore[assignment]  # (n,) multiplies tau_motor, >= 1 typical

    def __post_init__(self) -> None:
        self.effectiveness = np.asarray(self.effectiveness, dtype=float)
        if self.lag_factor is None:
            self.lag_factor = np.ones_like(self.effectiveness)
        self.lag_factor = np.asarray(self.lag_factor, dtype=float)
        if np.any(self.effectiveness < 0.0) or np.any(self.effectiveness > 1.0):
            raise ValueError("rotor effectiveness must lie in [0, 1]")
        if np.any(self.lag_factor <= 0.0):
            raise ValueError("lag_factor must be positive")

    @classmethod
    def nominal(cls, n_rotors: int) -> "RotorHealth":
        return cls(np.ones(n_rotors), np.ones(n_rotors))


def saturate_speed_command(omega_cmd: FloatArray, cfg: RotorConfig) -> FloatArray:
    return np.minimum(np.maximum(omega_cmd, cfg.omega_min), cfg.omega_max)


def motor_speed_derivative(
    omega: FloatArray, omega_cmd: FloatArray, cfg: RotorConfig, health: RotorHealth
) -> FloatArray:
    """First-order lag toward the saturated command."""
    return (saturate_speed_command(omega_cmd, cfg) - omega) / (cfg.tau_motor * health.lag_factor)


def rotor_thrusts(omega: FloatArray, cfg: RotorConfig, health: RotorHealth) -> FloatArray:
    """Actual per-rotor thrust [N]."""
    return health.effectiveness * cfg.k_f * omega**2


def rotor_drag_torques(omega: FloatArray, cfg: RotorConfig, health: RotorHealth) -> FloatArray:
    """Per-rotor aerodynamic drag-torque magnitude ``Q_i`` [N m]."""
    return health.effectiveness * cfg.k_m * omega**2


def rotor_wrench_body(
    omega: FloatArray, cfg: RotorConfig, health: RotorHealth
) -> tuple[FloatArray, FloatArray]:
    """Total propulsive force and torque about the CoM, both in the body frame."""
    thrusts = rotor_thrusts(omega, cfg, health)
    drag = rotor_drag_torques(omega, cfg, health)
    force = np.array([0.0, 0.0, thrusts.sum()])
    # r_i x (T_i e_z) = T_i * [y_i, -x_i, 0]
    torque = np.array(
        [
            np.dot(cfg.positions[:, 1], thrusts),
            -np.dot(cfg.positions[:, 0], thrusts),
            -np.dot(cfg.spin_dirs, drag),
        ]
    )
    return force, torque


def x_config_positions(arm_length: float, rotor_height: float = 0.0) -> FloatArray:
    """Hub positions of a symmetric X quadrotor, numbered counter-clockwise from front-right.

    Rotor 1 front-right (azimuth -45 deg), 2 front-left (+45), 3 rear-left (+135),
    4 rear-right (-135). ``arm_length`` is the CoM-to-hub distance in the rotor plane [m]
    (half the diagonal wheelbase); ``rotor_height`` is the rotor plane height above the CoM [m].
    """
    azimuths = np.deg2rad([-45.0, 45.0, 135.0, -135.0])
    return np.column_stack(
        [arm_length * np.cos(azimuths), arm_length * np.sin(azimuths), np.full(4, rotor_height)]
    )
