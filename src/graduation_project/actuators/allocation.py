"""Control allocation: map a commanded wrench ``u = [T, tau_x, tau_y, tau_z]`` to rotor thrusts.

The effectiveness matrix ``B`` (4 x n) satisfies ``u = B @ T_rotors`` for the *nominal* model
(``eta_i = 1``)::

    row 0:  1                      collective thrust along +z_B
    row 1:  y_i                    roll torque   (r_i x T_i e_z)_x
    row 2: -x_i                    pitch torque  (r_i x T_i e_z)_y
    row 3: -s_i * k_m / k_f        yaw (reaction) torque

Saturation handling (documented in docs/controller_design.md): the unconstrained solution
``B^+ u`` is used if it respects ``[T_min, T_max]``. Otherwise the yaw torque is scaled down
(bisection), then roll/pitch torque, keeping collective thrust; any remaining violation is clipped.
This prioritises thrust > roll/pitch > yaw, as is common practice for multirotors.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from graduation_project.actuators.rotor import RotorConfig
from graduation_project.dynamics.frames import FloatArray


def effectiveness_matrix(cfg: RotorConfig) -> FloatArray:
    return np.vstack(
        [
            np.ones(cfg.n_rotors),
            cfg.positions[:, 1],
            -cfg.positions[:, 0],
            -cfg.spin_dirs * cfg.k_m / cfg.k_f,
        ]
    )


@dataclass(frozen=True)
class AllocationResult:
    thrusts: FloatArray  # (n,) allocated rotor thrusts [N]
    achieved_wrench: FloatArray  # (4,) B @ thrusts
    yaw_scale: float  # fraction of commanded yaw torque retained
    attitude_scale: float  # fraction of commanded roll/pitch torque retained
    saturated: bool  # True if the command could not be met exactly


class ControlAllocator:
    def __init__(self, cfg: RotorConfig, n_bisection: int = 20) -> None:
        self.cfg = cfg
        self.B = effectiveness_matrix(cfg)
        self.B_pinv = np.linalg.pinv(self.B)
        self.t_min = cfg.thrust_min
        self.t_max = cfg.thrust_max
        self._n_bisection = n_bisection

    def _feasible(self, thrusts: FloatArray) -> bool:
        return bool(np.all(thrusts >= self.t_min - 1e-9) and np.all(thrusts <= self.t_max + 1e-9))

    def _largest_feasible_scale(self, base: FloatArray, axis_mask: FloatArray) -> float:
        """Largest s in [0, 1] such that allocating base with masked axes scaled by s is feasible."""
        def alloc(s: float) -> FloatArray:
            return self.B_pinv @ (base * (1.0 - axis_mask) + s * base * axis_mask)

        if self._feasible(alloc(1.0)):
            return 1.0
        if not self._feasible(alloc(0.0)):
            return 0.0
        lo, hi = 0.0, 1.0
        for _ in range(self._n_bisection):
            mid = 0.5 * (lo + hi)
            if self._feasible(alloc(mid)):
                lo = mid
            else:
                hi = mid
        return lo

    def allocate(self, wrench: FloatArray) -> AllocationResult:
        u = np.asarray(wrench, dtype=float)
        thrusts = self.B_pinv @ u
        if self._feasible(thrusts):
            return AllocationResult(thrusts, self.B @ thrusts, 1.0, 1.0, False)

        yaw_mask = np.array([0.0, 0.0, 0.0, 1.0])
        yaw_scale = self._largest_feasible_scale(u, yaw_mask)
        u_mod = u * np.array([1.0, 1.0, 1.0, yaw_scale])
        att_scale = 1.0
        if yaw_scale == 0.0:
            att_mask = np.array([0.0, 1.0, 1.0, 0.0])
            att_scale = self._largest_feasible_scale(u_mod, att_mask)
            u_mod = u_mod * np.array([1.0, att_scale, att_scale, 1.0])
        thrusts = np.clip(self.B_pinv @ u_mod, self.t_min, self.t_max)
        return AllocationResult(thrusts, self.B @ thrusts, yaw_scale, att_scale, True)

    def speed_commands(self, thrusts: FloatArray) -> FloatArray:
        """Invert the nominal thrust model ``T = k_f Omega^2`` [rad/s]."""
        return np.sqrt(np.maximum(thrusts, 0.0) / self.cfg.k_f)
