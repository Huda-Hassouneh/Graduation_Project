"""Simplified airframe drag (docs/model_equations.md, section 4).

The air-relative velocity in the body frame is ``v_rel_B = R^T (v_W - w_W)`` with wind
velocity ``w_W``. The drag force in the body frame is, per body axis ``j``,

    F_B,j = -c_lin,j * v_rel_B,j - c_quad,j * |v_rel_B,j| * v_rel_B,j

This is a documented lumped approximation: it neglects rotor-induced (blade-flapping) drag
dependence on thrust, ground/wall effects and rotor-wake interaction.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from graduation_project.dynamics.frames import FloatArray


@dataclass(frozen=True)
class DragParams:
    linear: FloatArray  # (3,) body-axis linear coefficients [N s/m]
    quadratic: FloatArray  # (3,) body-axis quadratic coefficients [N s^2/m^2]

    def __post_init__(self) -> None:
        lin = np.asarray(self.linear, dtype=float)
        quad = np.asarray(self.quadratic, dtype=float)
        if lin.shape != (3,) or quad.shape != (3,) or np.any(lin < 0) or np.any(quad < 0):
            raise ValueError("drag coefficients must be non-negative 3-vectors")
        object.__setattr__(self, "linear", lin)
        object.__setattr__(self, "quadratic", quad)

    @classmethod
    def none(cls) -> "DragParams":
        return cls(np.zeros(3), np.zeros(3))


def drag_force_world(
    velocity_w: FloatArray, wind_w: FloatArray, R_wb: FloatArray, params: DragParams, scale: float = 1.0
) -> FloatArray:
    """Drag force in the world frame [N]. ``scale`` is a truth-side multiplier for drag changes."""
    v_rel_b = R_wb.T @ (velocity_w - wind_w)
    f_b = -params.linear * v_rel_b - params.quadratic * np.abs(v_rel_b) * v_rel_b
    return scale * (R_wb @ f_b)
