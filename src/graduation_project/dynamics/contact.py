"""Penalty-based ground contact for take-off and landing (flat ground at ``z = ground_z``).

Each landing-gear contact point ``r_i`` (body frame, relative to the CoM) that penetrates the ground
receives a normal force from a spring-damper and a regularised Coulomb friction force::

    pen_i   = ground_z - (p + R r_i)_z                      (> 0 when in contact)
    v_i     = v + R (omega x r_i)                           (contact-point velocity, world)
    N_i     = max(0, k_i pen_i - c_i v_i,z)                 (no adhesion)
    F_t,i   = -mu N_i tanh(|v_t,i| / v_eps) v_t,i / |v_t,i|  (smooth stick-slip approximation)

The force acts at the contact point, so it produces a body torque ``r_i x R^T F_i``. Total stiffness
and damping are split equally between the points. This is a simulation device for touchdown and
lift-off, not a landing-gear model; parameters are SIMULATION_ASSUMPTIONs (config).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from graduation_project.dynamics.frames import FloatArray


@dataclass(frozen=True)
class GroundContactParams:
    points_b: FloatArray  # (k, 3) contact points relative to the CoM, body frame [m]
    stiffness: float  # total vertical stiffness [N/m]
    damping: float  # total vertical damping [N s/m]
    friction: float  # Coulomb coefficient [-]
    v_eps: float = 0.05  # friction regularisation speed [m/s]
    ground_z: float = 0.0  # ground height in the world frame [m]

    def __post_init__(self) -> None:
        pts = np.asarray(self.points_b, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 3:
            raise ValueError("need at least three contact points of shape (k, 3)")
        if self.stiffness <= 0 or self.damping < 0 or self.friction < 0 or self.v_eps <= 0:
            raise ValueError("invalid contact parameters")
        object.__setattr__(self, "points_b", pts)

    @classmethod
    def from_damping_ratio(cls, points_b: FloatArray, mass: float, stiffness: float, damping_ratio: float,
                           friction: float, ground_z: float = 0.0) -> "GroundContactParams":
        """Damping chosen so the vertical mass-spring mode has the requested damping ratio."""
        return cls(points_b, stiffness, 2.0 * damping_ratio * np.sqrt(stiffness * mass), friction, ground_z=ground_z)

    @property
    def reach(self) -> float:
        """Largest distance of a contact point from the CoM [m] (no contact possible above this height)."""
        return float(np.max(np.linalg.norm(self.points_b, axis=1)))

    @property
    def lowest_point_z(self) -> float:
        """Lowest contact point in the body frame (negative) [m]."""
        return float(self.points_b[:, 2].min())

    def resting_height(self, mass: float, gravity: float) -> float:
        """CoM height above ground when resting level under its own weight [m]."""
        return self.ground_z - self.lowest_point_z - mass * gravity / self.stiffness


def contact_wrench(
    p_w: FloatArray, v_w: FloatArray, R: FloatArray, omega_b: FloatArray, params: GroundContactParams
) -> tuple[FloatArray, FloatArray, int]:
    """Total contact force (world) and torque about the CoM (body), and the number of points in contact."""
    r_b = params.points_b
    pos_z = p_w[2] + r_b @ R[2, :]  # (R r_i)_z for every point
    pen = params.ground_z - pos_z
    active = pen > 0.0
    if not np.any(active):
        return np.zeros(3), np.zeros(3), 0
    r_b = r_b[active]
    k_i = params.stiffness / len(params.points_b)
    c_i = params.damping / len(params.points_b)
    vel_w = v_w + (np.cross(omega_b, r_b)) @ R.T  # (k, 3)
    normal = np.maximum(0.0, k_i * pen[active] - c_i * vel_w[:, 2])
    vt = vel_w[:, :2]
    speed = np.linalg.norm(vt, axis=1)
    scale = np.where(speed > 1e-12, np.tanh(speed / params.v_eps) / np.maximum(speed, 1e-12), 0.0)
    ft = -(params.friction * normal * scale)[:, None] * vt
    f_w = np.column_stack([ft, normal])  # (k, 3)
    f_b = f_w @ R  # R^T f for every point
    torque_b = np.cross(r_b, f_b).sum(axis=0)
    return f_w.sum(axis=0), torque_b, int(active.sum())
