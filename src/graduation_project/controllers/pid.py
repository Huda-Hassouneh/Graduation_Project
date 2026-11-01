"""Cascaded PID baseline: position/velocity -> attitude -> body rate -> wrench -> allocation.

See docs/controller_design.md for the derivation of the structure and gains.

1. Position loop (world frame)::

       e_p = p_ref - p,  e_v = v_ref - v
       a_cmd = a_ref + Kp e_p + Kd e_v + Ki int(e_p)
       F_des = m_nom (a_cmd + g e_z)          (tilt- and thrust-limited)

2. Desired attitude: ``b3 = F_des/|F_des|``, heading from ``yaw_ref``; collective thrust
   ``T = F_des . (R e_z)`` (projection onto the current body z axis).
3. Attitude loop: ``q_e = q^* (x) q_des``; ``omega_des = 2 k_q sign(q_e,w) q_e,vec + R^T yaw_rate_ref e_z``,
   clipped to the body-rate limits.
4. Rate loop: ``tau = J_nom (K_w e_w + K_wi int(e_w)) + omega x J_nom omega``.
5. Allocation with the nominal rotor model; rotor-speed commands ``sqrt(T_i / k_f,nom)``.

Anti-windup: integrators are clamped to a maximum contribution and frozen while the allocator
reports saturation. The controller uses only nominal (configuration) parameters, never truth.

Model update hook: ``model_thrust_gain`` (default 1, i.e. the nominal model) is the identified ratio of
achieved to commanded collective acceleration. ``F_des`` is divided by it, so a recovery action can
apply an online-identified thrust/mass change without retuning gains.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from graduation_project.actuators.allocation import AllocationResult, ControlAllocator
from graduation_project.config import CONFIG_DIR, AircraftParams, load_yaml
from graduation_project.dynamics.frames import (
    FloatArray,
    cross3,
    norm,
    quat_conjugate,
    quat_multiply,
    quat_to_rotmat,
    rotmat_to_quat,
)
from graduation_project.planning.trajectories import RefPoint
from graduation_project.sensors.state_estimate import StateEstimate

E_Z = np.array([0.0, 0.0, 1.0])


@dataclass(frozen=True)
class PIDGains:
    pos_p: FloatArray  # [1/s^2]
    pos_d: FloatArray  # [1/s]
    pos_i: FloatArray  # [1/s^3]
    pos_i_limit: FloatArray  # per-axis max integral contribution to a_cmd [m/s^2]
    att_p: FloatArray  # [1/s]  (rate command per rad of attitude error)
    rate_p: FloatArray  # [1/s]
    rate_i: FloatArray  # [1/s^2]
    rate_i_limit: float  # max integral contribution to angular acceleration [rad/s^2]
    min_thrust_fraction: float  # lower bound on F_des,z as a fraction of m g [-]

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "PIDGains":
        a = lambda k: np.asarray(d[k], dtype=float)  # noqa: E731
        return cls(
            pos_p=a("pos_p"),
            pos_d=a("pos_d"),
            pos_i=a("pos_i"),
            pos_i_limit=a("pos_i_limit"),
            att_p=a("att_p"),
            rate_p=a("rate_p"),
            rate_i=a("rate_i"),
            rate_i_limit=float(d["rate_i_limit"]),
            min_thrust_fraction=float(d["min_thrust_fraction"]),
        )


def position_gains_from_poles(wn: float, zeta: float, a: float) -> tuple[float, float, float]:
    """PID gains placing the closed-loop poles at ``(s + a)(s^2 + 2 zeta wn s + wn^2)``.

    With ideal inner loops each axis is a double integrator; with ``xi = int(e_p)`` the
    error dynamics are ``xi''' + Kd xi'' + Kp xi' + Ki xi = 0``. Returns ``(Kp, Kd, Ki)``.
    """
    if wn <= 0 or zeta <= 0 or a < 0:
        raise ValueError("require wn > 0, zeta > 0, a >= 0")
    return wn**2 + 2 * zeta * wn * a, 2 * zeta * wn + a, a * wn**2


def load_pid_gains(aircraft_name: str, path: str | Path | None = None) -> PIDGains:
    d = load_yaml(path or CONFIG_DIR / "controllers" / "pid_baseline.yaml")
    entry = dict(d["aircraft"][aircraft_name]["gains"])
    design = d["aircraft"][aircraft_name]["position_design"]
    xy = position_gains_from_poles(design["xy"]["wn"], design["xy"]["zeta"], design["xy"]["a"])
    z = position_gains_from_poles(design["z"]["wn"], design["z"]["zeta"], design["z"]["a"])
    for i, key in enumerate(("pos_p", "pos_d", "pos_i")):
        entry[key] = [xy[i], xy[i], z[i]]
    return PIDGains.from_dict(entry)


@dataclass(frozen=True)
class ControlOutput:
    omega_cmd: FloatArray  # rotor-speed commands [rad/s]
    wrench_cmd: FloatArray  # [T, tau_x, tau_y, tau_z] requested before allocation
    allocation: AllocationResult
    q_des: FloatArray  # desired attitude
    omega_des: FloatArray  # desired body rates [rad/s]
    accel_cmd: FloatArray  # commanded world acceleration [m/s^2]


class CascadedPIDController:
    def __init__(self, nominal: AircraftParams, gains: PIDGains) -> None:
        self.nom = nominal
        self.gains = gains
        self.allocator = ControlAllocator(nominal.rotor)
        # integrator clamps depend only on the (constant) gains: compute once
        self._bound_pos = self._integral_bound(gains.pos_i, gains.pos_i_limit)
        self._bound_rate = self._integral_bound(gains.rate_i, gains.rate_i_limit)
        lim = nominal.limits
        self._rate_lim = np.array([lim.max_body_rate, lim.max_body_rate, lim.max_yaw_rate])
        self.reset()

    def reset(self) -> None:
        self.model_thrust_gain = 1.0
        self._int_pos = np.zeros(3)
        self._int_rate = np.zeros(3)
        self._last_t: float | None = None
        self._saturated = False

    @staticmethod
    def _integral_bound(gain: FloatArray, limit: FloatArray | float) -> FloatArray:
        with np.errstate(divide="ignore"):
            return np.where(gain > 0, limit / np.where(gain > 0, gain, 1.0), 0.0)

    @classmethod
    def _clamp_integral(cls, state: FloatArray, gain: FloatArray, limit: FloatArray | float) -> FloatArray:
        bound = cls._integral_bound(gain, limit)
        return np.minimum(np.maximum(state, -bound), bound)

    def _desired_force(self, est: StateEstimate, ref: RefPoint, dt: float) -> tuple[FloatArray, FloatArray]:
        g = self.gains
        e_p = ref.position - est.position
        e_v = ref.velocity - est.velocity
        if dt > 0 and not self._saturated:
            b = self._bound_pos
            self._int_pos = np.minimum(np.maximum(self._int_pos + e_p * dt, -b), b)
        a_cmd = ref.acceleration + g.pos_p * e_p + g.pos_d * e_v + g.pos_i * self._int_pos
        m, grav = self.nom.mass, self.nom.gravity
        f = m * (a_cmd + grav * E_Z) / self.model_thrust_gain
        f[2] = max(f[2], g.min_thrust_fraction * m * grav)
        h_max = f[2] * np.tan(self.nom.limits.max_tilt)
        h = norm(f[:2])
        if h > h_max:
            f[:2] *= h_max / h
        return f, a_cmd

    @staticmethod
    def _desired_attitude(f_des: FloatArray, yaw: float) -> FloatArray:
        b3 = f_des / norm(f_des)
        heading = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        b2 = cross3(b3, heading)
        b2 /= norm(b2)  # well defined because tilt is limited below 90 deg
        b1 = cross3(b2, b3)
        return rotmat_to_quat(np.column_stack([b1, b2, b3]))

    def update(self, est: StateEstimate, ref: RefPoint) -> ControlOutput:
        g = self.gains
        dt = 0.0 if self._last_t is None else est.t - self._last_t
        self._last_t = est.t

        f_des, a_cmd = self._desired_force(est, ref, dt)
        q_des = self._desired_attitude(f_des, ref.yaw)
        R = quat_to_rotmat(est.quaternion)
        thrust = float(f_des @ R[:, 2])

        q_err = quat_multiply(quat_conjugate(est.quaternion), q_des)
        sign = 1.0 if q_err[0] >= 0.0 else -1.0
        omega_des = 2.0 * g.att_p * sign * q_err[1:] + R.T @ (ref.yaw_rate * E_Z)
        omega_des = np.minimum(np.maximum(omega_des, -self._rate_lim), self._rate_lim)

        e_w = omega_des - est.angular_velocity
        if dt > 0 and not self._saturated:
            b = self._bound_rate
            self._int_rate = np.minimum(np.maximum(self._int_rate + e_w * dt, -b), b)
        J = self.nom.mass_properties.inertia
        w = est.angular_velocity
        tau = J @ (g.rate_p * e_w + g.rate_i * self._int_rate) + cross3(w, J @ w)

        wrench = np.concatenate(([thrust], tau))
        alloc = self.allocator.allocate(wrench)
        self._saturated = alloc.saturated
        return ControlOutput(
            omega_cmd=self.allocator.speed_commands(alloc.thrusts),
            wrench_cmd=wrench,
            allocation=alloc,
            q_des=q_des,
            omega_des=omega_des,
            accel_cmd=a_cmd,
        )
