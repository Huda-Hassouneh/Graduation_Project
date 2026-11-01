"""15-state error-state extended Kalman filter (ESKF) for position, velocity, attitude and IMU biases.

Nominal state: ``p`` (world ENU), ``v``, ``q`` (body->world), accelerometer bias ``b_a``, gyro bias ``b_g``.
Error state ``dx = [dp, dv, dtheta, db_a, db_g]`` with the LOCAL attitude error ``R_true = R (I + [dtheta]x)``.

Propagation with IMU sample ``(f_m, w_m)`` over ``dt``::

    a      = R (f_m - b_a) - g e_z
    p     += v dt + a dt^2 / 2,   v += a dt,   q = q (x) Exp((w_m - b_g) dt)
    F      = I + dt * [[0, I, 0, 0, 0],
                       [0, 0, -R [f_m - b_a]x, -R, 0],
                       [0, 0, -[w_m - b_g]x, 0, -I],
                       [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    P      = F P F^T + Q(dt)

Aiding measurements and Jacobians (``H`` acts on ``dx``):

* GNSS position ``z = p`` -> ``H = [I 0 0 0 0]``; GNSS velocity -> ``[0 I 0 0 0]``;
* barometric altitude ``z = p_z`` -> ``[e_z^T 0 0 0 0]``;
* heading ``z = yaw(q)`` -> ``[0 0 e_z^T R 0 0]`` (small-tilt approximation), innovation wrapped;
* facade range ``z = n . (p - o)`` -> ``[n^T 0 0 0 0]``.

Delayed measurements: GNSS/heading carry their time of validity. The innovation is formed against
the nominal state stored at that time (ring buffer) and the correction is applied to the current
state ("delayed-state" approximation; exact for position/velocity offsets, approximate for the
cross-covariance with motion since then).

Every update passes a chi-square innovation gate (99.9 %). NaN measurements and gated outliers are
rejected and counted. Per-sensor statistics (NIS, acceptance, staleness) are exposed for diagnosis;
they are computed from measurements and the filter only, never from truth.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from graduation_project.dynamics.frames import FloatArray, cross3, quat_exp_increment, quat_multiply, quat_normalize, quat_to_rotmat, skew
from graduation_project.sensors.models import FacadePlane, ImuSample, Measurement, SensorParams
from graduation_project.sensors.state_estimate import StateEstimate

ESKF = "eskf"
CHI2_999 = {1: 10.828, 2: 13.816, 3: 16.266}
E_Z = np.array([0.0, 0.0, 1.0])
I3 = np.eye(3)


@dataclass
class SensorStats:
    accepted: int = 0
    rejected_gate: int = 0
    rejected_invalid: int = 0
    last_accept_t: float = -np.inf
    last_arrival_t: float = -np.inf
    nis_ema: float = 1.0  # exponential moving average of NIS / dof (expected ~1)
    last_innovation: FloatArray | None = None
    last_t_meas: float = -np.inf


@dataclass(frozen=True)
class EkfDiagnostics:
    """Read-only snapshot for the research framework (no truth inside)."""

    t: float
    position_std: FloatArray
    velocity_std: FloatArray
    attitude_std: FloatArray
    accel_bias: FloatArray
    gyro_bias: FloatArray
    stats: dict[str, SensorStats]


@dataclass
class ErrorStateEKF:
    sensors: SensorParams
    gravity: float
    facade: FacadePlane | None = None
    p: FloatArray = field(default_factory=lambda: np.zeros(3))
    v: FloatArray = field(default_factory=lambda: np.zeros(3))
    q: FloatArray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    ba: FloatArray = field(default_factory=lambda: np.zeros(3))
    bg: FloatArray = field(default_factory=lambda: np.zeros(3))
    nis_alpha: float = 0.05
    gnss_latency_compensation: float = 0.0  # [s] subtracted from GNSS/heading timestamps (identified online)
    baro_std_inflation: float = 2.0  # baro bias drift is not a filter state; inflate its noise instead

    def __post_init__(self) -> None:
        self.P = np.diag(np.concatenate([np.full(3, 0.05**2), np.full(3, 0.02**2), [0.02**2, 0.02**2, 0.05**2],
                                         np.full(3, 0.05**2), np.full(3, 0.003**2)]))
        s = self.sensors
        acc_std = np.hypot(s.accel_noise_density * np.sqrt(s.imu_rate), s.vibration_accel_std)
        self._q_vel_density2 = (acc_std**2) / s.imu_rate  # var per second
        self._q_att_density2 = s.gyro_noise_density**2
        self._q_ba_density2 = s.accel_bias_rw**2
        self._q_bg_density2 = s.gyro_bias_rw**2
        self._hist: deque[tuple[float, FloatArray, FloatArray, FloatArray]] = deque(maxlen=200)
        self.stats = {name: SensorStats() for name in ("gnss_pos", "gnss_vel", "baro", "heading", "range")}
        self.t = 0.0
        self._omega = np.zeros(3)
        self._f = np.array([0.0, 0.0, self.gravity])

    # ------------------------------------------------------------------ propagation
    def predict(self, imu: ImuSample, dt: float) -> None:
        R = quat_to_rotmat(self.q)
        f = imu.accel - self.ba
        w = imu.gyro - self.bg
        a = R @ f - self.gravity * E_Z
        self.p = self.p + self.v * dt + 0.5 * a * dt * dt
        self.v = self.v + a * dt
        self.q = quat_normalize(quat_multiply(self.q, quat_exp_increment(w, dt)))
        F = np.eye(15)
        F[0:3, 3:6] = I3 * dt
        F[3:6, 6:9] = -R @ skew(f) * dt
        F[3:6, 9:12] = -R * dt
        F[6:9, 6:9] = I3 - skew(w) * dt
        F[6:9, 12:15] = -I3 * dt
        P = F @ self.P @ F.T
        P[3:6, 3:6] += I3 * (self._q_vel_density2 * dt)
        P[6:9, 6:9] += I3 * (self._q_att_density2 * dt)
        P[9:12, 9:12] += I3 * (self._q_ba_density2 * dt)
        P[12:15, 12:15] += I3 * (self._q_bg_density2 * dt)
        self.P = 0.5 * (P + P.T)
        self.t = imu.t
        self._omega = w
        self._f = f
        self._hist.append((self.t, self.p.copy(), self.v.copy(), self.q.copy()))

    def _state_at(self, t: float) -> tuple[FloatArray, FloatArray, FloatArray]:
        for (ts, p, v, q) in reversed(self._hist):
            if ts <= t + 1e-9:
                return p, v, q
        if self._hist:
            _, p, v, q = self._hist[0]
            return p, v, q
        return self.p, self.v, self.q

    # ------------------------------------------------------------------ updates
    def _update(self, name: str, H: FloatArray, r: FloatArray, Rm: FloatArray, t_arrival: float, t_meas: float) -> bool:
        st = self.stats[name]
        st.last_arrival_t = t_arrival
        S = H @ self.P @ H.T + Rm
        Sinv = np.linalg.inv(S)
        nis = float(r @ Sinv @ r)
        m = len(r)
        st.last_innovation = r.copy()
        st.last_t_meas = t_meas
        if nis > CHI2_999[m]:
            st.rejected_gate += 1
            st.nis_ema = (1 - self.nis_alpha) * st.nis_ema + self.nis_alpha * min(nis / m, 50.0)
            return False
        K = self.P @ H.T @ Sinv
        dx = K @ r
        IKH = np.eye(15) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.p = self.p + dx[0:3]
        self.v = self.v + dx[3:6]
        dq = np.concatenate(([1.0], 0.5 * dx[6:9]))
        self.q = quat_normalize(quat_multiply(self.q, dq))
        self.ba = self.ba + dx[9:12]
        self.bg = self.bg + dx[12:15]
        st.accepted += 1
        st.last_accept_t = t_arrival
        st.nis_ema = (1 - self.nis_alpha) * st.nis_ema + self.nis_alpha * (nis / m)
        return True

    def update(self, meas: Measurement, t_now: float) -> bool:
        name, z = meas.sensor, np.asarray(meas.value, dtype=float)
        if name == "battery":
            return False
        if not np.all(np.isfinite(z)):
            st = self.stats[name]
            st.rejected_invalid += 1
            st.last_arrival_t = t_now
            return False
        s = self.sensors
        t_valid = meas.t - (self.gnss_latency_compensation if name in ("gnss_pos", "gnss_vel", "heading") else 0.0)
        p_m, v_m, q_m = self._state_at(t_valid)
        H = np.zeros((3 if name in ("gnss_pos", "gnss_vel") else 1, 15))
        if name == "gnss_pos":
            H[:, 0:3] = I3
            return self._update(name, H, z - p_m, np.diag(s.gnss_pos_std**2), t_now, meas.t)
        if name == "gnss_vel":
            H[:, 3:6] = I3
            return self._update(name, H, z - v_m, I3 * s.gnss_vel_std**2, t_now, meas.t)
        if name == "baro":
            H[0, 2] = 1.0
            std = s.baro_std * self.baro_std_inflation
            return self._update(name, H, z - p_m[2:3], np.array([[std**2]]), t_now, meas.t)
        if name == "heading":
            R = quat_to_rotmat(q_m)
            yaw = np.arctan2(R[1, 0], R[0, 0])
            H[0, 6:9] = R[2, :]  # e_z^T R
            r = np.array([np.arctan2(np.sin(z[0] - yaw), np.cos(z[0] - yaw))])
            return self._update(name, H, r, np.array([[s.heading_std**2]]), t_now, meas.t)
        if name == "range":
            if self.facade is None:
                return False
            H[0, 0:3] = self.facade.normal
            r = z - np.array([self.facade.distance(p_m)])
            return self._update(name, H, r, np.array([[s.range_std**2]]), t_now, meas.t)
        raise ValueError(f"unknown measurement {name}")

    # ------------------------------------------------------------------ outputs
    def estimate(self) -> StateEstimate:
        return StateEstimate(self.t, self.p.copy(), self.v.copy(), self.q.copy(), self._omega.copy(), True, ESKF)

    def diagnostics(self) -> EkfDiagnostics:
        d = np.sqrt(np.maximum(np.diag(self.P), 0.0))
        return EkfDiagnostics(self.t, d[0:3], d[3:6], d[6:9], self.ba.copy(), self.bg.copy(), self.stats)

    @property
    def specific_force(self) -> FloatArray:
        """Bias-corrected specific force of the latest IMU sample, body frame [m/s^2]."""
        return self._f
