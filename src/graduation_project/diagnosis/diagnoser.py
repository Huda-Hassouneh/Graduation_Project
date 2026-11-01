"""Self-diagnosis: residual generation, change detection and cautious hypothesis ranking.

Inputs are the framework inputs (estimator output, IMU, the controller's own allocated wrench,
estimator health statistics) and the online-identification snapshot. No truth, no fault labels.

Residual channels (each normalised so that nominal flight gives ``z < 1``):

=============  ======================================================================  ==================
channel        statistic                                                               tolerance (config)
=============  ======================================================================  ==================
gain           ``|theta_hat - theta_expected|``                                        ``gain_tol``
torque         ``|tau_d,xy|``, ``tau_d = J w_dot + w x J w - tau_cmd,lagged`` (LPF)    ``torque_tol``
yaw_torque     ``|tau_d,z|``                                                           ``yaw_torque_tol``
force          ``m_nom |LPF(R [f_x, f_y, 0])_xy|`` (horizontal external force)         ``force_tol``
tracking       ``|p_ref - p_est|``                                                     ``tracking_tol``
lag            ``(tau_hat - tau_nom) / tau_nom`` if identifiable                       ``lag_tol``
gnss_delay     ``delta_hat`` if ``delta_hat > 3 sigma``                                ``delay_tol``
sensors        staleness, invalid samples, chi-square innovation consistency (NIS)    see ``SensorHealth``
=============  ======================================================================  ==================

Detection: one-sided CUSUM ``g = max(0, g + z - 1)`` per channel, alarm when ``g > h``. A channel that
stays in alarm for ``persist_s`` is PERSISTENT; one that clears earlier was TRANSIENT.

Hypotheses (evidence -> normalised probability with an "unexplained" mass ``e0``):
``wind_gust``, ``steady_wind``, ``drag_change``, ``mass_or_thrust_change``,
``actuator_degradation_or_com_shift``, ``response_delay``, ``gnss_delay``, ``sensor_dropout``,
``invalid_measurements``, ``sensor_inconsistency``, ``unexplained``. The report is DETERMINED only if
the best probability is >= ``p_determined`` and exceeds the runner-up by ``margin``; otherwise it says
"An abnormal change was detected, but its cause cannot be determined confidently".
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from graduation_project.dynamics.frames import FloatArray, cross3, quat_to_rotmat
from graduation_project.estimation.eskf import EkfDiagnostics
from graduation_project.sysid.online import SysIdSnapshot

HYPOTHESES = ("wind_gust", "steady_wind", "drag_change", "mass_or_thrust_change", "actuator_degradation_or_com_shift",
              "response_delay", "gnss_delay", "sensor_dropout", "invalid_measurements", "sensor_inconsistency",
              "unexplained")
UNDETERMINED_MSG = "An abnormal change was detected, but its cause cannot be determined confidently."
SENSOR_PERIOD = {"gnss_pos": 0.1, "gnss_vel": 0.1, "baro": 0.02, "heading": 0.1, "range": 0.05}


@dataclass(frozen=True)
class DiagnosisConfig:
    rate: float = 50.0
    gain_tol: float = 0.05
    torque_tol: float = 0.45  # [N m]
    yaw_torque_tol: float = 0.15  # [N m]
    force_tol: float = 1.5  # [N]
    tracking_tol: float = 0.35  # [m]
    lag_tol: float = 0.6  # relative increase of tau_eff
    delay_tol: float = 0.05  # [s]
    cusum_h: float = 12.0
    persist_s: float = 4.0
    clear_s: float = 1.5
    torque_filter_s: float = 0.4
    force_filter_s: float = 0.3
    stale_factor: float = 6.0
    nis_inconsistent: float = 3.0
    p_determined: float = 0.6
    margin: float = 0.25
    unexplained_mass: float = 1.0


@dataclass
class _Channel:
    g: float = 0.0
    alarm: bool = False
    onset: float | None = None
    last_alarm_t: float | None = None
    persistent: bool = False
    transient_count: int = 0
    z: float = 0.0


@dataclass(frozen=True)
class DiagnosisReport:
    t: float
    anomaly: bool
    onset_t: float | None
    probabilities: dict[str, float]
    cause: str | None  # None when nominal or undetermined
    determined: bool
    confidence: float
    channels: dict[str, float]  # normalised residuals z
    persistent: tuple[str, ...]
    message: str


@dataclass
class Diagnoser:
    mass_nominal: float
    inertia_nominal: FloatArray
    tau_nominal: float
    drag_nominal_coeff: float  # c / m_nom [1/m]
    cfg: DiagnosisConfig = field(default_factory=DiagnosisConfig)
    expected_gain: float = 1.0

    def __post_init__(self) -> None:
        self.J = np.asarray(self.inertia_nominal, dtype=float)
        self.ch = {k: _Channel() for k in ("gain", "torque", "yaw_torque", "force", "tracking", "lag", "gnss_delay")}
        self._tau_model = None
        self._omega_prev = None
        self._tau_d = np.zeros(3)
        self._force = np.zeros(2)
        self._invalid_prev: dict[str, int] = {}
        self._invalid_recent: dict[str, float] = {}
        self.anomaly_onset: float | None = None
        self.report: DiagnosisReport | None = None

    # ------------------------------------------------------------------ residuals
    def _residuals(self, t: float, q: FloatArray, f_body: FloatArray, omega: FloatArray, tau_cmd: FloatArray,
                   airborne: bool) -> None:
        dt = 1.0 / self.cfg.rate
        a_m = 1.0 - np.exp(-dt / self.tau_nominal)
        self._tau_model = tau_cmd.copy() if self._tau_model is None else self._tau_model + a_m * (tau_cmd - self._tau_model)
        if self._omega_prev is None:
            self._omega_prev = omega.copy()
        w_dot = (omega - self._omega_prev) / dt
        self._omega_prev = omega.copy()
        if not airborne:
            self._tau_d[:] = 0.0
            self._force[:] = 0.0
            return
        tau_d = self.J @ w_dot + cross3(omega, self.J @ omega) - self._tau_model
        self._tau_d += (1.0 - np.exp(-dt / self.cfg.torque_filter_s)) * (tau_d - self._tau_d)
        R = quat_to_rotmat(q)
        f_ext = self.mass_nominal * (R @ np.array([f_body[0], f_body[1], 0.0]))[:2]
        self._force += (1.0 - np.exp(-dt / self.cfg.force_filter_s)) * (f_ext - self._force)

    def _cusum(self, name: str, z: float, t: float) -> None:
        c = self.ch[name]
        c.z = z
        c.g = max(0.0, c.g + z - 1.0)
        c.g = min(c.g, 5.0 * self.cfg.cusum_h)
        if not c.alarm and c.g > self.cfg.cusum_h:
            c.alarm, c.onset, c.persistent = True, t, False
        if c.alarm:
            if z > 1.0:
                c.last_alarm_t = t
            if t - c.onset >= self.cfg.persist_s:
                c.persistent = True
            if c.g == 0.0 and (c.last_alarm_t is None or t - c.last_alarm_t > self.cfg.clear_s):
                if not c.persistent:
                    c.transient_count += 1
                c.alarm, c.onset, c.persistent = False, None, False

    # ------------------------------------------------------------------ main update
    def update(self, t: float, q: FloatArray, f_body: FloatArray, omega: FloatArray, wrench_cmd: FloatArray,
               tracking_error: float, sysid: SysIdSnapshot, ekf: EkfDiagnostics, airborne: bool) -> DiagnosisReport:
        cfg = self.cfg
        self._residuals(t, q, f_body, omega, np.asarray(wrench_cmd[1:4], dtype=float), airborne)
        z = {
            "gain": abs(sysid.thrust_gain - self.expected_gain) / cfg.gain_tol if airborne else 0.0,
            "torque": float(np.linalg.norm(self._tau_d[:2])) / cfg.torque_tol,
            "yaw_torque": abs(float(self._tau_d[2])) / cfg.yaw_torque_tol,
            "force": float(np.linalg.norm(self._force)) / cfg.force_tol,
            "tracking": tracking_error / cfg.tracking_tol if airborne else 0.0,
            "lag": ((sysid.lag / self.tau_nominal - 1.0) / cfg.lag_tol
                    if airborne and sysid.lag_valid and sysid.lag_std < 0.35 * sysid.lag else 0.0),
            "gnss_delay": (sysid.gnss_delay / cfg.delay_tol
                           if sysid.gnss_delay > 3.0 * sysid.gnss_delay_std and sysid.gnss_delay_r2 > 0.5 else 0.0),
        }
        for name, val in z.items():
            self._cusum(name, max(val, 0.0), t)

        # sensor health (from estimator statistics)
        stale, invalid, inconsistent = [], [], []
        for name, st in ekf.stats.items():
            if name == "range":
                continue  # range is legitimately absent outside the facade extent
            if t - max(st.last_arrival_t, 0.0) > cfg.stale_factor * SENSOR_PERIOD[name] and t > 2.0:
                stale.append(name)
            prev = self._invalid_prev.get(name, 0)
            if st.rejected_invalid > prev:
                self._invalid_recent[name] = t
            self._invalid_prev[name] = st.rejected_invalid
            if t - self._invalid_recent.get(name, -np.inf) < 3.0:
                invalid.append(name)
            if st.nis_ema > cfg.nis_inconsistent:
                inconsistent.append(name)

        ev = dict.fromkeys(HYPOTHESES, 0.0)
        c = self.ch
        force_active = c["force"].alarm
        if force_active and not c["force"].persistent:
            ev["wind_gust"] += c["force"].z
        if force_active and c["force"].persistent:
            cn = self.drag_nominal_coeff
            ident = sysid.drag_coeff_std < 0.35 * cn
            drag_identified = bool(np.any(ident))
            dev = np.abs(sysid.drag_coeff / cn - 1.0)
            significant = ident & (dev > np.maximum(0.6, 3.0 * sysid.drag_coeff_std / cn))
            if drag_identified and np.any(significant):
                ev["drag_change"] += 2.0 * c["force"].z
            elif drag_identified:
                ev["steady_wind"] += 2.0 * c["force"].z
            else:
                ev["steady_wind"] += c["force"].z
                ev["drag_change"] += 0.7 * c["force"].z
        torque_active = c["torque"].alarm or c["yaw_torque"].alarm
        if c["gain"].alarm and not torque_active:
            ev["mass_or_thrust_change"] += c["gain"].z
        if torque_active:
            ev["actuator_degradation_or_com_shift"] += max(c["torque"].z, c["yaw_torque"].z) * (1.5 if c["gain"].alarm else 1.0)
        if c["lag"].alarm:
            ev["response_delay"] += 2.0 * c["lag"].z
        if c["gnss_delay"].alarm:
            ev["gnss_delay"] += 3.0 * c["gnss_delay"].z
        if stale:
            ev["sensor_dropout"] += 5.0 * len(stale)
        if invalid:
            ev["invalid_measurements"] += 5.0 * len(invalid)
        if inconsistent and not c["gnss_delay"].alarm:
            ev["sensor_inconsistency"] += 3.0 * len(inconsistent)
        explained = sum(ev.values())
        if c["tracking"].alarm and explained < 1.0:
            ev["unexplained"] += c["tracking"].z

        anomaly = any(ch.alarm for ch in c.values()) or bool(stale or invalid or inconsistent)
        if anomaly and self.anomaly_onset is None:
            onsets = [ch.onset for ch in c.values() if ch.alarm and ch.onset is not None]
            self.anomaly_onset = min(onsets) if onsets else t
        if not anomaly:
            self.anomaly_onset = None
        total = sum(ev.values()) + cfg.unexplained_mass
        probs = {k: v / total for k, v in ev.items()}
        ranked = sorted(probs.items(), key=lambda kv: kv[1], reverse=True)
        best, p1 = ranked[0]
        p2 = ranked[1][1]
        determined = anomaly and p1 >= cfg.p_determined and (p1 - p2) >= cfg.margin and best != "unexplained"
        if not anomaly:
            msg = "nominal"
        elif determined:
            msg = f"diagnosed: {best} (p = {p1:.2f})"
        else:
            msg = UNDETERMINED_MSG
        self.report = DiagnosisReport(
            t=t, anomaly=anomaly, onset_t=self.anomaly_onset, probabilities=probs,
            cause=best if determined else None, determined=determined, confidence=p1 if anomaly else 0.0,
            channels={k: ch.z for k, ch in c.items()},
            persistent=tuple(k for k, ch in c.items() if ch.persistent), message=msg,
        )
        return self.report

    @property
    def torque_disturbance(self) -> FloatArray:
        return self._tau_d.copy()

    @property
    def external_force_h(self) -> FloatArray:
        return self._force.copy()
