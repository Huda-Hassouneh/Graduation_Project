"""Safe recovery and replanning decisions.

The supervisor maps (diagnosis report, identified parameters, energy state, estimator uncertainty,
mission state) to explicit actions. Rules are evaluated in priority order; SAFETY rules are hard
constraints that override everything else:

1. **Safety monitor** (always on when the framework is enabled)
   * facade clearance ``d_est - r_vehicle - 3 sigma_p < clearance_min``  -> retreat (stand-off shift) or,
     without replanning, safe termination;
   * position uncertainty ``max sigma_p > pos_std_land``                  -> safe termination (land in place);
   * ``max sigma_p > pos_std_degraded``                                   -> reduce speed + increase stand-off.
2. **Energy** - energy-aware: if the remaining mission + reserve is not affordable, truncate the remaining
   passes (replanning) or abort; if even the safe exit is not affordable, land in place. Not energy-aware:
   only the hard low-battery failsafe (reported SoC <= threshold -> abort to exit), like a stock RTH.
3. **Diagnosis-driven** (anomaly present)
   * ``mass_or_thrust_change`` -> UPDATE_MODEL (controller thrust gain = identified gain) if adaptation is
     enabled and the estimate is confident; abort if the implied hover throttle exceeds ``max_hover_throttle``;
   * ``actuator_degradation_or_com_shift`` -> reduce speed, increase stand-off; abort if the torque
     disturbance exceeds ``torque_abort_fraction`` of the nominal roll/pitch torque authority;
   * ``steady_wind`` / ``drag_change`` -> reduce speed; increase stand-off if the force points at the facade;
   * ``wind_gust`` -> continue (transient);
   * ``response_delay`` / ``gnss_delay`` -> reduce speed (slower references make latency less harmful);
   * ``sensor_dropout`` -> reduce speed, increase stand-off; abort if GNSS is missing for ``dropout_abort_s``;
   * ``invalid_measurements`` / ``sensor_inconsistency`` -> reduce speed (the estimator gate rejects outliers);
   * undetermined -> conservative: reduce speed, small stand-off increase, request active diagnosis.
4. **Restore** - after ``restore_after_s`` without anomaly and no persistent cause, return to nominal speed.

Without replanning (ablation) only path-preserving actions are allowed: model update, speed scaling
(re-timing the same path) and termination by vertical descent.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from graduation_project.diagnosis.diagnoser import DiagnosisReport

ACTIONS = ("continue", "update_model", "reduce_speed", "modify_trajectory", "diagnostic_maneuver", "replan_remaining",
           "abort", "safe_termination", "restore_nominal")


@dataclass(frozen=True)
class RecoveryConfig:
    clearance_min: float = 0.5  # [m] beyond vehicle radius and 3 sigma
    pos_std_degraded: float = 0.30  # [m]
    pos_std_land: float = 1.0  # [m]
    speed_scale_disturbance: float = 0.7
    speed_scale_sensor: float = 0.5
    speed_scale_undetermined: float = 0.7
    standoff_step: float = 1.0  # [m]
    standoff_step_small: float = 0.5  # [m]
    max_standoff_increase: float = 2.0  # [m]
    gain_confident_std: float = 0.02
    max_hover_throttle: float = 0.80
    torque_abort_fraction: float = 0.5
    dropout_abort_s: float = 15.0
    restore_after_s: float = 15.0
    decision_period: float = 0.5  # minimum time between decisions [s]


@dataclass(frozen=True)
class Action:
    kind: str
    value: float | None
    reason: str


@dataclass
class RecoverySupervisor:
    thrust_max_total: float
    torque_authority_xy: float  # nominal max roll/pitch torque [N m]
    mass_nominal: float
    gravity: float
    vehicle_radius: float
    cfg: RecoveryConfig = field(default_factory=RecoveryConfig)
    adaptation: bool = True
    replanning: bool = True
    energy_aware: bool = True

    def __post_init__(self) -> None:
        self.standoff_added = 0.0
        self.speed_scale = 1.0
        self.model_gain = 1.0
        self._last_decision = -np.inf
        self._last_anomaly_t = -np.inf
        self._gnss_missing_since: float | None = None
        self.handled: set[str] = set()
        self._last_replan_t = -np.inf
        self.done = False  # abort / termination issued

    def _speed(self, target: float, reason: str, out: list[Action]) -> None:
        if target < self.speed_scale - 1e-9:
            self.speed_scale = target
            out.append(Action("reduce_speed", target, reason))

    def _standoff(self, step: float, reason: str, out: list[Action]) -> None:
        if not self.replanning:
            return
        step = min(step, self.cfg.max_standoff_increase - self.standoff_added)
        if step > 1e-9:
            self.standoff_added += step
            out.append(Action("modify_trajectory", step, reason))

    def decide(self, t: float, report: DiagnosisReport | None, thrust_gain: float, thrust_gain_std: float,
               torque_disturbance: np.ndarray, force_h: np.ndarray, energy, position: np.ndarray,
               position_std: np.ndarray, facade, mission_time_remaining: float, exit_time: float,
               coverage_remaining_time: float, gnss_stale: bool, phase: str) -> list[Action]:
        out: list[Action] = []
        if self.done or phase in ("ground", "takeoff", "landing", "landed"):
            return out
        if t - self._last_decision < self.cfg.decision_period:
            return out
        self._last_decision = t
        cfg = self.cfg
        sigma = float(np.max(position_std))

        # 1. safety monitor -------------------------------------------------------------
        clearance = facade.distance(position) - self.vehicle_radius - 3.0 * sigma
        if clearance < cfg.clearance_min and phase in ("coverage", "transit", "probe", "hold"):
            if self.replanning and self.standoff_added < cfg.max_standoff_increase:
                self._standoff(cfg.standoff_step, f"facade clearance {clearance:.2f} m < {cfg.clearance_min} m", out)
            else:
                self.done = True
                return [Action("safe_termination", None, f"facade clearance {clearance:.2f} m and no admissible retreat")]
        if sigma > cfg.pos_std_land:
            self.done = True
            return [Action("safe_termination", None, f"position uncertainty {sigma:.2f} m > {cfg.pos_std_land} m")]
        if sigma > cfg.pos_std_degraded and "pos_degraded" not in self.handled:
            self.handled.add("pos_degraded")
            self._speed(cfg.speed_scale_sensor, f"position uncertainty {sigma:.2f} m", out)
            self._standoff(cfg.standoff_step, "position uncertainty", out)

        # 2. energy -----------------------------------------------------------------------
        if self.energy_aware and energy.soc is not None:
            if energy.margin_after(exit_time, thrust_gain) < 0.0:
                self.done = True
                return out + [Action("safe_termination", None, "energy insufficient even for the safe exit route + reserve")]
            if energy.margin_after(mission_time_remaining, thrust_gain) < 0.0 and phase in ("coverage", "probe", "hold"):
                if self.replanning and coverage_remaining_time > 0.0:
                    if t - self._last_replan_t >= 10.0:  # let the replanned mission settle before re-evaluating
                        self._last_replan_t = t
                        budget = energy.affordable_time(thrust_gain) - (mission_time_remaining - coverage_remaining_time)
                        out.append(Action("replan_remaining", max(0.0, budget),
                                          f"remaining mission needs more than the affordable energy; keep {max(0.0, budget):.0f} s of coverage"))
                else:
                    self.done = True
                    return out + [Action("abort", None, "remaining mission not affordable with reserve")]
        elif energy.below_hard_reserve():
            self.done = True
            return out + [Action("abort", None, f"low-battery failsafe: SoC {energy.soc:.2f} <= {energy.cfg.hard_reserve_soc}")]

        # 3. diagnosis-driven ------------------------------------------------------------------
        if gnss_stale:
            self._gnss_missing_since = t if self._gnss_missing_since is None else self._gnss_missing_since
            if t - self._gnss_missing_since > cfg.dropout_abort_s:
                self.done = True
                return out + [Action("abort", None, f"GNSS missing for more than {cfg.dropout_abort_s:.0f} s")]
        else:
            self._gnss_missing_since = None
        if report is None or not report.anomaly:
            if (t - self._last_anomaly_t > cfg.restore_after_s and self.speed_scale < 1.0
                    and not (self.handled & {"mass_or_thrust_change", "actuator_degradation_or_com_shift", "drag_change",
                                             "response_delay", "gnss_delay", "pos_degraded"})):
                self.speed_scale = 1.0
                out.append(Action("restore_nominal", 1.0, f"no anomaly for {cfg.restore_after_s:.0f} s"))
            return out
        self._last_anomaly_t = t
        cause = report.cause
        if cause == "mass_or_thrust_change":
            hover_throttle = self.mass_nominal * self.gravity / max(thrust_gain, 1e-3) / self.thrust_max_total
            if hover_throttle > cfg.max_hover_throttle:
                self.done = True
                return out + [Action("abort", None, f"identified hover throttle {hover_throttle:.2f} > {cfg.max_hover_throttle}")]
            if self.adaptation and thrust_gain_std < cfg.gain_confident_std and abs(thrust_gain - self.model_gain) > 0.02:
                self.model_gain = thrust_gain
                out.append(Action("update_model", thrust_gain, f"thrust gain identified {thrust_gain:.3f} +/- {thrust_gain_std:.4f}"))
            self.handled.add(cause)
        elif cause == "actuator_degradation_or_com_shift":
            frac = float(np.linalg.norm(torque_disturbance[:2])) / self.torque_authority_xy
            if frac > cfg.torque_abort_fraction:
                self.done = True
                return out + [Action("abort", None, f"torque disturbance uses {frac:.0%} of roll/pitch authority")]
            self._speed(cfg.speed_scale_disturbance, "actuator asymmetry / CoM shift suspected", out)
            if cause not in self.handled:
                self._standoff(cfg.standoff_step, "actuator asymmetry / CoM shift suspected", out)
            self.handled.add(cause)
        elif cause in ("steady_wind", "drag_change"):
            self._speed(cfg.speed_scale_disturbance, f"{cause}: limit disturbance and tracking error", out)
            toward = -float(np.asarray(force_h) @ facade.normal[:2])
            if toward > 1.0 and cause not in self.handled:
                self._standoff(cfg.standoff_step, f"{cause}: {toward:.1f} N pushing toward the facade", out)
            self.handled.add(cause)
        elif cause == "wind_gust":
            pass
        elif cause in ("response_delay", "gnss_delay"):
            self._speed(cfg.speed_scale_sensor, f"{cause}: slower references reduce latency effects", out)
            self.handled.add(cause)
        elif cause == "sensor_dropout":
            self._speed(cfg.speed_scale_sensor, "sensor dropout", out)
            if cause not in self.handled:
                self._standoff(cfg.standoff_step, "sensor dropout: larger position uncertainty", out)
            self.handled.add(cause)
        elif cause in ("invalid_measurements", "sensor_inconsistency"):
            self._speed(cfg.speed_scale_disturbance, f"{cause}: rely on gating, slow down", out)
            self.handled.add(cause)
        elif cause is None:  # undetermined
            self._speed(cfg.speed_scale_undetermined, "undetermined anomaly: conservative speed", out)
            if "undetermined" not in self.handled:
                self._standoff(cfg.standoff_step_small, "undetermined anomaly: extra stand-off", out)
                self.handled.add("undetermined")
            out.append(Action("diagnostic_maneuver", None, "undetermined anomaly: request active diagnosis"))
        if cause in ("steady_wind", "drag_change", "response_delay") and report.probabilities.get(cause, 0) < 0.75:
            out.append(Action("diagnostic_maneuver", None, f"{cause} with limited confidence"))
        return out
