"""Research framework orchestrator: identification -> diagnosis -> (active diagnosis) -> recovery, energy-aware.

Information boundary: :class:`FrameworkInputs` contains only what a real aircraft provides to an
onboard/offboard computer (estimator output and health statistics, bias-corrected IMU means, the
controller's own allocated wrench, battery telemetry, mission state). The framework has NO reference
to the simulated plant, the simulated battery or the scenario's fault schedule.

Switches (:class:`FrameworkConfig`) implement the experimental conditions and ablations:

* ``enabled``          False -> experiments A/B: stock behaviour (only a low-battery failsafe abort);
* ``adaptation``       online model update of the controller (fixed model vs. online adaptation);
* ``diagnosis_mode``   ``event``: decide whenever detection fires; ``periodic``: decide (and attempt a
  scheduled probe with uninformative hypothesis weights) only every ``periodic_interval`` seconds;
* ``active``           active diagnosis (probes) on/off (passive vs. active);
* ``energy_aware``     energy prediction in recovery and probe cost on/off;
* ``replanning``       path-changing recovery actions on/off.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from graduation_project.active_diagnosis.planner import ActiveConfig, ActiveDiagnosisPlanner
from graduation_project.config import AircraftParams
from graduation_project.diagnosis.diagnoser import HYPOTHESES, DiagnosisConfig, Diagnoser, DiagnosisReport
from graduation_project.dynamics.frames import FloatArray, quat_to_rotmat
from graduation_project.energy.battery import BatteryParams
from graduation_project.energy.manager import EnergyConfig, EnergyManager
from graduation_project.estimation.eskf import EkfDiagnostics
from graduation_project.planning.mission import MissionManager
from graduation_project.recovery.supervisor import Action, RecoveryConfig, RecoverySupervisor
from graduation_project.sensors.state_estimate import StateEstimate
from graduation_project.sysid.online import OnlineIdentifier, SysIdConfig


@dataclass(frozen=True)
class FrameworkConfig:
    enabled: bool = True
    adaptation: bool = True
    diagnosis_mode: str = "event"
    periodic_interval: float = 10.0
    active: bool = True
    energy_aware: bool = True
    replanning: bool = True
    expected_density_ratio: float = 1.0  # site density / sea-level ISA, known to the operator
    rate: float = 50.0

    def __post_init__(self) -> None:
        if self.diagnosis_mode not in ("event", "periodic"):
            raise ValueError("diagnosis_mode must be 'event' or 'periodic'")


@dataclass(frozen=True)
class FrameworkInputs:
    t: float
    est: StateEstimate
    ekf: EkfDiagnostics
    f_body: FloatArray  # mean bias-corrected specific force since the last framework step [m/s^2]
    omega: FloatArray  # mean bias-corrected body rate [rad/s]
    wrench_cmd: FloatArray  # allocator-achieved [T, tau_x, tau_y, tau_z] (the controller's own output)
    battery: FloatArray | None  # latest telemetry [V, I, SoC]
    tracking_error: float  # |p_ref - p_est| [m]
    phase: str
    tag: str


@dataclass
class ResearchFramework:
    cfg: FrameworkConfig
    nominal: AircraftParams
    battery_nominal: BatteryParams
    vehicle_radius: float
    mission: MissionManager
    controller: object  # CascadedPIDController or subclass (model_thrust_gain hook)
    estimator: object | None = None  # ErrorStateEKF (gnss_latency_compensation hook) for sensor-model updates
    energy_cfg: EnergyConfig = field(default_factory=EnergyConfig)
    diag_cfg: DiagnosisConfig = field(default_factory=DiagnosisConfig)
    rec_cfg: RecoveryConfig = field(default_factory=RecoveryConfig)
    active_cfg: ActiveConfig = field(default_factory=ActiveConfig)

    def __post_init__(self) -> None:
        nom = self.nominal
        hover_power = (nom.rotor.n_rotors * nom.rotor.k_m * nom.hover_speed**3 / nom.power.propulsion_efficiency
                       + nom.power.avionics_power)
        self.energy = EnergyManager(self.battery_nominal, hover_power, self.energy_cfg)
        self.sysid = OnlineIdentifier(nom.mass, nom.rotor.tau_motor, float(nom.drag.quadratic[0]), SysIdConfig(rate=self.cfg.rate))
        self.diagnoser = Diagnoser(nom.mass, nom.mass_properties.inertia, nom.rotor.tau_motor,
                                   float(nom.drag.quadratic[0]) / nom.mass, replace(self.diag_cfg, rate=self.cfg.rate),
                                   expected_gain=self.cfg.expected_density_ratio)
        t_max = nom.rotor.thrust_max * nom.rotor.n_rotors
        arm = float(np.max(np.abs(nom.rotor.positions[:, :2])))
        torque_xy = 2.0 * (nom.rotor.thrust_max - nom.rotor.thrust_min) * arm
        self.supervisor = RecoverySupervisor(t_max, torque_xy, nom.mass, nom.gravity, self.vehicle_radius, self.rec_cfg,
                                             adaptation=self.cfg.adaptation, replanning=self.cfg.replanning,
                                             energy_aware=self.cfg.energy_aware)
        self.planner = ActiveDiagnosisPlanner(nom.mass, nom.gravity, t_max, self.vehicle_radius, self.active_cfg,
                                              energy_aware=self.cfg.energy_aware)
        self.report: DiagnosisReport | None = None
        self.events: list[dict] = []
        self._next_periodic = self.cfg.periodic_interval
        self._prev_anomaly = False
        self._prev_cause: str | None = None
        self._last_active_eval = -np.inf
        self._last_reject_reason = ""
        self.active_eval_period = 3.0  # [s] between active-diagnosis evaluations

    def _log(self, t: float, source: str, event: str, detail: str = "", value=None) -> None:
        self.events.append({"t": round(float(t), 4), "source": source, "event": event, "detail": detail,
                            "value": None if value is None else float(value)})

    def airborne(self, inp: FrameworkInputs) -> bool:
        return inp.phase not in ("ground", "landed") and inp.est.position[2] > self.mission.z_rest + 0.3

    # ------------------------------------------------------------------ main step
    def update(self, inp: FrameworkInputs) -> None:
        t, cfg = inp.t, self.cfg
        self.energy.update(t, inp.battery, inp.tag)
        if not cfg.enabled:
            if self.energy.below_hard_reserve() and not self.mission.aborted and inp.phase in ("coverage", "transit", "hold", "probe"):
                self.mission.abort(t, "low-battery failsafe")
                self._log(t, "energy", "abort", "stock low-battery failsafe", self.energy.soc)
            return

        airborne = self.airborne(inp)
        R = quat_to_rotmat(inp.est.quaternion)
        self.sysid.update(t, inp.f_body, R, inp.est.velocity, float(inp.wrench_cmd[0]), airborne)
        g = inp.ekf.stats.get("gnss_pos")
        if g is not None and g.last_innovation is not None:
            self.sysid.update_gnss_innovation(g.last_t_meas, g.last_innovation, inp.est.velocity)
        snap = self.sysid.snapshot()
        self.report = rep = self.diagnoser.update(t, inp.est.quaternion, inp.f_body, inp.omega, inp.wrench_cmd,
                                                  inp.tracking_error, snap, inp.ekf, airborne)
        if rep.anomaly != self._prev_anomaly or rep.cause != self._prev_cause:
            if rep.anomaly:
                self._log(t, "diagnosis", "anomaly" if rep.determined else "anomaly_undetermined", rep.message,
                          rep.confidence)
            else:
                self._log(t, "diagnosis", "cleared", "residuals back within tolerance")
            self._prev_anomaly, self._prev_cause = rep.anomaly, rep.cause

        if cfg.diagnosis_mode == "periodic":
            if t < self._next_periodic:
                return
            self._next_periodic += cfg.periodic_interval
        if inp.phase in ("ground", "takeoff", "landing", "landed"):
            return

        gnss_stale = g is not None and t - max(g.last_arrival_t, 0.0) > 1.0 and t > 2.0
        actions = self.supervisor.decide(
            t, rep, snap.thrust_gain, snap.thrust_gain_std, self.diagnoser.torque_disturbance,
            self.diagnoser.external_force_h, self.energy, inp.est.position, inp.ekf.position_std, self.mission.facade,
            self.mission.remaining_time(t), self.mission.exit_time_estimate(inp.est.position),
            self.mission.remaining_coverage_time(t), gnss_stale, inp.phase)
        # sensor-model adaptation: compensate an identified GNSS latency in the estimator
        if (cfg.adaptation and self.estimator is not None and rep.cause == "gnss_delay"
                and snap.gnss_delay > 3 * snap.gnss_delay_std
                and abs(snap.gnss_delay - self.estimator.gnss_latency_compensation) > 0.03):
            self.estimator.gnss_latency_compensation = float(np.clip(snap.gnss_delay, 0.0, 0.5))
            self._log(t, "recovery", "update_model", f"GNSS latency compensation set to {snap.gnss_delay:.3f} s",
                      snap.gnss_delay)
        want_probe = any(a.kind == "diagnostic_maneuver" for a in actions)
        if cfg.diagnosis_mode == "periodic" and cfg.active:
            want_probe = True
        for a in actions:
            self._apply(t, a, snap.thrust_gain)
        probe_pending = any(s.phase == "probe" and s.t1 > t for s in self.mission.segments)
        if want_probe and (probe_pending or t - self._last_active_eval < self.active_eval_period):
            want_probe = False
        if want_probe and cfg.active and not (self.mission.aborted or self.mission.terminated):
            self._last_active_eval = t
            prior = rep
            if cfg.diagnosis_mode == "periodic":
                uniform = {h: 1.0 / len(HYPOTHESES) for h in HYPOTHESES}
                prior = replace(rep, probabilities=uniform)
            dec = self.planner.evaluate(t, prior, self.sysid, self.energy, inp.est.position, inp.est.velocity,
                                        inp.ekf.position_std, inp.phase, self.mission.facade,
                                        self.mission.exit_time_estimate(inp.est.position), snap.thrust_gain)
            if dec.accepted:
                t0 = self.mission.insert_probe(t, dec.probe)
                self._log(t, "active", "probe_accepted", f"{dec.reason}; starts at {t0:.1f} s", dec.chosen.ig)
            else:
                key = "; ".join(sorted(f"{c.kind}:{','.join(r.split(' ')[0] for r in c.reasons)}" for c in dec.candidates))
                if key != self._last_reject_reason:
                    self._log(t, "active", "probe_rejected", dec.reason)
                    self._last_reject_reason = key
        elif want_probe and not cfg.active:
            self._last_active_eval = t
            if self._last_reject_reason != "passive":
                self._log(t, "active", "probe_not_available", "active diagnosis disabled (passive configuration)")
                self._last_reject_reason = "passive"

    def _apply(self, t: float, a: Action, gain: float) -> None:
        m = self.mission
        if a.kind == "update_model":
            self.controller.model_thrust_gain = float(a.value)
        elif a.kind in ("reduce_speed", "restore_nominal"):
            m.set_speed_scale(t, float(a.value))
        elif a.kind == "modify_trajectory":
            m.shift_standoff(t, float(a.value))
        elif a.kind == "replan_remaining":
            m.truncate_coverage(t, float(a.value))
        elif a.kind == "abort":
            m.abort(t, a.reason)
        elif a.kind == "safe_termination":
            m.land_in_place(t, a.reason, retreat=self.cfg.replanning)
        if a.kind != "diagnostic_maneuver":
            self._log(t, "recovery", a.kind, a.reason, a.value)
