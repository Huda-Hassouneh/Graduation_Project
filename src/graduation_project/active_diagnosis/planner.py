"""Active diagnosis: choose (or reject) a bounded diagnostic manoeuvre by information, energy and safety.

For each candidate probe ``a`` (vertical excitation, along-facade speed excitation, hover-and-observe):

* expected information gain ``IG(a) = sum_h P(h) * relevance(a, h) * IG_param(a, h)`` [nats], where
  ``IG_param`` is computed from the CURRENT estimator uncertainty and the planned regressor sequence:
  - thrust gain / drag / GNSS-latency parameters: RLS information ``0.5 log det(P_prior P_post^-1)``;
  - effective response lag: expected entropy reduction of the grid posterior (sysid/online.py);
  - transient-vs-persistent disturbance (hover): ``H_b(0.5) (1 - exp(-T / tau_c))`` with the channel clear
    time ``tau_c`` (a documented HEURISTIC, the only non-model-based term);
* energy cost ``dE(a) = P_pred T (1 + 3/8 delta^2)`` (``delta`` = relative thrust excursion; momentum theory
  ``P ~ T^1.5`` averaged over a sinusoid);
* utility ``U = IG - lambda_E dE[kJ] - lambda_T T`` (``lambda_E = 0`` when energy awareness is disabled).

HARD constraints (any violation rejects the candidate, independent of its utility):
facade clearance with vehicle radius and 3-sigma position uncertainty; minimum altitude; along-facade
excursion inside the facade extent; peak speed/acceleration limits; thrust margin for vertical
excitation; energy after the probe still covers safe termination plus reserve; allowed mission phase;
cooldown and maximum probe count. A probe is executed only if ``IG >= ig_min`` and ``U > 0``; otherwise
the rejection and its reasons are logged.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from graduation_project.active_diagnosis.probes import Probe
from graduation_project.diagnosis.diagnoser import DiagnosisReport
from graduation_project.energy.manager import EnergyManager
from graduation_project.sysid.online import OnlineIdentifier


@dataclass(frozen=True)
class ActiveConfig:
    ig_min: float = 0.25  # [nats]
    lambda_energy: float = 0.02  # [nats per kJ]
    lambda_time: float = 0.01  # [nats per s]
    cooldown: float = 12.0  # [s]
    max_probes: int = 3
    clearance_margin: float = 0.5  # [m] beyond vehicle radius and 3 sigma
    z_min: float = 2.0  # [m]
    max_probe_speed: float = 2.0  # [m/s]
    max_probe_accel: float = 2.0  # [m/s^2]
    max_hover_throttle: float = 0.70  # thrust margin for vertical excitation
    max_peak_throttle: float = 0.95  # hover throttle x (1 + relative excursion) during vertical excitation
    # amplitude [m], frequency [Hz], duration [s]; chosen so the conservative peak bounds of probes.py stay
    # inside max_probe_speed / max_probe_accel (vertical: 1.75 m/s^2; speed: 1.17 m/s, 1.72 m/s^2)
    vertical: tuple[float, float, float] = (0.10, 0.5, 6.0)
    speed: tuple[float, float, float] = (0.8, 0.15, 12.0)
    hover_duration: float = 5.0
    channel_clear_s: float = 3.0


RELEVANCE = {
    "vertical": {"response_delay": 1.0, "mass_or_thrust_change": 0.6, "actuator_degradation_or_com_shift": 0.4,
                 "unexplained": 0.6},
    "speed": {"steady_wind": 1.0, "drag_change": 1.0, "gnss_delay": 1.0, "sensor_inconsistency": 0.5, "unexplained": 0.4},
    "hover": {"wind_gust": 1.0, "steady_wind": 0.3, "unexplained": 0.3},
}


@dataclass(frozen=True)
class CandidateEvaluation:
    kind: str
    ig: float
    energy_j: float
    utility: float
    feasible: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ActiveDecision:
    t: float
    accepted: bool
    probe: Probe | None
    chosen: CandidateEvaluation | None
    candidates: tuple[CandidateEvaluation, ...]
    reason: str


@dataclass
class ActiveDiagnosisPlanner:
    mass_nominal: float
    gravity: float
    thrust_max_total: float  # nominal [N]
    vehicle_radius: float  # [m]
    cfg: ActiveConfig = field(default_factory=ActiveConfig)
    energy_aware: bool = True

    def __post_init__(self) -> None:
        self.n_probes = 0
        self.last_probe_t = -np.inf

    def _probe(self, kind: str, tangent: np.ndarray) -> Probe:
        if kind == "vertical":
            A, f, T = self.cfg.vertical
            return Probe("vertical", A, f, T, np.array([0.0, 0.0, 1.0]))
        if kind == "speed":
            A, f, T = self.cfg.speed
            return Probe("speed", A, f, T, np.asarray(tangent, dtype=float))
        return Probe("hover", 0.0, 0.0, self.cfg.hover_duration, np.zeros(3))

    def _ig(self, probe: Probe, report: DiagnosisReport, sysid: OnlineIdentifier, v_now: np.ndarray) -> float:
        rel = RELEVANCE[probe.kind]
        w = sum(report.probabilities.get(h, 0.0) * r for h, r in rel.items())
        if w <= 0.0:
            return 0.0
        dt = 1.0 / sysid.cfg.rate
        if probe.kind == "vertical":
            u = probe.input_profile(dt, self.mass_nominal, self.gravity)
            ig = sysid.lag.expected_information_gain(u, float(sysid.gain.theta[0]))
            ig += sysid.gain.information_gain(u[:, None])
        elif probe.kind == "speed":
            taus = np.arange(0.0, probe.duration, dt)
            ax = int(np.argmax(np.abs(probe.axis[:2])))
            v = np.array([v_now[ax] + probe.offset(t)[1][ax] for t in taus])
            ig = sysid.drag[ax].information_gain(v)
            v10 = v[:: max(1, int(round(0.1 / dt)))]
            ig += sysid.delay.information_gain(-v10[:, None])
        else:
            ig = np.log(2.0) * (1.0 - np.exp(-probe.duration / self.cfg.channel_clear_s))
        return float(w * ig)

    def evaluate(self, t: float, report: DiagnosisReport, sysid: OnlineIdentifier, energy: EnergyManager,
                 position: np.ndarray, velocity: np.ndarray, position_std: np.ndarray, phase: str, facade,
                 exit_time: float, thrust_gain: float) -> ActiveDecision:
        cands: list[CandidateEvaluation] = []
        common: list[str] = []
        if phase not in ("coverage", "transit", "hold"):
            common.append(f"phase '{phase}' does not allow probes")
        if t - self.last_probe_t < self.cfg.cooldown:
            common.append("cooldown")
        if self.n_probes >= self.cfg.max_probes:
            common.append("probe budget exhausted")
        sigma = float(np.max(position_std))
        d_facade = facade.distance(position)
        s_along = float((position - facade.origin) @ facade.tangent)
        p_hover = energy.predicted_power(thrust_gain)
        hover_throttle = self.mass_nominal * self.gravity / max(thrust_gain, 1e-3) / self.thrust_max_total
        best: CandidateEvaluation | None = None
        for kind in ("vertical", "speed", "hover"):
            pr = self._probe(kind, facade.tangent)
            reasons = list(common)
            delta = pr.peak_acceleration / self.gravity if kind == "vertical" else 0.05
            e_cost = p_hover * pr.duration * (1.0 + 0.375 * delta**2)
            if d_facade - self.vehicle_radius - 3.0 * sigma < self.cfg.clearance_margin:
                reasons.append("facade clearance")
            if kind == "vertical" and position[2] - pr.amplitude - 3.0 * sigma < self.cfg.z_min:
                reasons.append("minimum altitude")
            if kind == "speed" and not (0.5 <= s_along - pr.amplitude and s_along + pr.amplitude <= facade.width - 0.5):
                reasons.append("along-facade excursion outside facade extent")
            if pr.peak_velocity > self.cfg.max_probe_speed:
                reasons.append("peak speed limit")
            if pr.peak_acceleration > self.cfg.max_probe_accel:
                reasons.append("peak acceleration limit")
            if kind == "vertical" and (hover_throttle > self.cfg.max_hover_throttle
                                       or hover_throttle * (1.0 + delta) > self.cfg.max_peak_throttle):
                reasons.append("insufficient thrust margin")
            if energy.margin_after(pr.duration + exit_time, thrust_gain) < 0.0:
                reasons.append("energy after probe would not cover safe termination + reserve")
            ig = self._ig(pr, report, sysid, velocity)
            lam_e = self.cfg.lambda_energy if self.energy_aware else 0.0
            util = ig - lam_e * e_cost / 1000.0 - self.cfg.lambda_time * pr.duration
            if ig < self.cfg.ig_min:
                reasons.append(f"information gain {ig:.2f} < {self.cfg.ig_min}")
            if util <= 0.0:
                reasons.append("utility <= 0")
            ev = CandidateEvaluation(kind, ig, e_cost, util, not reasons, tuple(reasons))
            cands.append(ev)
            if ev.feasible and (best is None or ev.utility > best.utility):
                best = ev
        if best is None:
            return ActiveDecision(t, False, None, None, tuple(cands), "no admissible probe: " +
                                  "; ".join(f"{c.kind}: {', '.join(c.reasons)}" for c in cands))
        self.n_probes += 1
        self.last_probe_t = t
        return ActiveDecision(t, True, self._probe(best.kind, facade.tangent), best, tuple(cands),
                              f"selected {best.kind}: IG {best.ig:.2f} nats, {best.energy_j / 1000:.1f} kJ, U {best.utility:.2f}")
