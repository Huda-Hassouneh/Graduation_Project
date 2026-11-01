"""Energy state, prediction and accounting for decision-making (framework side; telemetry only).

Inputs: battery telemetry ``[V, I, SoC]`` (available on DJI aircraft as fused battery state; exact
M400 SDK fields and rates to be confirmed, docs/DJI_MATRICE_400_INTEGRATION_PLAN.md) and the mission
tag of the current segment. Nothing here reads the simulated battery directly.

Quantities kept SEPARATE (they are often confused):

* ``measured electrical energy`` = integral of telemetry ``V * I`` (what left the battery);
* ``remaining usable energy`` = min of (a) the energy implied by the reported SoC through the NOMINAL
  OCV curve and capacity, down to the usable floor, and (b) the initial estimate minus the measured
  consumption. Taking the minimum is conservative when the real battery is aged (capacity lower than
  nominal), which the framework cannot know a priori;
* ``predicted power`` = max(measured average power, nominal hover power / theta^1.5) where theta is the
  identified thrust gain (momentum theory: hover power ~ T^1.5 at fixed rotor geometry);
* ``energy for safe termination`` = predicted power x duration of the safe exit route x margin;
* motor/propulsion energy vs. avionics energy are not separable from pack telemetry (single bus);
  onboard computation is inside the avionics term of the simulated plant.

Accounting buckets: ``nominal``, ``diagnostic`` (probe dwell), ``recovery`` (replanned / exit segments
created by a recovery decision), ``termination`` (after an abort or safe-landing decision).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from graduation_project.energy.battery import BatteryParams

BUCKETS = ("nominal", "diagnostic", "recovery", "termination")


@dataclass(frozen=True)
class EnergyConfig:
    usable_fraction: float = 0.95  # of the nominal OCV-integral energy
    reserve_fraction: float = 0.05  # reserve kept at mission end (of usable energy at full charge)
    hard_reserve_soc: float = 0.10  # baseline low-battery failsafe threshold (reported SoC)
    prediction_margin: float = 1.15  # multiplicative margin on predicted energy needs
    power_time_constant: float = 8.0  # [s] EMA of measured power


@dataclass
class EnergyManager:
    battery: BatteryParams  # NOMINAL parameters (published capacity, assumed OCV curve)
    hover_power_nominal: float  # [W] from the nominal model at the configured mass
    cfg: EnergyConfig = field(default_factory=EnergyConfig)

    def __post_init__(self) -> None:
        self._soc_grid = np.linspace(0.0, 1.0, 401)
        e = np.array([self.battery.ocv(s) for s in self._soc_grid]) * self.battery.capacity_ah * 3600.0
        self._e_cum = np.concatenate(([0.0], np.cumsum(0.5 * (e[1:] + e[:-1]) * np.diff(self._soc_grid))))  # [J] from SoC 0
        self.soc_floor = 1.0 - self.cfg.usable_fraction
        self.e_full_usable = self._energy_between(self.soc_floor, 1.0)
        self.reserve_j = self.cfg.reserve_fraction * self.e_full_usable
        self.buckets = dict.fromkeys(BUCKETS, 0.0)
        self.consumed_j = 0.0
        self.power_avg: float | None = None
        self.soc: float | None = None
        self.voltage: float | None = None
        self._e0: float | None = None
        self._t: float | None = None

    def _energy_between(self, s0: float, s1: float) -> float:
        return float(np.interp(s1, self._soc_grid, self._e_cum) - np.interp(s0, self._soc_grid, self._e_cum))

    def update(self, t: float, telemetry, tag: str) -> None:
        if telemetry is None or not np.all(np.isfinite(telemetry)):
            self._t = t
            return
        v, i, soc = (float(x) for x in telemetry)
        p = max(v * i, 0.0)
        if self._t is not None and t > self._t:
            dt = t - self._t
            self.consumed_j += p * dt
            self.buckets[tag if tag in self.buckets else "nominal"] += p * dt
            a = 1.0 - np.exp(-dt / self.cfg.power_time_constant)
            self.power_avg = p if self.power_avg is None else self.power_avg + a * (p - self.power_avg)
        self._t = t
        self.soc, self.voltage = soc, v
        if self._e0 is None:
            self._e0 = self._energy_between(self.soc_floor, max(soc, self.soc_floor))

    # ------------------------------------------------------------------ estimates
    def remaining_j(self) -> float:
        if self.soc is None or self._e0 is None:
            return float("nan")
        by_soc = self._energy_between(self.soc_floor, max(self.soc, self.soc_floor))
        by_count = self._e0 - self.consumed_j
        return max(0.0, min(by_soc, by_count))

    def predicted_power(self, thrust_gain: float = 1.0) -> float:
        model = self.hover_power_nominal / max(thrust_gain, 0.3) ** 1.5
        return max(self.power_avg or 0.0, model)

    def energy_for(self, duration_s: float, thrust_gain: float = 1.0) -> float:
        return self.predicted_power(thrust_gain) * duration_s * self.cfg.prediction_margin

    def margin_after(self, duration_s: float, thrust_gain: float = 1.0) -> float:
        """Energy left above the reserve after flying ``duration_s`` more seconds [J] (negative = infeasible)."""
        return self.remaining_j() - self.energy_for(duration_s, thrust_gain) - self.reserve_j

    def affordable_time(self, thrust_gain: float = 1.0) -> float:
        """Flight time that keeps the reserve [s]."""
        return max(0.0, (self.remaining_j() - self.reserve_j) / (self.predicted_power(thrust_gain) * self.cfg.prediction_margin))

    def below_hard_reserve(self) -> bool:
        return self.soc is not None and self.soc <= self.cfg.hard_reserve_soc
