"""Mission scenarios (config/scenarios/*.yaml) with seeded, reproducible per-run variation."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from graduation_project.config import CONFIG_DIR, load_yaml
from graduation_project.environment.atmosphere import Site
from graduation_project.environment.wind import CompositeWind, DiscreteGust, DrydenTurbulence, MeanWindProfile, WindField
from graduation_project.sensors.models import SensorFault

SCENARIO_DIR = CONFIG_DIR / "scenarios"


@dataclass(frozen=True)
class TruthEvent:
    t: float
    type: str  # mass_scale | drag_scale | command_delay | rotor_effectiveness
    value: float
    rotor: int | None = None


@dataclass(frozen=True)
class MissionScenario:
    name: str
    description: str
    fault_class: str
    expected_behaviour: str
    seed: int
    raw: dict[str, Any]
    truth_events: tuple[TruthEvent, ...] = ()
    sensor_faults: tuple[SensorFault, ...] = ()
    soc0: float = 1.0
    capacity_scale: float = 1.0
    r0_scale: float = 1.0
    site: Site = field(default_factory=Site)
    facade_variant: str = "complex"
    duration_max: float = 260.0

    @property
    def fault_onset(self) -> float | None:
        times = [e.t for e in self.truth_events] + [f.t_start for f in self.sensor_faults]
        gusts = (self.raw.get("wind") or {}).get("_gust_times", [])
        times += list(gusts)
        return min(times) if times else None

    def wind(self, dt: float) -> WindField:
        w = self.raw.get("wind") or {}
        comps: list[WindField] = []
        if "mean" in w:
            comps.append(MeanWindProfile(**w["mean"]))
        if "turbulence" in w:
            mean = w.get("mean", {})
            comps.append(DrydenTurbulence.low_altitude(float(w["turbulence"]["w20"]), 6.0, float(mean.get("direction_deg", 0.0)),
                                                       seed=self.seed + 7919, dt=dt))
        for g in w.get("_gusts", []):
            comps.append(DiscreteGust(g["t_start"], g["duration"], np.asarray(g["amplitude"], dtype=float)))
        return CompositeWind(comps)


def list_scenarios() -> list[str]:
    return sorted(p.stem for p in SCENARIO_DIR.glob("*.yaml"))


def load_scenario(name: str, seed: int, path: Path | None = None, randomize: bool = True) -> MissionScenario:
    raw = load_yaml(path or SCENARIO_DIR / f"{name}.yaml")
    rng = np.random.default_rng(np.random.SeedSequence([seed, sum(map(ord, raw["name"]))]))
    rz = raw.get("randomize") or {}
    tj = float(rz.get("t_jitter", 0.0)) if randomize else 0.0
    mj = float(rz.get("mag_jitter", 0.0)) if randomize else 0.0
    shift = float(rng.uniform(-tj, tj)) if tj > 0 else 0.0  # one common onset shift keeps staggering
    mag = lambda v: float(v) * (1.0 + (float(rng.uniform(-mj, mj)) if mj > 0 else 0.0))  # noqa: E731

    events = []
    for e in raw.get("truth_events") or []:
        v = float(e["value"])
        if e["type"] in ("mass_scale", "drag_scale"):
            v = 1.0 + (mag(v - 1.0) if mj > 0 else v - 1.0)
        elif e["type"] == "rotor_effectiveness":
            v = 1.0 - mag(1.0 - v)
        else:
            v = mag(v)
        events.append(TruthEvent(float(e["t"]) + shift, e["type"], v, e.get("rotor")))
    faults = []
    for f in raw.get("sensor_faults") or []:
        m = f.get("magnitude", 0.0)
        if isinstance(m, (int, float)) and f["kind"] in ("noise", "delay", "invalid"):
            m = mag(m)
        t_end = float(f.get("t_end", np.inf))
        faults.append(SensorFault(f["sensor"], f["kind"], float(f["t_start"]) + shift,
                                  t_end + (shift if t_end < 1e8 else 0.0),
                                  m, float(f.get("probability", 1.0))))
    wind = dict(raw.get("wind") or {})
    gusts = []
    for g in wind.get("gusts", []) or []:
        amp = np.asarray(g["amplitude"], dtype=float)
        gusts.append({"t_start": float(g["t_start"]) + shift, "duration": float(g["duration"]),
                      "amplitude": (amp * (1.0 + (float(rng.uniform(-mj, mj)) if mj > 0 else 0.0))).tolist()})
    wind["_gusts"] = gusts
    if raw.get("fault_class") == "wind_gust":
        wind["_gust_times"] = [g["t_start"] for g in gusts]
    raw = {**raw, "wind": wind}
    b = raw.get("battery") or {}
    s = raw.get("site") or {}
    return MissionScenario(
        name=raw["name"], description=raw["description"], fault_class=raw.get("fault_class", "none"),
        expected_behaviour=raw.get("expected_behaviour", ""), seed=seed, raw=raw, truth_events=tuple(events),
        sensor_faults=tuple(faults), soc0=float(b.get("soc0", 1.0)), capacity_scale=float(b.get("capacity_scale", 1.0)),
        r0_scale=float(b.get("r0_scale", 1.0)), site=Site(float(s.get("altitude_msl", 0.0)), float(s.get("delta_t", 0.0))),
        facade_variant=raw.get("facade_variant", "complex"), duration_max=float(raw.get("duration_max", 260.0)),
    )
