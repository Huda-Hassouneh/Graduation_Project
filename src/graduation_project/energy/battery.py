"""Equivalent-circuit battery model (Thevenin, one RC branch) for the propulsion supply.

    V_t   = OCV(SoC) - I R0 - V1
    dV1/dt = I / C1 - V1 / (R1 C1)
    dSoC/dt = -I / (3600 Q)          (Q in Ah, coulomb counting)

The pack OCV is ``n_series * OCV_cell(SoC)`` interpolated from a tabulated curve. For a requested
electrical power ``P`` the current solves ``P = I (OCV - V1 - I R0)``:

    I = ((OCV - V1) - sqrt((OCV - V1)^2 - 4 R0 P)) / (2 R0)

(the smaller root, i.e. the physically operating branch). If ``P`` exceeds the maximum deliverable
power ``(OCV - V1)^2 / (4 R0)`` the battery delivers that maximum and reports ``power_limited``.

This is an EQUIVALENT model: DJI publishes TB100 capacity, energy, voltages and chemistry but not
internal resistance, cell format or the OCV curve (config/aircraft/dji_matrice_400.yaml).
Temperature effects and capacity fade are represented only through ``capacity_scale`` and
``r0_scale`` (aged-battery scenarios).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from graduation_project.dynamics.frames import FloatArray


@dataclass(frozen=True)
class BatteryParams:
    n_series: int
    capacity_ah: float
    ocv_soc: FloatArray  # increasing SoC breakpoints [-]
    ocv_cell: FloatArray  # cell open-circuit voltage at the breakpoints [V]
    r0: float  # [ohm]
    r1: float  # [ohm]
    c1: float  # [F]
    cell_cutoff: float  # [V]
    nominal_voltage: float  # [V] (pack, published)
    max_discharge_current: float  # [A]

    def __post_init__(self) -> None:
        soc = np.asarray(self.ocv_soc, dtype=float)
        v = np.asarray(self.ocv_cell, dtype=float)
        if soc.shape != v.shape or soc.ndim != 1 or np.any(np.diff(soc) <= 0) or np.any(np.diff(v) < 0):
            raise ValueError("OCV table must be increasing in SoC and non-decreasing in voltage")
        if min(self.r0, self.r1, self.c1, self.capacity_ah) <= 0 or self.n_series < 1:
            raise ValueError("battery parameters must be positive")
        object.__setattr__(self, "ocv_soc", soc)
        object.__setattr__(self, "ocv_cell", v)

    @classmethod
    def from_spec(cls, spec: dict[str, Any]) -> "BatteryParams":
        from graduation_project.aircraft_spec import sval

        cap = float(sval(spec, "battery.capacity"))
        return cls(
            n_series=int(sval(spec, "battery.series_cells")),
            capacity_ah=cap,
            ocv_soc=np.array(sval(spec, "simulation_assumptions.battery_ocv_curve.soc"), dtype=float),
            ocv_cell=np.array(sval(spec, "simulation_assumptions.battery_ocv_curve.cell_voltage"), dtype=float),
            r0=float(sval(spec, "simulation_assumptions.battery_r0")),
            r1=float(sval(spec, "simulation_assumptions.battery_r1")),
            c1=float(sval(spec, "simulation_assumptions.battery_c1")),
            cell_cutoff=float(sval(spec, "simulation_assumptions.battery_cell_cutoff")),
            nominal_voltage=float(sval(spec, "battery.nominal_voltage")),
            max_discharge_current=float(sval(spec, "battery.max_discharge_rate")) * cap,
        )

    def ocv(self, soc: float) -> float:
        return self.n_series * float(np.interp(soc, self.ocv_soc, self.ocv_cell))

    @property
    def nominal_energy_wh(self) -> float:
        """Integral of the OCV curve over SoC times capacity (consistency check vs the published Wh)."""
        s = np.linspace(0.0, 1.0, 2001)
        return float(np.trapz([self.ocv(x) for x in s], s) * self.capacity_ah)


@dataclass
class BatteryState:
    soc: float
    v1: float = 0.0  # polarisation voltage [V]
    current: float = 0.0  # [A]
    voltage: float = 0.0  # terminal voltage [V]
    energy_out_j: float = 0.0  # electrical energy delivered [J]
    power_limited: bool = False
    depleted: bool = False


class Battery:
    def __init__(self, params: BatteryParams, soc0: float = 1.0, capacity_scale: float = 1.0, r0_scale: float = 1.0):
        if not 0.0 <= soc0 <= 1.0:
            raise ValueError("initial SoC must be in [0, 1]")
        self.p = params
        self.capacity_ah = params.capacity_ah * capacity_scale
        self.r0 = params.r0 * r0_scale
        self.state = BatteryState(soc=soc0, voltage=params.ocv(soc0))

    def current_for_power(self, power: float) -> tuple[float, bool]:
        e = self.p.ocv(self.state.soc) - self.state.v1
        p_max = e * e / (4.0 * self.r0)
        if power >= p_max:
            return e / (2.0 * self.r0), True
        if power <= 0.0:
            return 0.0, False
        return (e - np.sqrt(e * e - 4.0 * self.r0 * power)) / (2.0 * self.r0), False

    def step(self, power: float, dt: float) -> BatteryState:
        s = self.state
        i, limited = self.current_for_power(power)
        tau = self.p.r1 * self.p.c1
        decay = np.exp(-dt / tau)
        v_term = self.p.ocv(s.soc) - i * self.r0 - s.v1
        s.v1 = s.v1 * decay + i * self.p.r1 * (1.0 - decay)  # exact for constant current over dt
        s.soc = max(0.0, s.soc - i * dt / (3600.0 * self.capacity_ah))
        s.current, s.voltage, s.power_limited = i, v_term, limited
        s.energy_out_j += v_term * i * dt
        s.depleted = s.soc <= 0.0 or v_term < self.p.n_series * self.p.cell_cutoff
        return s

    @property
    def voltage_ratio(self) -> float:
        """Terminal voltage relative to the published nominal voltage (thrust-headroom scaling)."""
        return self.state.voltage / self.p.nominal_voltage
