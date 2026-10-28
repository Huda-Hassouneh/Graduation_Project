"""Propulsion power and energy accounting (docs/energy_model.md).

* Shaft (mechanical) power of rotor i: ``P_shaft,i = Q_i * Omega_i = eta_i k_m Omega_i^3`` [W].
* Electrical power: ``P_el = sum_i P_shaft,i / efficiency + P_avionics`` [W], where
  ``efficiency`` lumps motor + ESC losses (assumed constant; a known simplification).
* Ideal induced power from momentum theory, ``T^(3/2) / sqrt(2 rho A)``, is provided only as
  a sanity check (figure of merit); it is not used to integrate energy.

Motor acceleration power and battery voltage sag are not modelled (see docs/limitations.md).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from graduation_project.dynamics.frames import FloatArray


@dataclass(frozen=True)
class PowerParams:
    propulsion_efficiency: float  # electrical-to-shaft efficiency of motor + ESC [-]
    avionics_power: float  # constant non-propulsive electrical load [W]
    battery_energy: float  # nominal battery energy [J]

    def __post_init__(self) -> None:
        if not 0.0 < self.propulsion_efficiency <= 1.0:
            raise ValueError("propulsion_efficiency must be in (0, 1]")
        if self.avionics_power < 0 or self.battery_energy <= 0:
            raise ValueError("avionics_power must be >= 0 and battery_energy > 0")


def shaft_power(omega: FloatArray, k_m: float, effectiveness: FloatArray) -> float:
    return float(np.sum(effectiveness * k_m * np.abs(omega) ** 3))


def electrical_power(shaft: float, params: PowerParams) -> float:
    return shaft / params.propulsion_efficiency + params.avionics_power


def ideal_induced_power(thrust: float, air_density: float, rotor_diameter: float) -> float:
    """Momentum-theory ideal hover power of a single rotor [W]."""
    area = np.pi * (0.5 * rotor_diameter) ** 2
    return float(thrust**1.5 / np.sqrt(2.0 * air_density * area))


class EnergyIntegrator:
    """Trapezoidal integration of ``E = integral P dt`` [J]."""

    def __init__(self) -> None:
        self.energy = 0.0
        self._last: tuple[float, float] | None = None

    def update(self, t: float, power: float) -> float:
        if self._last is not None:
            t0, p0 = self._last
            if t < t0:
                raise ValueError("time must be non-decreasing")
            self.energy += 0.5 * (p0 + power) * (t - t0)
        self._last = (t, power)
        return self.energy
