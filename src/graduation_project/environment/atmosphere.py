"""International Standard Atmosphere (troposphere, ISO 2533) with a temperature offset.

For geopotential altitude ``h`` below 11 km::

    T_std(h) = T0 - L h
    p(h)     = p0 (T_std(h) / T0) ^ (g0 / (R L))
    T(h)     = T_std(h) + delta_T            (hot/cold day; pressure follows the standard profile)
    rho(h)   = p(h) / (R T(h))

Rotor thrust and torque coefficients scale with ``rho / rho_ref`` (momentum/blade-element theory at
constant rotor speed); the barometer model uses ``p(h)``.
"""

from __future__ import annotations

from dataclasses import dataclass

T0 = 288.15  # [K]
P0 = 101325.0  # [Pa]
LAPSE = 0.0065  # [K/m]
R_AIR = 287.05287  # [J/(kg K)]
G0 = 9.80665  # [m/s^2]
RHO0 = P0 / (R_AIR * T0)  # 1.2250 kg/m^3
_EXP = G0 / (R_AIR * LAPSE)  # 5.25588


@dataclass(frozen=True)
class AtmosphereState:
    temperature: float  # [K]
    pressure: float  # [Pa]
    density: float  # [kg/m^3]


def isa(altitude_m: float, delta_t: float = 0.0) -> AtmosphereState:
    if not -500.0 <= altitude_m <= 11000.0:
        raise ValueError("ISA troposphere model valid for -500 m <= h <= 11 km")
    t_std = T0 - LAPSE * altitude_m
    p = P0 * (t_std / T0) ** _EXP
    t = t_std + delta_t
    if t <= 0.0:
        raise ValueError("non-physical temperature")
    return AtmosphereState(t, p, p / (R_AIR * t))


def pressure_altitude(pressure: float) -> float:
    """Inverse of the standard pressure profile (what a barometer reports) [m]."""
    return (T0 / LAPSE) * (1.0 - (pressure / P0) ** (1.0 / _EXP))


@dataclass(frozen=True)
class Site:
    """Operating site: ground altitude above mean sea level and temperature offset from ISA."""

    altitude_msl: float = 0.0  # [m]
    delta_t: float = 0.0  # [K]

    def at(self, height_above_ground: float) -> AtmosphereState:
        return isa(self.altitude_msl + max(height_above_ground, -100.0), self.delta_t)

    @property
    def density_ratio(self) -> float:
        """Ground-level density relative to ISA sea level (rotor coefficients are referenced to 1.225)."""
        return self.at(0.0).density / RHO0
