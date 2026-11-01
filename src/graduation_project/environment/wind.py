"""Wind fields in the world ENU frame [m/s]: the velocity *of the air* (wind toward +x east is ``[+u, 0, 0]``).

Available components (summed by :class:`CompositeWind`):

* :class:`ConstantWind`  uniform steady wind.
* :class:`MeanWindProfile`  power-law shear ``U(z) = U_ref (z / z_ref)^alpha`` with a direction that may
  rotate linearly in time (changing wind direction).
* :class:`DrydenTurbulence`  seeded turbulence. Each axis (along-wind u, cross-wind v, vertical w) is a
  first-order Gauss-Markov process ``x_{k+1} = a x_k + sigma sqrt(1 - a^2) n_k`` with
  ``a = exp(-V dt / L)``. This reproduces the Dryden u-component spectrum exactly and approximates the
  second-order v/w Dryden forms with the same variance and integral length scale (documented
  simplification). :meth:`DrydenTurbulence.low_altitude` uses the MIL-F-8785C low-altitude intensity
  and scale relations (h, L in feet internally). The advection speed V is the mean wind speed (frozen
  turbulence convected past a vehicle that is near hover), floored at ``v_min``.
* :class:`DiscreteGust`  "1-cosine" gust ``w = A/2 (1 - cos(2 pi (t - t0)/T))`` for ``t0 <= t <= t0 + T``.

All stochastic components draw from their own ``numpy.random.Generator`` seeded at construction, so a
given seed reproduces the same field. Turbulence is advanced in fixed steps ``dt``; querying a time
that was already passed returns the stored value.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, Sequence

import numpy as np

from graduation_project.dynamics.frames import FloatArray

FT = 0.3048  # [m/ft]


class WindField(Protocol):
    def velocity(self, t: float, position_w: FloatArray) -> FloatArray: ...


@dataclass(frozen=True)
class ConstantWind:
    vector_w: FloatArray

    def velocity(self, t: float, position_w: FloatArray) -> FloatArray:
        return np.asarray(self.vector_w, dtype=float)


NO_WIND = ConstantWind(np.zeros(3))


def _direction(deg: float) -> FloatArray:
    a = np.deg2rad(deg)
    return np.array([np.cos(a), np.sin(a), 0.0])


@dataclass(frozen=True)
class MeanWindProfile:
    """Horizontal mean wind; ``direction_deg`` is where the air goes, CCW from +x (east)."""

    speed_ref: float  # [m/s] at z_ref
    direction_deg: float
    z_ref: float = 10.0  # [m]
    alpha: float = 1.0 / 7.0  # power-law shear exponent (open terrain ~0.14, urban ~0.3)
    z_min: float = 0.5  # [m] below this height the profile is held constant
    direction_rate_deg_s: float = 0.0  # linear rotation of the wind direction [deg/s]
    t_rotation_start: float = 0.0
    t_rotation_end: float = np.inf

    def direction_at(self, t: float) -> float:
        dt = np.clip(t, self.t_rotation_start, self.t_rotation_end) - self.t_rotation_start
        return self.direction_deg + self.direction_rate_deg_s * dt

    def speed_at(self, z: float) -> float:
        return self.speed_ref * (max(z, self.z_min) / self.z_ref) ** self.alpha

    def velocity(self, t: float, position_w: FloatArray) -> FloatArray:
        return self.speed_at(float(position_w[2])) * _direction(self.direction_at(t))


@dataclass
class DrydenTurbulence:
    sigma: FloatArray  # (3,) std of [u, v, w] [m/s]
    length: FloatArray  # (3,) integral length scales [m]
    mean_speed: float  # advection speed [m/s]
    direction_deg: float  # along-wind axis (direction the mean wind blows toward)
    seed: int
    dt: float = 0.004
    v_min: float = 1.0
    _state: FloatArray = field(init=False, repr=False)
    _t: float = field(init=False, repr=False)
    _rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.sigma = np.asarray(self.sigma, dtype=float)
        self.length = np.asarray(self.length, dtype=float)
        if self.sigma.shape != (3,) or self.length.shape != (3,) or np.any(self.length <= 0) or np.any(self.sigma < 0):
            raise ValueError("sigma and length must be non-negative / positive 3-vectors")
        self._rng = np.random.default_rng(self.seed)
        self._state = self.sigma * self._rng.standard_normal(3)  # stationary initial condition
        self._t = 0.0
        v = max(self.mean_speed, self.v_min)
        self._a = np.exp(-v * self.dt / self.length)
        self._b = self.sigma * np.sqrt(1.0 - self._a**2)
        a = np.deg2rad(self.direction_deg)
        self._R = np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])

    @classmethod
    def low_altitude(cls, w20: float, height: float, direction_deg: float, seed: int, dt: float = 0.004,
                     mean_speed: float | None = None) -> "DrydenTurbulence":
        """MIL-F-8785C low-altitude Dryden parameters; ``w20`` is the wind speed at 20 ft (6.1 m) [m/s]."""
        h = max(height, 3.0) / FT  # [ft]; the relations are not defined at the ground
        sigma_w = 0.1 * w20
        sigma_uv = sigma_w / (0.177 + 0.000823 * h) ** 0.4
        l_uv = h / (0.177 + 0.000823 * h) ** 1.2 * FT
        l_w = h * FT
        return cls(np.array([sigma_uv, sigma_uv, sigma_w]), np.array([l_uv, l_uv, l_w]),
                   w20 if mean_speed is None else mean_speed, direction_deg, seed, dt)

    def _advance_to(self, t: float) -> None:
        n = int(np.floor((t - self._t) / self.dt + 1e-9))
        for _ in range(max(n, 0)):
            self._state = self._a * self._state + self._b * self._rng.standard_normal(3)
        self._t += max(n, 0) * self.dt

    def velocity(self, t: float, position_w: FloatArray) -> FloatArray:
        self._advance_to(t)
        return self._R @ self._state


@dataclass(frozen=True)
class DiscreteGust:
    t_start: float  # [s]
    duration: float  # [s]
    amplitude_w: FloatArray  # peak gust velocity vector [m/s]

    def velocity(self, t: float, position_w: FloatArray) -> FloatArray:
        s = t - self.t_start
        if s < 0.0 or s > self.duration:
            return np.zeros(3)
        return 0.5 * (1.0 - np.cos(2.0 * np.pi * s / self.duration)) * np.asarray(self.amplitude_w, dtype=float)


@dataclass
class CompositeWind:
    components: Sequence[WindField]

    def velocity(self, t: float, position_w: FloatArray) -> FloatArray:
        out = np.zeros(3)
        for c in self.components:
            out = out + c.velocity(t, position_w)
        return out
