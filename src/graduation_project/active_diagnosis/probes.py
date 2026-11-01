"""Bounded diagnostic manoeuvres, applied as smooth offsets to a stationary (dwell) reference.

Offset ``dp(tau) = A w(tau) sin(omega tau) * axis`` with the window ``w = sin^2(pi tau / T)`` for
``0 <= tau <= T``. Position, velocity and acceleration are continuous at both ends and zero there, so a
probe can be inserted and removed without a reference jump. Peak magnitudes (used by the safety
checks) are bounded by ``|dp| <= A``, ``|dv| <= A (omega + 2 pi / T)``,
``|da| <= A (omega + 2 pi / T)^2``.

* ``vertical``  thrust excitation (identifies thrust gain and effective response lag);
* ``speed``     along-facade velocity excitation (identifies drag vs. steady wind, GNSS latency);
* ``hover``     zero offset: wait and observe (separates transient from persistent disturbances).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from graduation_project.dynamics.frames import FloatArray

PROBE_KINDS = ("vertical", "speed", "hover")


@dataclass(frozen=True)
class Probe:
    kind: str
    amplitude: float  # [m]
    frequency: float  # [Hz]
    duration: float  # [s]
    axis: FloatArray  # unit vector (world)

    def __post_init__(self) -> None:
        if self.kind not in PROBE_KINDS or self.duration <= 0 or self.amplitude < 0:
            raise ValueError("invalid probe")

    @property
    def peak_velocity(self) -> float:
        return self.amplitude * (2 * np.pi * self.frequency + 2 * np.pi / self.duration)

    @property
    def peak_acceleration(self) -> float:
        return self.amplitude * (2 * np.pi * self.frequency + 2 * np.pi / self.duration) ** 2

    def offset(self, tau: float) -> tuple[FloatArray, FloatArray, FloatArray]:
        z = np.zeros(3)
        if self.kind == "hover" or not 0.0 <= tau <= self.duration:
            return z, z, z
        T, A, w = self.duration, self.amplitude, 2 * np.pi * self.frequency
        c = np.pi / T
        win, dwin, ddwin = np.sin(c * tau) ** 2, c * np.sin(2 * c * tau), 2 * c * c * np.cos(2 * c * tau)
        s, ds, dds = np.sin(w * tau), w * np.cos(w * tau), -w * w * np.sin(w * tau)
        ax = np.asarray(self.axis, dtype=float)
        return A * win * s * ax, A * (dwin * s + win * ds) * ax, A * (ddwin * s + 2 * dwin * ds + win * dds) * ax

    def input_profile(self, dt: float, mass: float, gravity: float) -> FloatArray:
        """Collective specific-force command implied by the probe (vertical axis), one value per ``dt``."""
        taus = np.arange(0.0, self.duration, dt)
        return np.array([gravity + self.offset(t)[2][2] for t in taus])
