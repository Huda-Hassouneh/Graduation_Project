"""Wind-field interface. Milestone 1 provides only constant wind; richer profiles come in Milestone 4.

Wind velocity is expressed in the world ENU frame [m/s] and is the velocity *of the air*
(a wind blowing toward +x east has ``w = [+u, 0, 0]``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from graduation_project.dynamics.frames import FloatArray


class WindField(Protocol):
    def velocity(self, t: float, position_w: FloatArray) -> FloatArray: ...


@dataclass(frozen=True)
class ConstantWind:
    vector_w: FloatArray

    def velocity(self, t: float, position_w: FloatArray) -> FloatArray:
        return np.asarray(self.vector_w, dtype=float)


NO_WIND = ConstantWind(np.zeros(3))
