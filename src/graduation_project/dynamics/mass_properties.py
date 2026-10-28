"""Mass properties and a lumped-component inertia estimator.

The estimator is used only to *derive* assumed inertia values when no measured tensor exists.
Each component is a solid cuboid (or point mass when ``size`` is zero) at a position relative
to the reference point; the tensor is shifted to the composite centre of mass with the parallel-axis
theorem. Results are recorded in the aircraft YAML files together with their provenance.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from graduation_project.dynamics.frames import FloatArray


@dataclass(frozen=True)
class MassProperties:
    mass: float  # [kg]
    inertia: FloatArray  # (3, 3) about the CoM, body axes [kg m^2]
    inertia_inv: FloatArray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        J = np.asarray(self.inertia, dtype=float)
        if self.mass <= 0.0:
            raise ValueError("mass must be positive")
        if J.shape != (3, 3) or not np.allclose(J, J.T):
            raise ValueError("inertia must be a symmetric 3x3 matrix")
        eig = np.linalg.eigvalsh(J)
        if np.any(eig <= 0.0):
            raise ValueError("inertia must be positive definite")
        # Triangle inequality on principal moments (physical realisability).
        a, b, c = np.sort(eig)
        if a + b < c * (1.0 - 1e-9):
            raise ValueError("principal moments violate the triangle inequality")
        object.__setattr__(self, "inertia", J)
        object.__setattr__(self, "inertia_inv", np.linalg.inv(J))


@dataclass(frozen=True)
class Component:
    name: str
    mass: float  # [kg]
    position: tuple[float, float, float]  # centroid [m]
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)  # cuboid edge lengths [m]; zeros = point mass


def lumped_mass_properties(components: list[Component]) -> tuple[MassProperties, FloatArray]:
    """Return mass properties about the composite CoM and the CoM position."""
    masses = np.array([c.mass for c in components])
    positions = np.array([c.position for c in components], dtype=float)
    total = masses.sum()
    com = (masses[:, None] * positions).sum(axis=0) / total
    J = np.zeros((3, 3))
    for c, r in zip(components, positions - com):
        lx, ly, lz = c.size
        J_own = (c.mass / 12.0) * np.diag([ly**2 + lz**2, lx**2 + lz**2, lx**2 + ly**2])
        J += J_own + c.mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return MassProperties(float(total), J), com
