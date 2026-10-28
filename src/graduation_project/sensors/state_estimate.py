"""State-estimate interface consumed by controllers, and an explicit ground-truth adapter.

Controllers only ever receive a :class:`StateEstimate`. In Milestones 1-2 the estimate is the
true state passed through :func:`ideal_state_feedback`; every run records
``feedback_mode = "ideal_truth"`` in its metadata so results are never mistaken for
sensor-based experiments. Milestone 4 replaces this with a sensor + estimator pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass

from graduation_project.dynamics.frames import FloatArray
from graduation_project.dynamics.multirotor import PlantState

IDEAL_TRUTH = "ideal_truth"


@dataclass(frozen=True)
class StateEstimate:
    t: float
    position: FloatArray  # world ENU [m]
    velocity: FloatArray  # world ENU [m/s]
    quaternion: FloatArray  # body->world
    angular_velocity: FloatArray  # body FLU [rad/s]
    valid: bool = True
    source: str = IDEAL_TRUTH


def ideal_state_feedback(t: float, state: PlantState) -> StateEstimate:
    """Copy the true rigid-body state (rotor speeds deliberately excluded)."""
    return StateEstimate(
        t,
        state.position.copy(),
        state.velocity.copy(),
        state.quaternion.copy(),
        state.angular_velocity.copy(),
    )
