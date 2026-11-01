"""Disturbance-observer-assisted cascaded controller (the "advanced" controller of the comparison).

Selection rationale (docs/CONTROL_ARCHITECTURE.md): compared with NMPC, a disturbance observer (DOB)

* runs in well under a millisecond in Python at 250 Hz and needs no new dependency (no QP/NLP solver);
* keeps the PID inner loops and gains IDENTICAL, so the comparison isolates one design change;
* produces an explicit lumped-disturbance estimate that is meaningful for diagnosis and recovery.

Translational observer (world frame), using only the state estimate and the last command::

    a_model = (T_cmd / m_nom) R e_z - g e_z           (nominal model, previous control step)
    z'      = a_model + d_hat
    d_hat   = L (v_est - z)                            (first-order, bandwidth L [1/s])

so ``d_hat`` converges to the unmodelled acceleration (wind/drag forces, mass or thrust changes) with
time constant ``1/L``. The position loop then commands ``a_cmd - d_hat`` (feed-forward cancellation,
bounded by ``max_compensation``). The PID integrator stays active (with anti-windup) for residual bias.
Limits: the observer cannot separate a disturbance from a model error; fast disturbances above ``L``
are not compensated; noisy velocity estimates limit ``L``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from graduation_project.config import CONFIG_DIR, AircraftParams, load_yaml
from graduation_project.controllers.pid import E_Z, CascadedPIDController, ControlOutput, PIDGains
from graduation_project.dynamics.frames import FloatArray, quat_to_rotmat
from graduation_project.planning.trajectories import RefPoint
from graduation_project.sensors.state_estimate import StateEstimate


@dataclass(frozen=True)
class DOBParams:
    bandwidth: FloatArray  # observer bandwidth per world axis [1/s]
    max_compensation: FloatArray  # bound on |d_hat| used for compensation per axis [m/s^2]


def load_dob_params(aircraft_name: str, path: str | Path | None = None) -> DOBParams:
    d = load_yaml(path or CONFIG_DIR / "controllers" / "dob.yaml")["aircraft"][aircraft_name]
    return DOBParams(np.asarray(d["bandwidth"], dtype=float), np.asarray(d["max_compensation"], dtype=float))


class DisturbanceObserverController(CascadedPIDController):
    def __init__(self, nominal: AircraftParams, gains: PIDGains, dob: DOBParams) -> None:
        self.dob = dob
        super().__init__(nominal, gains)

    def reset(self) -> None:
        super().reset()
        self._z: FloatArray | None = None
        self.d_hat = np.zeros(3)
        self._a_model = np.zeros(3)
        self._t_obs: float | None = None

    def _observe(self, est: StateEstimate) -> None:
        if self._z is None:
            self._z = est.velocity.copy()
            self._t_obs = est.t
            return
        dt = est.t - self._t_obs
        self._t_obs = est.t
        if dt <= 0:
            return
        self.d_hat = self.dob.bandwidth * (est.velocity - self._z)
        self._z = self._z + dt * (self._a_model + self.d_hat)

    def _desired_force(self, est: StateEstimate, ref: RefPoint, dt: float) -> tuple[FloatArray, FloatArray]:
        comp = np.clip(self.d_hat, -self.dob.max_compensation, self.dob.max_compensation)
        shifted = RefPoint(ref.position, ref.velocity, ref.acceleration - comp, ref.yaw, ref.yaw_rate)
        return super()._desired_force(est, shifted, dt)

    def update(self, est: StateEstimate, ref: RefPoint) -> ControlOutput:
        self._observe(est)
        out = super().update(est, ref)
        R = quat_to_rotmat(est.quaternion)
        thrust = float(out.allocation.achieved_wrench[0])
        # achieved/commanded acceleration under the *current* model (model_thrust_gain applied)
        self._a_model = self.model_thrust_gain * thrust / self.nom.mass * R[:, 2] - self.nom.gravity * E_Z
        return out
