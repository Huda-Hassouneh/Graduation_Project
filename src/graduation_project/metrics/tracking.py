"""Tracking, attitude, actuator and energy metrics (definitions: docs/experiment_protocol.md).

Over the evaluation window W = {k : t_k >= t_start} with N samples at the control rate:

* position RMSE      sqrt( (1/N) sum_k ||p_ref,k - p_k||^2 )            [m]
* max position error max_k ||p_ref,k - p_k||                             [m]
* attitude error     geodesic angle between q_k and q_des,k (mean, max)  [deg]
* energy             E(t_end) - E(t_start) from the trapezoidal integral [J]
* energy / distance  energy / sum_k ||p_{k+1} - p_k||                   [J/m]
* saturation ratio   fraction of control steps with allocator saturation [-]
"""

from __future__ import annotations

import numpy as np

from graduation_project.experiments.runner import RunResult


def tracking_metrics(run: RunResult, t_start: float = 0.0) -> dict[str, float]:
    t = run.col("t")
    m = t >= t_start
    if not np.any(m):
        raise ValueError("empty evaluation window")
    err = run.col("pos_err")[m]
    att = np.rad2deg(run.col("att_err")[m])
    p = run.cols("p")[m]
    e = run.col("energy")[m]
    dist = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    energy = float(e[-1] - e[0])
    xy_err = np.linalg.norm(run.cols("ref_p")[m][:, :2] - p[:, :2], axis=1)
    z_err = np.abs(run.col("ref_p_z")[m] - p[:, 2])
    return {
        "window_start_s": float(t_start),
        "window_end_s": float(t[m][-1]),
        "n_samples": int(m.sum()),
        "pos_rmse_m": float(np.sqrt(np.mean(err**2))),
        "pos_max_err_m": float(err.max()),
        "pos_rmse_xy_m": float(np.sqrt(np.mean(xy_err**2))),
        "pos_rmse_z_m": float(np.sqrt(np.mean(z_err**2))),
        "att_err_mean_deg": float(att.mean()),
        "att_err_max_deg": float(att.max()),
        "tilt_max_deg": float(np.rad2deg(np.max(np.arccos(np.clip(1 - 2 * (run.col("q_x")[m] ** 2 + run.col("q_y")[m] ** 2), -1, 1))))),
        "energy_J": energy,
        "mean_power_W": energy / max(float(t[m][-1] - t[m][0]), 1e-12),
        "distance_m": dist,
        "energy_per_m_J": energy / dist if dist > 1e-3 else float("nan"),
        "saturation_ratio": float(np.mean(run.col("alloc_saturated")[m])),
    }
