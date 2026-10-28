# Experiment protocol (living document)

## Current scope (Milestone 2)

Baseline PID, ideal state feedback, no wind unless stated, `seed = 0` (nothing is stochastic
yet). Scenarios are defined in `experiments/scenarios.py` and run with `scripts/run_baseline.py`.

## Metric definitions

Evaluation window `W = {k : t_k ≥ t_start}` at the 250 Hz control rate, with N samples.

| Metric | Definition | Unit |
|---|---|---|
| `pos_rmse_m` | `√((1/N) Σ_k ‖p_ref,k − p_k‖²)` | m |
| `pos_max_err_m` | `max_k ‖p_ref,k − p_k‖` | m |
| `pos_rmse_xy_m`, `pos_rmse_z_m` | same, using horizontal / vertical components only | m |
| `att_err_mean_deg`, `att_err_max_deg` | geodesic angle `2 acos|⟨q_k, q_des,k⟩|` | deg |
| `tilt_max_deg` | `max_k acos(R_k[2,2])`, angle between z_B and z_W | deg |
| `energy_J` | `E(t_end) − E(t_start)` | J |
| `mean_power_W` | `energy_J / (t_end − t_start)` | W |
| `energy_per_m_J` | `energy_J / Σ‖p_{k+1} − p_k‖`; NaN if path < 1 mm | J/m |
| `saturation_ratio` | fraction of control steps where the allocator saturated | – |

Detection, safety, mission and statistical metrics will be added with their modules (M5–M8).

## Log format (`results/runs/<id>/timeseries.csv`)

One row per control update (250 Hz). Units are SI and angles are in rad.
Columns: `t`; `p_*`, `v_*` (world ENU); `q_w..q_z` (body→world); `roll, pitch, yaw`
(Z-Y-X, logging only); `w_*` (body rates); `ref_p_*`, `ref_v_*`, `ref_a_*`, `ref_yaw`;
`qdes_*`, `att_err`, `wdes_*`; `pos_err`; `T_cmd`, `tau_cmd_*`, `alloc_saturated`;
`omega_cmd_i`, `omega_i` [rad/s]; `thrust_i` [N] (actual); `power_el` [W]; `energy` [J];
`wind_*` [m/s]; `mass_true` [kg].

`metadata.json` holds the experiment ID, scenario, seed, feedback mode, full aircraft YAML,
controller gains, sim config, initial conditions, trajectory, wind, injected changes,
metrics, status, warnings, git commit and dirty flag, and software versions.

## Rules for later comparative experiments (pre-declared)

* Identical aircraft file, initial conditions, trajectories, disturbance seeds, `dt_sim`
  (4 ms) and control rate (250 Hz) for every method.
* Separate tuning scenarios/seeds from evaluation scenarios/seeds. Held-out conditions are
  declared before running.
* Report mean, standard deviation, n and 95 % confidence intervals (bootstrap or t-based as
  appropriate). Use paired tests across matched seeds. Report every run, not selected ones.
