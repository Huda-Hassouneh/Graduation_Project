# Control and research architecture

## 1. Layers and modules

| # | layer | module | inputs → outputs | rate |
|---|---|---|---|---|
| 1 | mission planner | `planning/mission.py` (`build_coverage_mission`) | facade map, coverage config → waypoints with phases | once / on replanning |
| 2 | trajectory generator | `planning/mission.py` (`MissionManager`) | waypoints, motion limits, recovery actions → C² reference p, v, a, yaw | 250 Hz |
| 3 | position controller | `controllers/pid.py` (`_desired_force`), `controllers/dob.py` | estimate, reference → desired force / attitude | 250 Hz |
| 4 | attitude controller | `controllers/pid.py` (attitude + rate loops) | desired attitude → body torque | 250 Hz |
| 5 | control allocator | `actuators/allocation.py` | wrench → rotor thrusts / speeds (saturation priority) | 250 Hz |
| 6 | propulsion model | `actuators/rotor.py`, plant | rotor speed commands → forces (truth side) | 4 ms |
| 7 | state estimator | `estimation/eskf.py` | IMU, GNSS, baro, heading, range → state + covariance + sensor health | 250 Hz |
| 8 | online identification | `sysid/online.py`, `sysid/rls.py` | IMU means, estimate, own commands → θ, τ_eff, (c, w), δ_GNSS + uncertainty | 50 Hz |
| 9 | anomaly detection & diagnosis | `diagnosis/diagnoser.py` | residuals, identification, sensor health → report (cause or "undetermined") | 50 Hz |
| 10 | active diagnostic decision-maker | `active_diagnosis/planner.py`, `probes.py` | report, uncertainty, energy, safety → probe or justified rejection | on request, ≥ 3 s apart |
| 11 | safe recovery & replanning | `recovery/supervisor.py` | report, energy, safety state → explicit actions | 50 Hz (≥ 0.5 s apart) |
| 12 | energy-aware decision-maker | `energy/manager.py` (+ used by 10, 11) | battery telemetry, mission → remaining / required energy | 50 Hz |
| — | orchestrator | `framework/core.py` | wires 8–12, applies actions, logs decisions | 50 Hz |

Interfaces are typed dataclasses (`StateEstimate`, `RefPoint`, `FrameworkInputs`, `DiagnosisReport`,
`Action`, `ActiveDecision`). Aircraft-specific numbers come only from the aircraft spec and controller
YAML files, so a second quadrotor (Path B) is added by writing its spec and gains, not by changing code.

## 2. Information boundary

The framework (layers 8–12) receives `FrameworkInputs` only: estimator output and its health statistics,
bias-corrected IMU means, the controller's own allocated wrench, battery telemetry and mission state.
It never receives plant truth, the simulated battery state, wind, or the scenario's fault schedule/labels.
This is enforced by an AST test (`tests/test_framework.py::test_framework_has_no_access_to_truth`).
Truth is used only by the plant, the sensor models and the evaluation (metrics).

## 3. Baseline controller (cascaded PID)

Position PID (pole placement, ω_n = 0.9 / 1.2 rad/s), quaternion attitude P, body-rate PI, allocation.
Gains for `dji_matrice_400` follow the documented design of the earlier M400 reference model, with one
re-derivation: the rate-integrator limit was raised from 3 to 10 rad/s² (and rate_i 2.5 → 4) because a
15–30 % single-rotor loss needs 1.4–2.7 N·m of steady compensation, which the old limit (≈1 N·m)
could not provide (`config/controllers/pid_baseline.yaml`). A model-update hook (`model_thrust_gain`)
lets recovery apply an identified thrust/mass change without retuning gains.

Measured structural limitation: with the proportional rate gain limited by the assumed 60 ms motor lag,
a sudden 2.7 N·m imbalance (30 % rotor loss) forces a ≈17° attitude excursion until the integrator
compensates; at the 3 m stand-off next to balconies this crashed the aircraft (severity sweep in
`config/scenarios/actuator_degradation.yaml`).

## 4. Advanced controller: disturbance-observer-assisted PID (selected) vs NMPC (deferred)

| criterion | DOB-assisted PID | NMPC |
|---|---|---|
| research relevance | explicit lumped-disturbance estimate; directly comparable residual for diagnosis | handles constraints explicitly; strong for replanning |
| computational feasibility | < 0.1 ms per step in Python at 250 Hz | needs an NLP/QP solver (CasADi/acados not installed); real-time in Python doubtful at 250 Hz |
| simulation compatibility | drop-in (same interfaces) | needs a prediction model, warm starts, solver tuning |
| fair comparison with PID | identical gains and inner loops; isolates one change | different structure; tuning effort confounds comparison |
| integration with diagnosis/recovery | disturbance estimate + model hook | model update possible, but solver failures add a new failure mode |
| DJI transfer | outer-loop velocity/acceleration feed-forward compatible with velocity-level interfaces | would also have to run off-board at ≤ 25 Hz on velocity setpoints |

Decision: implement the DOB-assisted controller (`controllers/dob.py`); defer NMPC until a solver
dependency is approved and the literature review identifies what it would add. The DOB estimates the
unmodelled world acceleration `d̂ = L (v_est − z)`, `z' = a_model + d̂` (L = 2.5 rad/s for M400) and feeds
`−d̂` forward (bounded: 4.5 m/s² horizontal, 6 m/s² vertical); the PID integrators remain active.

## 5. DJI-compatible structure

On the real aircraft layers 4–6 are DJI's and cannot be replaced. Layers 1–3 and 7–12 can run off-board
(MSDK on an Android device or PSDK on an onboard computer) and command velocity/position setpoints at
5–25 Hz (Virtual Stick / joystick). The simulator keeps the full stack so the research algorithms can be
evaluated, but nothing in layers 8–12 requires motor-level access: they use estimator outputs, specific
force, commanded collective thrust/wrench (which on DJI would be replaced by commanded velocity and
reported attitude — see DJI_MATRICE_400_INTEGRATION_PLAN.md §7 for what changes).
