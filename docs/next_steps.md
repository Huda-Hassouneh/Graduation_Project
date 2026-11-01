# Next steps

## Current position (2026-11-01)

The DJI Matrice 400 company-platform phase is implemented (docs/PROJECT_STATUS.md): spec database, M400
reference model with sensors/ESKF, PID + DOB controllers, the research framework, facade mission, 13
scenarios, campaigns, Gazebo replay and a Gazebo-physics flight model. The older milestone plan below is
kept for history; Milestones 4 (wind and sensors) and parts of 5–8 were delivered in that phase.

**Next engineering milestone (recommended): DJI-interface-realistic evaluation.**
1. Literature review (docs/literature_review.md scope) before any contribution or novelty claim.
2. Add a velocity-setpoint "DJI mode" to the simulator: research framework as an outer loop at 10–25 Hz
   over a DJI-like inner loop with measured-style latency; re-formulate identification on commanded
   velocity -> measured acceleration; re-run the campaign (DJI_MATRICE_400_INTEGRATION_PLAN.md §7, §10.1).
3. Finish the 30-seed campaigns if fewer seeds were completed (they are resumable).
4. When the aircraft is available: read-only telemetry logging (rates, latency), then identification
   flights to replace assumptions (ASSUMPTIONS_AND_LIMITATIONS.md §6).


Milestones 0–2 are done (see validation_report.md). These are the exact next actions, in order.

## Open verification items (do first, small)
1. Read the motor/propeller page of the *Matrice 400 Maintenance Manual v1.0* PDF (link in
   aircraft_reference.md §3). Quote the rotor count and spin directions. If they differ, update
   `spin_dirs` in `config/aircraft/m400_reference_quad.yaml`.
2. Optional: `sudo apt install poppler-utils python3-pandas`, which enables PDF text extraction
   and pandas analysis. Pin the versions in docs/environment.md if installed.

## Milestone 3: Gazebo world and model
**Status (2026-10-30): implemented and verified headlessly**, with kinematic replay as the
Gazebo role (item 4). Rotors are SDF frames, not joints. Open decisions are listed in
docs/gazebo_milestone3.md. Original plan:

1. `simulation/models/research_quad/model.sdf` and `m400_reference_quad/model.sdf`, generated
   from the YAML files by a script (one source of truth). Link origin at the CoM, inertia and
   rotor joints from the config, visual envelope for the M400 reference of 980×760×480 mm.
2. Parameterised facade world generator (`simulation/worlds/generate_facade_world.py`): flat
   facade, complex facade (corners, balconies), ground, lighting, keep-out boxes, take-off
   pad. The scenario facade (plane y = 8 m, x ∈ [0, 12]) must match `experiments/scenarios.py`.
3. Validate with `gz sdf -k <file>` (syntax) **and** a headless `gz sim -s -r --iterations N`
   load test (topics exist and the model spawns). An SDF syntax check is not a physics check.
4. Decide the Gazebo role: replay Python trajectories kinematically (recommended first,
   because there is no second physics engine), then optionally compare a Gazebo-physics hover
   against the Python simulator.

## Milestone 4: wind and sensors
1. `environment/wind.py`: gusts (1-cosine), time-varying direction, Dryden-like coloured
   noise with a seeded `numpy.random.Generator`, and an optional simplified facade-proximity
   model, clearly labelled.
2. `sensors/`: IMU, GNSS, barometer, range models with rate, noise, bias random walk, delay
   buffer, dropout and validity flags. Plus an EKF (`sensors/ekf.py`) that consumes them.
3. Swap `ideal_state_feedback` for the estimator behind the same `StateEstimate` interface.
   Add a test that the controller path cannot reach `plant.state` when `feedback_mode = "estimated"`.
4. Write docs/sensor_models.md and docs/environment_and_wind.md.

## Milestone 5 and later
Literature review (docs/literature_review.md scope) **before** choosing the diagnosis method.
Then passive diagnosis (residuals, CUSUM/GLR, RLS for mass and effectiveness, with an
identifiability analysis), recovery and energy (M6), active diagnosis (M7), and seeded batch
experiments with confidence intervals and ablations (M8).

## Performance note
A 30 s run takes 15–20 s wall time while the user's Gazebo session is running. For
M8-scale batches (about 10³ runs), consider running batches with Gazebo closed,
multiprocessing across seeds (8 cores), or vectorising the plant across trials.
