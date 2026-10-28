# Architecture

## Authoritative simulator

The **Python mathematical simulator** (`src/graduation_project`) is the authoritative physics
for every scientific experiment. It is deterministic, headless and fast enough for batches.
Gazebo Harmonic (Milestone 3) will be used for 3D visualisation, sensor rendering (camera)
and selected integration tests. Where Gazebo integrates its own physics, its trajectories
will be compared with the Python simulator under identical inputs, and any difference will be
reported. Two unvalidated physics implementations will not be used interchangeably.

## Data flow (current state: Milestone 2)

```
Trajectory.sample(t) ──► RefPoint ─────────────────────────────┐
                                                               ▼
MultirotorPlant (truth) ──► ideal_state_feedback ──► StateEstimate ──► CascadedPIDController
   ▲  ▲  ▲                    (Milestone 4: sensors + estimator)            │
   │  │  └── ScheduledChange (truth mass / effectiveness / lag / drag)      │ wrench → allocation
   │  └───── WindField.velocity(t, p)                                      ▼
   └──────── Ω_cmd (ZOH) ◄──────────────────────────────────── ControlAllocator
             │
             └─► EnergyIntegrator (P_el from true Ω, η) ──► RunResult ──► metrics / io / plots
```

## Packages

| Package | Responsibility | Status |
|---|---|---|
| `dynamics/` | frames and quaternions, mass properties, 6-DOF plant, state validation | done (M1) |
| `actuators/` | rotor geometry, lag, saturation, effectiveness; allocation | done (M1) |
| `environment/` | drag; wind interface (constant only) | partial (M4 adds gusts, turbulence, facade effects) |
| `energy/` | shaft and electrical power, energy integration | done (M1); remaining-energy estimate in M6 |
| `controllers/` | cascaded PID baseline | done (M2) |
| `planning/` | quintic and waypoint trajectories, circle/helix, facade raster | done (M2); recovery planner in M6 |
| `sensors/` | `StateEstimate` interface, explicit ideal-truth adapter | interface only (M4) |
| `diagnosis/` | residuals, change detection, estimation | empty (M5) |
| `experiments/` | runner, scenarios, run persistence | done (M2) |
| `metrics/` | tracking and energy metrics | done (M2) |
| `visualization/` | publication figures | done (M2) |
| `simulation/` | Gazebo models and worlds | empty (M3) |

## Separation rules enforced in code

* Controllers receive only `StateEstimate` and `RefPoint`. They hold a *nominal* copy of the
  aircraft parameters and never see `MultirotorPlant.truth`.
* Truth modifications go only through `ScheduledChange.apply(plant)`, and every one is
  recorded in metadata.
* Every run records its `feedback_mode`, so ideal-feedback results are never mistaken for
  sensor-based results.
