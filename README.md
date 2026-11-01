# Graduation project

**Working title:** *Energy-Aware Active Diagnosis and Safe Trajectory Recovery for Quadrotors
Under Unknown Dynamics Changes.* This is a simulation-first, hardware-independent research platform.

> **Status (2026-11-01):** Milestones 0–3 plus the **DJI Matrice 400 company-platform phase**: a
> traceable M400 specification database, an M400 reference quadrotor model (not a digital twin) with
> take-off/landing, battery, wind, sensors and an error-state EKF, a PID and a disturbance-observer
> controller, and the research framework (online identification, diagnosis with an explicit
> "undetermined" outcome, active diagnosis, safe recovery/replanning, energy-aware decisions) on a 3D
> facade-coverage mission with 13 disturbance/fault scenarios and seeded A–D/ablation campaigns.
> Everything is **simulation**; nothing is physically validated and the literature review has not been
> conducted, so **no research claims or novelty claims are made**. Current state and results:
> [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md).

## Aircraft configurations

| Config | Meaning |
|---|---|
| `research_quad` | Generic 1.5 kg X-quadrotor with assumed parameters, for algorithm development (class A) |
| `m400_reference_quad` | Earlier M400-inspired quadrotor (schema v1), kept unchanged for the Milestone 2 baseline results |
| `dji_matrice_400` | **Company platform (Path A).** Schema-v2 specification database: 162 parameters, each VERIFIED_OFFICIAL / VERIFIED_SECONDARY / ENGINEERING_ESTIMATE / SIMULATION_ASSUMPTION / UNKNOWN / NOT_PUBLICLY_DISCLOSED with sources; reference model, **not a digital twin** |

Details and sources: [docs/DJI_MATRICE_400_TECHNICAL_SPECIFICATION.md](docs/DJI_MATRICE_400_TECHNICAL_SPECIFICATION.md),
[docs/DJI_MATRICE_400_SOURCE_REGISTER.md](docs/DJI_MATRICE_400_SOURCE_REGISTER.md), [docs/aircraft_reference.md](docs/aircraft_reference.md).
Our own drone (Path B) is not selected or modelled; adding it means writing its spec and gains (no code changes).

## Setup

Verified on Ubuntu 24.04.5 (WSL 2) with Python 3.12.3 and the system packages
`python3-numpy` 1.26.4, `python3-scipy` 1.11.4, `python3-matplotlib` 3.6.3,
`python3-yaml` 6.0.1 and `python3-pytest` 7.4.4. On a fresh Ubuntu 24.04:

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-yaml python3-pytest
git clone <repo-url> "Graduation project" && cd "Graduation project"
```

Elsewhere, use `pip install -r requirements.txt` (exact pins). Gazebo Harmonic (Gazebo Sim 8,
with its `gz.transport13` Python bindings) is needed only for the Milestone 3 commands below;
the Gazebo tests skip if `gz` is not installed. `psutil` (system package `python3-psutil`) is used for
campaign CPU/memory metrics. ROS 2 Jazzy is installed on the development machine but not needed.

## Run

```bash
# Quick smoke tests (~30 s)
python3 -m pytest -m "not slow"

# Comprehensive tests (several minutes)
python3 -m pytest

# One baseline simulation
python3 scripts/run_baseline.py --aircraft research_quad --scenario circle

# Full baseline batch: 8 scenarios × 2 aircraft
python3 scripts/run_baseline.py

# Print derived model coefficients for each aircraft
python3 scripts/derive_parameters.py
```

Each run writes `results/runs/<experiment_id>/timeseries.csv` and `metadata.json` (config
snapshot, git commit, versions, metrics). The batch also writes
`results/baseline_summary.csv` and figures under `figures/baseline/`.

### DJI Matrice 400 platform

```bash
# Specification database: validate + regenerate the JSON export and the parameter table
python3 scripts/export_m400_spec.py --check
python3 scripts/calibrate_m400_power.py          # reproduces the C_P calibration

# One facade-coverage mission (take-off -> coverage -> landing) with logs and an overview figure
python3 scripts/run_mission.py --scenario nominal
python3 scripts/run_mission.py --scenario mass_change --controller dob --seed 3
python3 scripts/run_mission.py --scenario sensor_delay --framework off       # stock behaviour

# Seeded campaigns (resumable) and analysis
python3 scripts/run_campaign.py --suite smoke                               # 13 scenarios x A-D x 1 seed
python3 scripts/run_campaign.py --suite main --seeds 30                     # A-D x 13 scenarios x 30 seeds
python3 scripts/run_campaign.py --suite ablations --seeds 10
python3 scripts/analyze_campaign.py results/campaign/main_30seeds
python3 scripts/analyze_campaign.py results/campaign/ablations_10seeds --reference C

# Gazebo: replay a Python mission in the M400 facade world; Gazebo-physics flight test
simulation/gazebo/launch_world.sh facade_complex_m400 &
python3 simulation/gazebo/replay_run.py --world facade_complex_m400 --run results/missions/<id> --speed 4
python3 simulation/gazebo/flight_test.py
```

Architecture: [docs/CONTROL_ARCHITECTURE.md](docs/CONTROL_ARCHITECTURE.md). Physics and models:
[docs/SIMULATION_MODEL.md](docs/SIMULATION_MODEL.md). Methods: [docs/DIAGNOSIS_AND_RECOVERY.md](docs/DIAGNOSIS_AND_RECOVERY.md),
[docs/ENERGY_MODEL.md](docs/ENERGY_MODEL.md), [docs/EXPERIMENTAL_METHODOLOGY.md](docs/EXPERIMENTAL_METHODOLOGY.md).

### Gazebo (Milestone 3)

The worlds include the aircraft as `model://<aircraft>`. The scripts below set the model
search paths (`GZ_SIM_RESOURCE_PATH`, `SDF_PATH`) themselves, so they work from a fresh
terminal. Plain `gz sim` / `gz sdf` need `source simulation/gazebo/env.sh` first.

```bash
# Validate models + worlds and load all four worlds headlessly
python3 simulation/gazebo/check_worlds.py

# Launch a world (GUI), or headless with --headless
simulation/gazebo/launch_world.sh facade_flat
simulation/gazebo/launch_world.sh facade_complex --headless

# Kinematic replay of a run into the running world (second terminal)
python3 simulation/gazebo/replay_run.py --world facade_flat --run results/runs/<experiment_id>
```

Details, verification results and limitations: [docs/gazebo_milestone3.md](docs/gazebo_milestone3.md).

## Repository layout

```
config/aircraft/        provenance-annotated aircraft parameter files (dji_matrice_400.yaml: schema v2)
config/controllers/     controller gains and design parameters (PID, DOB)
config/scenarios/       13 mission scenarios (disturbances, faults, energy, generalisation)
src/graduation_project/ dynamics (+contact), actuators, environment (wind, ISA), energy (power, battery,
                        manager), sensors, estimation (ESKF), controllers (PID, DOB), sysid, diagnosis,
                        active_diagnosis, recovery, framework, planning (trajectories, mission),
                        experiments (runner, scenarios, campaigns), metrics, gazebo, visualization
tests/                  unit and integration tests (pytest; `slow` marker for long tests)
scripts/                command-line entry points
docs/                   equations, frames, design, protocol, validation, limitations
simulation/             Gazebo models, worlds, launcher and replay (Milestone 3)
config/visual/          Gazebo-only appearance/contact shapes (not read by the simulator)
results/, figures/      generated outputs
```

## Documentation

**DJI Matrice 400 phase:** [project audit](docs/PROJECT_AUDIT.md) · [status](docs/PROJECT_STATUS.md) ·
[M400 specification](docs/DJI_MATRICE_400_TECHNICAL_SPECIFICATION.md) · [source register](docs/DJI_MATRICE_400_SOURCE_REGISTER.md) ·
[parameter table](docs/DJI_MATRICE_400_PARAMETER_TABLE.md) · [simulation model](docs/SIMULATION_MODEL.md) ·
[control architecture](docs/CONTROL_ARCHITECTURE.md) · [diagnosis & recovery](docs/DIAGNOSIS_AND_RECOVERY.md) ·
[energy model](docs/ENERGY_MODEL.md) · [experimental methodology](docs/EXPERIMENTAL_METHODOLOGY.md) ·
[DJI integration plan](docs/DJI_MATRICE_400_INTEGRATION_PLAN.md) · [assumptions & limitations](docs/ASSUMPTIONS_AND_LIMITATIONS.md)

**Milestones 0–3:** [architecture](docs/architecture.md) · [coordinate frames](docs/coordinate_frames.md) ·
[model equations](docs/model_equations.md) · [aircraft reference](docs/aircraft_reference.md) ·
[controller design](docs/controller_design.md) · [energy model](docs/energy_model.md) ·
[experiment protocol](docs/experiment_protocol.md) · [validation report](docs/validation_report.md) ·
[limitations](docs/limitations.md) · [Gazebo (M3)](docs/gazebo_milestone3.md) · [literature review](docs/literature_review.md) (not yet conducted) ·
[research contribution](docs/research_contribution.md) · [environment](docs/environment.md) ·
[next steps](docs/next_steps.md)
