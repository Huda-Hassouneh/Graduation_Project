# Graduation project

**Working title:** *Energy-Aware Active Diagnosis and Safe Trajectory Recovery for Quadrotors
Under Unknown Dynamics Changes.* This is a simulation-first, hardware-independent research platform.

> **Status (2026-10-28):** Milestones 0–2 are complete. They provide a validated 6-DOF quadrotor
> model and a baseline PID tracking controller with **ideal state feedback**. There are no
> sensors, diagnosis, recovery, Gazebo world or literature review yet; see
> [docs/next_steps.md](docs/next_steps.md). No research claims are made.

## Aircraft configurations

| Config | Meaning |
|---|---|
| `research_quad` | Generic 1.5 kg X-quadrotor with assumed parameters, for algorithm development (class A) |
| `m400_reference_quad` | Quadrotor with DJI Matrice 400 published mass, wheelbase, propeller size, battery and limits; propulsion and inertia assumed (class B, **not a digital twin**) |

Details and sources: [docs/aircraft_reference.md](docs/aircraft_reference.md).

## Setup

Verified on Ubuntu 24.04.5 (WSL 2) with Python 3.12.3 and the system packages
`python3-numpy` 1.26.4, `python3-scipy` 1.11.4, `python3-matplotlib` 3.6.3,
`python3-yaml` 6.0.1 and `python3-pytest` 7.4.4. On a fresh Ubuntu 24.04:

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-yaml python3-pytest
git clone <repo-url> "Graduation project" && cd "Graduation project"
```

Elsewhere, use `pip install -r requirements.txt` (exact pins). ROS 2 Jazzy and Gazebo
Harmonic are not needed for anything implemented so far.

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

## Repository layout

```
config/aircraft/        provenance-annotated aircraft parameter files
config/controllers/     controller gains and design parameters
src/graduation_project/ dynamics, actuators, environment, energy, controllers, planning,
                        sensors, experiments, metrics, visualization (diagnosis: planned)
tests/                  unit and integration tests (pytest; `slow` marker for long tests)
scripts/                command-line entry points
docs/                   equations, frames, design, protocol, validation, limitations
simulation/             Gazebo models and worlds (Milestone 3)
results/, figures/      generated outputs
```

## Documentation

[architecture](docs/architecture.md) · [coordinate frames](docs/coordinate_frames.md) ·
[model equations](docs/model_equations.md) · [aircraft reference](docs/aircraft_reference.md) ·
[controller design](docs/controller_design.md) · [energy model](docs/energy_model.md) ·
[experiment protocol](docs/experiment_protocol.md) · [validation report](docs/validation_report.md) ·
[limitations](docs/limitations.md) · [literature review](docs/literature_review.md) (not yet conducted) ·
[research contribution](docs/research_contribution.md) · [environment](docs/environment.md) ·
[next steps](docs/next_steps.md)
