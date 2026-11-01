# Project audit (start of the DJI Matrice 400 phase, 2026-11-01)

State of the repository **before** the Matrice 400 work in this phase, as inspected on 2026-11-01.
What was built afterwards is in PROJECT_STATUS.md.

## 1. Repository and version control

* Working directory `/home/huda/Graduation project`, git branch `main`, last commit a1cb0b3
  ("Exclude generated outputs from the run-metadata dirty check"). Milestones 0–2 are committed.
* Uncommitted (preserved, not modified by this phase): Milestone 3 Gazebo work (models, worlds,
  replay, launcher, checker, tests, docs), six modified baseline figures and `results/baseline_summary.csv`
  from a re-run of the Milestone 2 batch, the earlier audit `docs/upgrade_audit_hexacopter.md`, and small
  edits to README / docs from the Milestone 3 verification session.

## 2. Environment and dependencies (verified)

| item | status | how verified |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS on WSL 2 (kernel 6.6.87.2) | `lsb_release`, environment |
| Python | 3.12.3 with NumPy 1.26.4, SciPy 1.11.4, Matplotlib 3.6.3, PyYAML 6.0.1, pytest 7.4.4, psutil 5.9.8 | imports |
| ROS 2 | **Jazzy installed** at `/opt/ros/jazzy` (319 packages incl. `rclpy`, `ros_gz_bridge`, `ros_gz_sim`); **not sourced** in the default shell | `source /opt/ros/jazzy/setup.bash && ros2 pkg list` |
| Gazebo | Gazebo Sim 8.15.0 (Harmonic) with `gz.transport13` / `gz.msgs10` Python bindings | `gz sim --versions`, imports |
| Not available | CasADi / acados (no NMPC solver), pandas, pip, poppler (PDF rendering) | import / command checks |

ROS 2 is not needed by anything implemented; it is an option for a later software-in-the-loop bridge.

## 3. Existing architecture and capabilities

| area | state at audit | nature |
|---|---|---|
| 6-DOF quaternion plant, RK4 4 ms, n-rotor quasi-static thrust/torque, first-order motor lag, effectiveness/lag faults | implemented, tested (analytic properties) | Python physics |
| Allocation (pseudo-inverse with priority scaling) | implemented | control |
| Aircraft configs | `research_quad` (generic, assumed) and `m400_reference_quad` (M400-inspired, schema v1, partial provenance) | config |
| Drag / wind / atmosphere | lumped quadratic drag; constant wind only; fixed density | physics (minimal) |
| Energy | shaft power / efficiency + avionics, integrated; no battery model | bookkeeping |
| Controller | cascaded PID with **ideal truth feedback** | control |
| Sensors, estimator, identification, diagnosis, active diagnosis, recovery, energy-aware decisions | **absent** (empty packages) | — |
| Scenarios / results | 8 tracking scenarios, 16 deterministic runs, no wind, no faults | baseline only |
| Gazebo | static visual models and two facade worlds; kinematic replay of Python logs | visual / replay only |
| Ground contact, take-off and landing | **absent** (runs started airborne) | — |

**Hexacopter question.** The repository contains **no hexacopter model** (visual or dynamic). The
previous session produced only an audit document (`docs/upgrade_audit_hexacopter.md`) that analysed a
proposed hexacopter upgrade and found that the DJI Matrice 400 is a four-rotor aircraft. The two existing
models were quadrotors: dynamic in Python, static (visual) in Gazebo.

## 4. Issues that prevented a working company-platform simulation

1. No M400 specification database with source-level traceability; the v1 file mixed manufacturer values
   and assumptions without ranges/sensitivities and lacked sensors, interfaces and limits.
2. Controllers used ground truth (no sensors / estimator), so no diagnosis research was possible.
3. No take-off/landing (no ground contact); no battery, atmosphere, turbulence or gust models.
4. No mission manager able to replan; facade geometry not used as a safety constraint.
5. None of the six research capabilities beyond trajectory tracking existed.
6. Gazebo models could not fly (static); no Gazebo-physics propulsion.
7. Broken link: README referenced `docs/validation_report.md`, which does not exist (still open; see
   docs/next_steps.md — not fixed here because it belongs to the Milestone 2 documentation).

## 5. Implementation sequence chosen

1. Traceable M400 specification database + validator + exports (Phase C).
2. Physics extensions with no change to existing behaviour (contact, arming, delay, density, battery voltage
   headroom; ISA; profile/turbulence/gust wind; equivalent-circuit battery) — Phase D.
3. Sensors with faults + 15-state ESKF; framework information boundary (Phase E §7.2).
4. DOB-assisted controller as the advanced controller (NMPC deferred: no solver, fairness) (Phase E §7.1).
5. Online identification, diagnosis with "undetermined" outcome, active diagnosis with information/energy/
   safety trade-off, recovery supervisor, energy manager (Phase E, H).
6. Facade coverage mission with hard limits and replanning (Phase F); 13 scenarios (Phase G).
7. Campaign runner A–D + ablations, metrics, statistics, figures (Phases I, J).
8. Gazebo: M400 static replay model + Gazebo-physics flight model + flight test; M400 worlds.
9. Integration plan (Phase K) and documentation.
