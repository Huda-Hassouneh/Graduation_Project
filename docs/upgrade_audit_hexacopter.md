# Audit and architecture: dual-hexacopter upgrade

**Date:** 2026-10-30. **Type:** audit and plan only. No code, configuration or result was changed
to produce this document. All statements about the repository refer to the working tree on
this date, including the uncommitted Milestone 3 work.

Evidence labels used below:
**[V]** verified against a manufacturer source or by running code/tests in this session;
**[S]** strong secondary evidence; **[A]** assumption; **[U]** unknown or proprietary.

---

## 0. Findings that change the brief

### 0.1 The DJI Matrice 400 is not a hexacopter

The brief asks for "a true six-rotor hexacopter, NOT a quadcopter approximation". The
evidence says the M400 is a **four-rotor** aircraft:

| Evidence | Value | Label |
|---|---|---|
| DJI spec page (enterprise.dji.com/matrice-400/specs, read 2026-10-30) | 1070 mm diagonal wheelbase, 25 in 2510F propellers, motor model 7510, unfolded 980 × 760 × 480 mm, folded 490 × 490 × 480 mm. **No rotor count is stated.** | [V] |
| Geometry, four-rotor X | adjacent hub spacing 1070/√2 = 757 mm, matching the 760 mm published width; 635 mm discs do not overlap | [V] (computed) |
| Geometry, coplanar hexagon at the same wheelbase | adjacent hub spacing 535 mm < 635 mm disc diameter: **the rotor discs would overlap** | [V] (computed) |
| Maintenance Manual v1.0 (summary of the official PDF) | motors numbered 1–4 | [S]; first-hand quotation still pending (the PDF cannot be rendered here without `poppler-utils`) |
| Propellers sold as CW/CCW pairs | consistent with an even rotor count; does not discriminate between 4 and 6 | — |

A six-rotor "Matrice 400" would therefore contradict the manufacturer's own published
dimensions. Building one would mean inventing a configuration, which the brief itself forbids.
**This must be resolved by a human decision before any Model A work (D1, section 12).**
Options:

* **(a) Recommended.** Keep Model A as the M400 **quadrotor** reference (class B, as now). If a
  hexacopter reference is scientifically needed, add a separate, *published* hexacopter as the
  reference, after verifying that its specifications are still available. One candidate is
  the DJI Matrice 600 Pro (a six-rotor aircraft, now discontinued, with a published
  specification sheet).
* **(b)** Define an "M400-class hexacopter": a hypothetical six-rotor aircraft that reuses the
  M400 mass, battery and payload figures with its own geometry. It must carry a name that does
  not suggest it is an M400, and every geometry and propulsion value is then [A].
* **(c)** Not acceptable: label a six-rotor model "DJI Matrice 400".

### 0.2 Scope: the brief moves the research away from its stated question

The repository's research question (docs/research_contribution.md) is *energy-aware active
diagnosis and safe recovery under unknown dynamics changes*. The brief adds a full digital
twin of two aircraft, photorealistic glass, building aerodynamics, hose and water dynamics, and
a near-full-factorial campaign. Each of these is a project in its own right. Fidelity should
be added **where it changes the answer to the research question**, not because it is
available. Section 10 therefore separates a critical path from optional fidelity branches.
The scope decision belongs to you and your supervisor (D2).

### 0.3 "Digital twin" is not achievable for Model A from public data

A digital twin needs identified parameters and validation against the specific physical
aircraft. For the M400, the flight-controller internals, motor and ESC characteristics,
inertia, CoM, propeller aerodynamics and IMU specifications are **unpublished**. Without
access to an aircraft (thrust stand, pendulum inertia test, flight logs), Model A can only be
a **reference model with published-data consistency checks**. That is the existing class B,
and it is a legitimate and defensible thing to be.

### 0.4 Model B has no data in the repository

There is no design document, CAD, mass budget, motor/propeller selection, hose or water-system
specification anywhere in the repository. docs/aircraft_reference.md §5 currently states that
"hose handling and spraying are outside the research scope". Model B cannot start until the
design inputs in section 11.2 are supplied.

---

## 1. Current state audit

### 1.1 What each layer actually does

| Layer | Implemented | Nature |
|---|---|---|
| 6-DOF rigid body, quaternion, RK4 at 4 ms (`dynamics/multirotor.py`) | yes | **physics simulation** (Python) |
| n-rotor thrust `T = η k_f Ω²` and reaction torque `Q = η k_m Ω²`, along +z_B only | yes; generic in n | physics, quasi-static rotor |
| Motor: first-order speed lag with saturation; effectiveness and lag faults | yes | physics (no electrical model) |
| Allocation: pseudo-inverse with yaw-then-attitude priority scaling | yes; generic in n | control |
| Lumped per-axis linear + quadratic airframe drag, air-relative velocity | yes | physics, crude |
| Wind | constant vector only (`environment/wind.py`) | physics, minimal |
| Atmosphere | fixed ρ = 1.225 kg/m³, baked into `k_f` at config load | constant |
| Energy: `P = Σ k_m Ω³ / η + P_avionics`, integrated | yes | **energy bookkeeping, no battery** |
| Truth changes: mass/inertia, rotor effectiveness/lag, drag scale, external force/torque | yes (`ScheduledChange`) | physics |
| Controller: cascaded PID, 250 Hz, nominal parameters only | yes | control |
| State estimation | **ideal truth only** (`feedback_mode = ideal_truth`) | none |
| Sensors, faults on sensors, comms | none | — |
| Diagnosis, recovery, planning (beyond fixed trajectories) | none | — |
| Gazebo models and facade worlds | yes (M3) | **visual only**; static models, no Gazebo physics on the vehicle |
| Gazebo replay | yes (M3) | **kinematic replay** of Python logs |
| Keep-out boxes | yes | **visual markers**; not enforced, not vehicle-size-aware |
| ROS 2 | not used | — |

### 1.2 Evidence quality

* **Verification, not validation.** The 143 tests [V] check analytic properties: hover
  equilibrium, free fall, angular-momentum conservation, torque signs, fourth-order RK4
  convergence, determinism of generated files, and Gazebo loading/replay. None compares the
  model with measured flight data.
* **Results.** 16 baseline runs [V] (8 scenarios × 2 aircraft), one deterministic run each,
  ideal state feedback, no wind. They are a regression baseline, not research evidence.
* **Speed.** 20 s of simulated hover takes about 4 s wall time [V], i.e. 5× real time on one
  core with Gazebo closed (8 cores, 7 GB RAM available).

### 1.3 Defects and gaps in the existing code that matter for the upgrade

1. **The CoM cannot move.** Rotor positions are stored relative to the nominal CoM.
   `set_mass_properties` changes only mass and inertia, so a payload that shifts the CoM is
   not representable. The only workaround is an external torque, which is not physically
   consistent. **Blocking for the CoG and payload experiments.**
2. **Layout is hard-coded to `x_quad`.** `config.py` rejects any other layout, and
   `x_config_positions` produces only four rotors. The dynamics and allocation themselves are
   already generic in n.
3. **Thrust axes are fixed to +z_B.** There is no rotor cant or dihedral, so tilted-rotor
   hexacopter designs (used to improve yaw authority and fault tolerance) cannot be modelled.
4. **Air density is frozen at load time** (`k_f` and `k_m` are computed once from ρ). Altitude
   and temperature conditions need these to be evaluated at run time.
5. **Thrust does not depend on inflow or airspeed.** In 10–12 m/s wind, the thrust change and
   rotor H-force are first-order effects. Omitting them biases every wind result.
6. **Mass is constant within a step and no `ṁ` term exists.** Water depletion and jet reaction
   need a variable-mass formulation.
7. **No ground contact.** Runs start airborne, so take-off, landing and emergency landing
   cannot be simulated.
8. **Broken link.** README links `docs/validation_report.md`, which does not exist.
9. **Uncommitted state.** Milestone 3 is uncommitted, and six baseline PNGs are modified
   (pending decision from the previous session).

---

## 2. Model A gap analysis (DJI Matrice 400 reference)

This assumes decision D1 (a), i.e. the M400 is modelled with its real rotor count. Columns: what
the repository has, what public data exists, the gap, and how it can be closed.

| Item | Repository now | Public data | Gap / action |
|---|---|---|---|
| Rotor count, layout | 4, square X [S] | no explicit count | Quote the maintenance manual first-hand (needs `poppler-utils` or a manual read) |
| Spin directions | `[+1,−1,+1,−1]` [A] | manual summary contradicts itself | Quote the manual; affects only the yaw-row sign |
| Wheelbase, dimensions | 1070 mm, envelope [V] | [V] | none |
| Rotor plane height, arm dihedral/cant | 0.05 m, none [A] | [U] | Photographs/CAD-free estimate only; keep [A] with a range |
| Propeller | 25 in, 2510F [V] | pitch and C_T/C_P not published | C_T 0.10, C_P 0.045 [A]. Calibrate C_P to the 53 min hover (§7); bound C_T with a range from comparable 25 in props |
| Motor | not modelled electrically | "7510" model name only [V] | KV, R, I₀, torque limit [U]; use a class-representative datasheet, labelled [A] |
| ESC | none | [U] | Assumed first-order + rate limit + current limit |
| Mass | 9.74 kg [V]; MTOW 15.8 kg [V]; payload ≤ 6 kg [V] | [V] | none |
| CoM | assumed at the geometric centre [A] | [U] | Range-based sensitivity; needs physical measurement for truth |
| Inertia | lumped-box derivation [A] | [U] | ±30 % range; bifilar/trifilar pendulum test needed for truth |
| Battery TB100 | 977 Wh only [V] | 13S Li-ion, 20 254 mAh, 48.23 V nominal, 54.6 V max, 4C, 4.72 kg [V] | Add an equivalent-circuit model. OCV curve, R₀ and RC values [U]: use 13S Li-ion class values [A] |
| Endurance targets | 53 min hover used as a check | 53 min hover; 59 min flight with H30T at 10.67 kg [V] | Use both as calibration/consistency targets. Flight time exceeding hover time is a direct check of a translational-lift model |
| Performance limits | 35° tilt, 100 °/s yaw [V]; body rate 120 °/s [A] | 25 m/s, ascent 10 m/s, descent 8 m/s [V] | Add the ascent/descent limits to the controller/planner |
| Wind limit | documented only | 12 m/s take-off/landing [V] | Operating reference, not a safety proof |
| Altitude | none | max take-off altitude 7000 m; payload decreases with altitude [V] | Needs the atmosphere model (§4) |
| Flight controller | our PID | [U] proprietary | Cannot be reproduced. Compare against a published open architecture instead (PX4, §9) |
| Sensors | none | GNSS constellations, RTK accuracy, hover accuracy, sensing suite listed [V]; IMU/baro specs [U] | Use class-representative MEMS datasheet values [A]; check closed-loop hover error against the published ±0.1 m (RTK) |
| Payload camera | none | payload specs published per gimbal model | Choose one payload (e.g. H30T, if verified) and model its published FOV/resolution |
| Drag | crude top-speed derivation | [U] | Keep the derivation [A]; it is only valid if the top speed is drag-limited |

**Achievable claim:** "a quadrotor reference model consistent with the M400's published mass,
geometry, battery energy and performance envelope". **Not achievable:** "digital twin",
"validated M400 model", or anything about DJI's controller or obstacle-avoidance behaviour.

---

## 3. Model B gap analysis (custom heavy-lift hexacopter)

Nothing exists yet. The model needs the following, and each value needs a source, units,
provenance and uncertainty range.

### 3.1 Airframe and propulsion
* **Layout.** The rotor ordering matters. In the standard alternating hexacopter
  (PNPNPN), losing one rotor removes full controllability, whereas some alternative orderings
  (e.g. PPNNPN) keep it for certain failures. This result is reported in the hexacopter
  controllability literature (to be located and verified in the review, §9.3). It is a
  **design decision with direct consequences for the motor-failure experiments** (D4).
* Geometry: arm length, hub positions, rotor height, cant/dihedral angles, rotor diameter,
  and whether rotors are coaxial. A coaxial layout needs a rotor-interaction model.
* Motor and propeller selection from datasheets **with published thrust-stand tables**: thrust,
  torque, current and RPM versus throttle at a stated voltage. This is the strongest public
  provenance available for Model B, much better than for Model A.
* ESC: current limit, update rate, control mode (throttle vs RPM closed loop).
* Thrust requirements: thrust-to-weight at MTOW (commonly ≥ 2 for heavy-lift [A], to be
  justified), hover throttle, and one-rotor-out margin. Verify all of these analytically in tests.

### 3.2 Mass, CoM, inertia
* A component mass budget with positions, ideally exported from CAD (masses, CoM and inertia
  tensor per part). The existing lumped-box estimator is acceptable only until CAD exists.
* Configurations: empty, full tank, partial tank, hose attached or detached.

### 3.3 Battery and energy
* Cell chemistry, series/parallel count, capacity, datasheet discharge curve, internal
  resistance and temperature range. Parallel packs need a model if more than one is used.

### 3.4 Water system and hose
The brief mentions both "hose" and "changing water mass". The system architecture must be
specified first, because it decides the physics needed:

| Architecture | Physics needed |
|---|---|
| Ground-fed hose (pump on the ground) | Hose weight and drag (hanging mass grows with altitude), tension at the attachment point, internal-flow momentum, nozzle reaction |
| Onboard tank | Mass and inertia change over time, CoM shift, slosh, nozzle reaction |
| Both | All of the above |

Order-of-magnitude illustrations (assumed values, for scoping only):
* water in a 12.7 mm inner-diameter hose weighs 0.127 kg per metre, plus the hose itself;
* a 10 L/min jet through a 4 mm nozzle leaves at about 13 m/s, giving a reaction of about 2.2 N.

Applied off-CoM and horizontally, such a reaction produces a sustained force and moment toward
the facade. Neither effect is negligible for stand-off control.

Data required: hose inner and outer diameter, mass per metre (empty), bending stiffness,
length, attachment point, flow rate, nozzle diameter and direction, tank volume and geometry,
and pump location.

### 3.5 Structural constraints
Arm deflection and frame resonance affect IMU vibration and control bandwidth. Simulate them
only if FEA or test data exist; otherwise record them as design checks (static stress at max
thrust with a safety factor), not as dynamics.

---

## 4. Physics gap analysis

Priority: **P1** affects the research question or the validity of planned experiments;
**P2** improves fidelity measurably; **P3** only if specifically justified.

| # | Model | Current | Needed | Priority | Data / justification |
|---|---|---|---|---|---|
| 1 | 6-DOF quaternion rigid body | done | Variable-mass terms; CoM offset; time-varying inertia | P1 | Required for payload, CoG and water experiments |
| 2 | Rotor geometry | planar, +z_B, quad only | Arbitrary hub positions and thrust axes (cant/dihedral), any n | P1 | Required for any hexacopter |
| 3 | Rotor aerodynamics | `k Ω²` | Inflow-dependent thrust/torque (blade-element momentum or a fitted `C_T(J)` advance-ratio map), rotor H-force / induced drag, translational lift | P1 | Wind experiments; the 59 min > 53 min check |
| 4 | Rotor inertia effects | none | Gyroscopic `J_r Ω × ω`, yaw reaction `J_r Ω̇` | P2 | Larger for 25 in propellers; cheap |
| 5 | Descent aerodynamics | none | Vortex-ring-state boundary as a constraint/flag (not a full model) | P2 | M400 publishes an 8 m/s descent limit |
| 6 | Ground effect | none | Image-rotor (Cheeseman–Bennett form) near the ground | P2 | Take-off, landing, emergency landing |
| 7 | Wall effect | none | Only if the stand-off falls below a few rotor diameters | P3 | At the current 3 m stand-off, distance/diameter is 4.7 (M400) and 11.8 (research quad). Literature to establish the threshold before modelling |
| 8 | Rotor–rotor interaction | none | Only for overlapping or coaxial designs (coaxial efficiency loss) | P3 | Depends on the Model B layout |
| 9 | Motor electrical | first-order speed lag | DC motor `V = R i + k_e Ω`, `k_t i = Q + B Ω + J Ω̇`, current/voltage limits | P1 | Couples battery sag to control authority; datasheets for Model B |
| 10 | ESC | none | Command rate, quantisation, delay, current limit | P2 | |
| 11 | Motor faults | effectiveness and lag | Add stuck/zero output, intermittent output, thermal derating | P1 | Fault experiments |
| 12 | Battery | energy integral | Thevenin 1-RC or 2-RC model: OCV(SoC, T), R₀, R₁C₁, capacity, SoC; voltage sag → max thrust | P1 | Battery-state experiments; energy-aware planning |
| 13 | Atmosphere | ρ fixed | ISA (ISO 2533) T, p, ρ vs altitude with a temperature offset; ρ evaluated at run time in rotor and drag models | P1 | Altitude experiments; barometer model |
| 14 | Mean wind | constant | Height profile (log or power law), direction veer, slow time variation | P1 | |
| 15 | Turbulence | none | Dryden or von Kármán spectra (MIL-F-8785C / MIL-HDBK-1797 forms), seeded | P1 | Applicability to low-altitude near-building flow is limited; label as such |
| 16 | Discrete gusts | none | 1-cosine gusts, direction changes | P1 | |
| 17 | Building-induced flow | none | Precomputed field from RANS/LES CFD (e.g. OpenFOAM) interpolated in the simulator, or simple labelled heuristics | P3 | Expensive, and unvalidated without wind-tunnel data. Only justified if near-facade disturbance is part of the research question |
| 18 | Airframe drag | lumped per axis | Keep it, separate from rotor drag (row 3); report uncertainty range | P2 | |
| 19 | Hose | none | Lumped-mass spring-damper chain with drag and contained fluid; tension at the attachment point; static catenary as a validation case | P1 if hose is in scope | Fluid-conveying pipe instabilities are possible at high flow; check before simplifying |
| 20 | Tank and slosh | none | Equivalent pendulum or spring–mass slosh model | P2 if tank is in scope | |
| 21 | Nozzle jet | none | Reaction `ρ Q v_j` along the nozzle axis, plus mass flow | P1 if spraying is in scope | |
| 22 | Ground contact | none | Spring-damper landing gear with friction | P2 | Emergency landing |
| 23 | Communication link | none | Command-link loss, latency and packet loss; failsafe behaviour | P1 | Comm-loss experiments |
| 24 | Structure | none | Static checks only unless FEA data exist | P3 | |

**Numerics.** Motor electrical time constants (about L/R, typically milliseconds) and stiff
hose segments may force a smaller `dt` or a semi-implicit integrator. Each addition must keep
the existing convergence test and add its own, and must not reduce speed below the campaign
budget (§8.4).

---

## 5. Sensor and camera gap analysis

None of these exist yet. Every sensor must be seeded, rate-correct and timestamped, and every
run must record the sensor configuration.

| Sensor | Model needed | Faults | Notes |
|---|---|---|---|
| IMU (accel, gyro) | White noise, bias instability, bias random walk, scale and misalignment, sample rate, anti-alias filter, rotor-vibration content | Bias jump, stuck, saturation, dropout | Model A IMU unpublished: use a public MEMS datasheet, labelled [A] |
| Magnetometer | Noise, hard/soft iron, **local disturbance near steel/reinforced buildings** | Disturbance, dropout | Missing from the brief but needed for heading |
| GNSS / RTK | Noise per fix type (single, float, fix), 5–10 Hz, latency, **multipath and blockage near the facade** | Dropout, jump, degraded fix | Published M400 accuracies give the check targets |
| Barometer | ISA pressure from the atmosphere model, noise, drift, temperature, propwash error | Drift, offset, dropout | |
| Rangefinder / LiDAR to the facade | Beam model, range limits, incidence-angle and reflectivity dependence | **Dropouts on glass** (specular/transparent surfaces), spurious returns | Directly relevant to glass facades |
| Camera | Pinhole + distortion, FOV, resolution, frame rate, exposure, motion blur from angular rate, rolling shutter, occlusion | Frame drop, blur, glare | Rendered images only where image content matters |
| Estimator | EKF (15–24 states, PX4-EKF2-like structure) consuming the above | — | Required to replace `ideal_truth` |

**Coverage metric.** Facade coverage needs a measurable definition: ground sampling distance,
image overlap, incidence angle and occlusion per facade element. It can be computed
geometrically in Python from the camera model and facade geometry, without rendering. Use
rendered images only when image *quality* is the variable under study.

---

## 6. Environment gap analysis

| Item | Now | Needed | Tool | Feasibility note |
|---|---|---|---|---|
| Building geometry | parameterised boxes (flat, complex) | Windows, mullions, recesses, roof edges; optional import from CAD/BIM | Generator (Python) → SDF + geometry JSON | Keep one geometry source for both Gazebo and Python |
| Glass materials | none | PBR materials (roughness, metalness, transparency), environment map | Gazebo ogre2 | Gazebo reflections are cube-map approximations, not ray-traced. Adequate for visualisation, **not** for claims about optical sensor performance on glass |
| Lighting, shadows | one directional light | Sun position from date/time/location; sky | Gazebo | Image realism claims need a renderer comparison |
| Camera viewpoints | none | Camera sensor on the vehicle, lockstep with the Python state | Gazebo | WSL2 headless GPU rendering is untested here: a risk (§11) |
| Wind, turbulence, atmosphere | constant wind | §4 rows 13–17 | **Python** | Gazebo wind plugins act on Gazebo physics, which is not used for the vehicle |
| Collision geometry | solids with collision; keep-outs visual | Signed distance from the **vehicle envelope** to every solid | Python (from geometry JSON) | Fast and testable; Gazebo not needed |
| Vehicle-size-aware clearance | not implemented | Margin = vehicle radius (incl. rotor tips) + position-error bound + gust displacement bound | Python | The M3 review measured 0.65 m net clearance for the M400 at the complex-world wing corner |
| Safety boundaries | none | Geofence, altitude limits, keep-outs, enforced by planner/supervisor | Python | |

---

## 7. Validation strategy

Fidelity levels, recorded in every run's metadata. Results are never presented above their level:

| Level | Meaning |
|---|---|
| L0 visual | Rendering only (Gazebo scenes) |
| L1 kinematic replay | Gazebo shows Python results |
| L2 physics simulation | Python dynamics + models, with stated parameter provenance |
| L3 software-in-the-loop | Real autopilot software (e.g. PX4) closed around the L2 simulator |
| L4 hardware-in-the-loop | Real flight controller hardware |
| L5 flight test | Physical aircraft; the only level that validates physics |

All work in this project is currently L0–L2.

| Subsystem | Check | Reference | Acceptance (proposed) |
|---|---|---|---|
| Rigid body | Analytic cases (exists) | Analytic | as now |
| Hexacopter allocation | Rank, attainable moment set, controllability after each single-rotor failure | Analytic + controllability literature | Matches published results for the same layouts |
| Rotor aero | `C_T`, `C_P` vs advance ratio | UIUC Propeller Data Site (public wind-tunnel data) for comparable propellers; motor/propeller datasheet tables (Model B) | Within the reported scatter; residuals documented |
| Motor + ESC | Step response, steady current vs thrust | Datasheet thrust-stand tables | Current within ±10 % over the table range (Model B) |
| Battery | Discharge voltage vs time at constant current | Cell datasheet curves | Terminal-voltage RMSE target stated before fitting |
| M400 energy | Hover endurance and flight endurance | 53 min hover, 59 min flight with H30T [V] | Calibrate on one, **check on the other** (not both) |
| M400 envelope | Achievable top speed at 35° tilt, climb 10 m/s | [V] | Consistent within assumed ranges |
| Atmosphere | ISA tables | ISO 2533 | < 0.1 % |
| Turbulence | Simulated PSD vs target spectrum; variance vs intensity | MIL spec forms | PSD within statistical bounds |
| Hose | Static catenary shape and tension | Analytic catenary | < 1 % at convergence; mesh-convergence test |
| Slosh | Natural frequency | Analytic tank formulas | < 2 % |
| Sensors | Allan variance of simulated IMU | Configured noise parameters | Recovered within 10 % |
| EKF | NEES/NIS consistency over Monte Carlo runs | χ² bounds | Within 95 % bounds |
| Closed loop, M400 | Hover error with simulated RTK | Published ±0.1 m | Same order; not a validation of DJI's controller |
| Cross-simulator | Hover/step responses in Python vs another engine | Code-to-code comparison | Labelled *verification*, not validation |
| Future physical | Thrust stand, pendulum inertia, battery discharge, flight logs | Model B prototype | Defines L5; planned, not claimed |

Rules: calibration data and validation data are always separate. Every calibrated parameter
records the data it was fitted to. Uncertainty ranges are propagated by Monte Carlo
(§8.3, stage 4).

---

## 8. Experimental test matrix

### 8.1 Factors (for both aircraft, identical where physically meaningful)

| Factor | Levels (proposed) | Notes |
|---|---|---|
| Aircraft | A, B (and the research quad as a development control) | Comparisons are relative within each aircraft; the two aircraft differ in mass class |
| Payload / mass | empty, nominal, max (as a fraction of each aircraft's capability) | Normalise by MTOW so the aircraft are comparable |
| CoG offset | 0; +x; diagonal (fraction of arm length) | Requires defect 1.3-1 fixed |
| Battery state at start | 100 %, 50 %, 25 % SoC; plus aged (higher R₀) | Requires the battery model |
| Wind | calm; steady 5 and 10 m/s; gust; moderate turbulence; direction change | 4 directions relative to the facade |
| Altitude / atmosphere | ISA sea level; 1500 m; ISA + 20 °C | |
| Sensor faults | IMU bias jump; GNSS dropout/degradation; baro drift; magnetometer disturbance | Onset time and magnitude are randomised per seed |
| Actuator faults | 30 % effectiveness loss; increased lag; single-motor failure | Failure only where the layout keeps controllability |
| Comms | link loss with failsafe | |
| Facade | flat; complex; glass (sensor-relevant effects only) | |
| Trajectory | horizontal raster; vertical scan; repeated coverage; emergency landing | |
| Controller / method | baseline PID; strong external baseline (§9.3); proposed method; ablations | Ablations: remove one component at a time |
| Seeds | ≥ 30 per cell, decided by a power analysis | Separate tuning and evaluation seed sets |

### 8.2 Why a full factorial is impossible

The levels above give 2 × 3 × 3 × 4 × 6 × 4 × 3 × 8 × 3 × 4 × 4 ≈ 2.0 × 10⁶ cells, or
about 6 × 10⁷ runs with 30 seeds. At the measured 5× real time, a 120 s mission takes about 24 s
on one core, so 8 cores do about 1200 runs per hour today: about 6 years for the full factorial.
With the planned fidelity this may fall to roughly 250–400 runs per hour (an assumption to
measure), which means **about two decades**. The campaign must be designed.

### 8.3 Staged design

1. **One-factor-at-a-time sweeps** around a nominal condition, per aircraft (about 10² runs).
   This gives sensitivity and catches modelling errors.
2. **Screening** with a fractional factorial or definitive screening design over all factors
   (about 10³ runs). This identifies which factors and two-way interactions matter.
3. **Focused factorial** on the 3–4 most important factors × methods × 30+ seeds (about 10⁴
   runs, roughly 1–2 days of compute). This is the primary evidence.
4. **Parameter-uncertainty Monte Carlo** (Latin hypercube over the [A] ranges, e.g. C_T, C_P,
   inertia, drag). It tests whether conclusions survive model uncertainty, which matters most
   for Model A.
5. **Pre-registration.** Hypotheses, primary metrics, held-out conditions and analysis plan
   are written before stage 3 is run.

### 8.4 Metrics and statistics

* **Tracking:** RMSE, max error.
* **Safety:** minimum vehicle-envelope clearance; keep-out violation count and duration;
  constraint violations; crash rate.
* **Energy:** energy per mission; remaining SoC; energy attributed to diagnosis/recovery actions.
* **Mission:** coverage % by the geometric metric (§5).
* **Diagnosis:** detection delay, false-alarm rate, missed-detection rate, isolation accuracy.
* **Recovery:** success rate, time to safe state.

Statistics: paired comparisons across matched seeds, 95 % confidence intervals (bootstrap),
effect sizes, multiple-comparison correction, and every run reported. These rules already
exist in docs/experiment_protocol.md and remain binding.

---

## 9. Proposed architecture

### 9.1 Division of responsibilities

| Component | Responsibility | Why this tool |
|---|---|---|
| **Python simulator** (existing, extended) | The only vehicle dynamics; rotor/motor/battery; atmosphere; wind/turbulence; hose and payload; sensor models and faults; estimator; controllers; diagnosis; recovery; clearance; batch campaigns; statistics | Deterministic, seeded, lockstep and testable; already verified; parallelises across cores; no middleware nondeterminism. Keeping one physics engine avoids two inconsistent truths |
| **Gazebo Harmonic** | Rendering only: scenes, glass/lighting, on-board camera images when image content matters; visual replay | Has the camera/depth sensors and the PBR renderer; already integrated (M3). Its physics stays off for the vehicle |
| **Python ↔ Gazebo bridge** | Synchronous lockstep: Python steps → sets pose → requests a rendered frame with the Gazebo server paused/stepped | Replaces after-the-fact replay when images are part of the experiment; keeps Python as the source of truth |
| **ROS 2 Jazzy** | Not needed for the research campaign. Introduce at SIL (L3), for a deployable software stack and for hardware transfer | Adds latency and nondeterminism, and gives no benefit for batch research runs |
| **PX4 SITL (optional, L3)** | A credible external baseline: PX4's controllers and EKF2 flying the Python plant through PX4's simulator interface (sensor messages in, actuator outputs out, lockstep) | Answers "is the baseline credible?" better than a self-tuned PID. Cost: significant integration work; decide after the literature review (D6) |
| **CFD (OpenFOAM), offline, optional** | Precomputed building flow fields, imported as lookup tables | Only if building-induced disturbance is in scope; validation data are scarce |
| **Analysis** | NumPy/SciPy (already installed), CSV per run, one summary table per campaign | No new dependencies required. Parquet/pandas would need `apt` and a recorded decision |

### 9.2 Code structure changes (planned, not implemented)

* `config/aircraft/<name>.yaml` **schema v2**: every value carries
  `source ∈ {manufacturer, datasheet, literature, measured, derived, assumed, unknown}`, a
  citation or URL with access date, units, and an uncertainty range. Hexacopter layouts are
  given as explicit hub positions and thrust axes. A schema validator test rejects any value
  without provenance.
* `dynamics/`: variable-mass rigid body; the CoM offset belongs to the truth state.
* `actuators/`: rotor aero, motor electrical, ESC; a controllability/attainable-set utility.
* `energy/battery.py`, `environment/atmosphere.py`, `environment/wind.py` (profiles,
  turbulence, gusts), `payload/` (tank, slosh, hose, nozzle), `sensors/` (+ faults),
  `estimation/` (EKF), `safety/` (clearance, geofence), `comms/`.
* Run metadata gains `fidelity_level` and the hashes of every parameter file used.

### 9.3 Literature review must precede method choices

Candidate sources to **locate and read**. None has been read yet, and per
docs/literature_review.md nothing may be cited until it has been:
* hexacopter controllability under rotor failure (Du, Quan and Cai, 2015, on PNPNPN vs
  PPNNPN layouts);
* multirotor rotor-drag and induced-drag modelling (Mahony, Kumar and Corke, IEEE RAM 2012);
* ground effect (Cheeseman and Bennett, 1955);
* propeller data (UIUC Propeller Data Site);
* turbulence models (MIL-F-8785C / MIL-HDBK-1797);
* ISA (ISO 2533);
* liquid slosh (Abramson, NASA SP-106);
* fluid-conveying pipe dynamics (Païdoussis);
* PX4 EKF2 and controller documentation;
* the existing review scope on fault detection, active diagnosis, fault-tolerant control,
  energy-aware planning and facade operations.

---

## 10. Phased milestone roadmap

**Critical path** (serves the research question): M3.5 → M4 → M5 → M6 → M7 → M10 → M11 → M12.
**Optional branches**, only if approved: M8 (hose/water), M9 (rendering), M13 (SIL), plus CFD.
Every milestone ends with: all earlier tests passing, the existing baseline reproduced
(regression), and documentation updated.

| Milestone | Content | Acceptance criteria | Automated tests |
|---|---|---|---|
| **M3.5 Decisions and hygiene** | Commit M3; resolve D1–D8; first-hand M400 rotor-count quotation; write the missing validation_report.md; provenance schema v2; `fidelity_level` in metadata; start the literature review (modelling part) | Decisions recorded in docs; schema validator passes on both existing aircraft without changing their values | Schema test: every leaf has source, units and range; baseline regression is bit-identical |
| **M4 Generic multirotor geometry** | Arbitrary n, hub positions and thrust axes; CoM offset in truth; variable mass/inertia; ground contact; Model B v0 from supplied data; Model A per D1 | Quad results unchanged within 1e-9; hexacopter hover equilibrium; controllability table per single-rotor failure matches analytic/literature results | Hover/free-fall/momentum tests for n = 6 with canted rotors; CoM-shift torque test; allocation rank and attainable-set tests; contact energy-dissipation test |
| **M5 Atmosphere and wind** | ISA with offsets; ρ at run time; wind profile; Dryden/von Kármán; 1-cosine gusts; direction changes | ISA within 0.1 %; turbulence PSD and variance within statistical bounds; same seed gives the same field | ISA table test; PSD/variance test; determinism test; rotor/drag ρ-scaling test |
| **M6 Rotor aero, motor, ESC, battery** | Inflow-dependent rotor; H-force; gyroscopic terms; DC motor + ESC; battery equivalent circuit; voltage-dependent max thrust | Model B matches datasheet tables within the stated tolerance; M400 calibrated on hover endurance and checked on flight endurance (reported, no tolerance pre-assumed) | Datasheet-table regression; battery discharge-curve test; energy conservation (electrical = mechanical + losses); dt-convergence test |
| **M7 Sensors, faults, EKF** | IMU, magnetometer, GNSS/RTK, barometer, rangefinder; fault injection; EKF replaces ideal truth | Allan variance recovered within 10 %; NEES/NIS within χ² bounds; controller cannot access truth when `feedback_mode = estimated` | Allan test; consistency Monte Carlo (small, in `slow`); access-isolation test; fault-onset timing tests |
| **M8 Payload: tank, hose, nozzle** *(if approved)* | Slosh; hose chain; nozzle reaction; water depletion | Static catenary < 1 %; slosh frequency < 2 %; momentum balance closes | Catenary, slosh and mesh-convergence tests; mass-flow conservation test |
| **M9 Environment rendering** *(if approved)* | Detailed facades, glass PBR, sun, camera in lockstep; geometric coverage metric; vehicle-size-aware clearance (clearance is on the critical path, rendering is not) | Coverage metric matches hand-computed cases; lockstep frame matches the pose at that timestamp; clearance matches analytic distances | Coverage unit tests; lockstep test (headless); signed-distance tests |
| **M10 Baselines** | Improved PID (feed-forward), a strong literature baseline (e.g. INDI or MPC, chosen from the review), optional PX4 SITL | Every baseline tuned on tuning seeds only; tuning documented | Fairness test: identical inputs for all methods; tuning/evaluation seed separation test |
| **M11 Diagnosis** (original research question) | Method chosen after the literature review | Detection metrics on held-out conditions | Synthetic-fault unit tests; false-alarm test on fault-free runs |
| **M12 Recovery, safe replanning, energy-aware planning** | Recovery, replanning with clearance and energy constraints | Recovery success and safety metrics; no keep-out violation by design in nominal runs | Constraint-satisfaction tests; energy-feasibility tests |
| **M13 SIL** *(optional)* | ROS 2 / PX4 integration of the final stack | Same scenario behaviour as L2 within stated bounds | Integration tests in CI-like scripts |
| **M14 Campaign and statistics** | Stages 1–5 of §8.3 | Pre-registered analysis executed; all runs reported | Reproducibility test: re-running a sample of cells gives identical results |
| **Future (not simulation)** | Thrust stand, inertia pendulum, battery tests, flight tests of the Model B prototype | L5 evidence | — |

---

## 11. Risks, feasibility and required data

### 11.1 Risks

| Risk | Impact | Mitigation |
|---|---|---|
| M400 rotor-count conflict (§0.1) | Model A invalid if built as a hexacopter | Decision D1 before any work |
| Proprietary M400 data | No digital twin is possible | Class B labelling; ranges and uncertainty propagation; claims only relative |
| Model B design not frozen | Rework | Everything config-driven; version the design inputs |
| Scope creep (rendering, CFD, hose) | Research question not reached in the project time | Critical path first; optional branches gated by explicit approval |
| Compute | Campaign infeasible at high fidelity | Staged design; measure speed per milestone; profile; vectorise if needed |
| WSL2 headless GPU rendering | Camera experiments blocked | Early spike test in M9; fall back to the geometric coverage metric |
| Model-form error (aero near buildings, turbulence) | Absolute numbers not trustworthy | Report relative comparisons; sensitivity to model choice; never claim physical validation |
| Stiff payload/motor dynamics | Slower runs or instability | Convergence tests; semi-implicit integration if needed |
| Hexacopter fault tolerance depends on layout | Motor-failure experiments may be uncontrollable by construction | Controllability analysis in M4 before designing those experiments |

### 11.2 Required verified data

**Model A (M400):**
* first-hand rotor count and spin directions (maintenance manual);
* the chosen payload's published specifications (mass, FOV, resolution, frame rate);
* optionally, any published thrust or motor data for the 7510 motor and 2510F propeller;
* physical tests (thrust, inertia, battery, logs) if an aircraft is ever available.

**Model B (team):**
1. Design requirements document: mission, MTOW, endurance, payload, wind limit.
2. Frame geometry: CAD or drawing with hub positions, rotor height and cant angles.
3. Motor, propeller and ESC part numbers with datasheets including thrust tables.
4. Battery chemistry, configuration, capacity and datasheet.
5. Component mass budget with positions, or CAD mass properties.
6. Water-system architecture (tank, hose or both) and all parameters in §3.4.
7. Camera and sensor selection.
8. Structural limits, if known.

**Environment:**
* target building(s) or representative dimensions;
* site altitude and climate range;
* wind statistics, if a real site is intended.

---

## 12. Decisions requiring human approval

| # | Decision | Recommendation |
|---|---|---|
| D1 | Model A rotor count: real M400 quadrotor, a published hexacopter reference, or a renamed hypothetical "M400-class" hexacopter | Keep the M400 as a quadrotor; add a published hexacopter only if a hexacopter-vs-hexacopter comparison is essential |
| D2 | Research scope: the original diagnosis/recovery question with fidelity in support, or a broader digital-twin platform | Keep the research question; fidelity on the critical path only |
| D3 | Model B water system: tank, ground-fed hose, both, or deferred | Decide from the real design; defer hose dynamics if the design is not frozen |
| D4 | Model B rotor ordering (PNPNPN vs fault-tolerant orderings) and cant angles | Decide after the M4 controllability analysis |
| D5 | Rendering scope: geometric coverage metric only, or rendered images (Gazebo) | Geometric metric first; rendering only if image quality is a studied variable |
| D6 | External baseline: PX4 SITL, a literature controller (INDI/MPC) or both | Decide after the literature review |
| D7 | CFD building flow fields: in or out of scope | Out, unless near-facade disturbance becomes part of the research question |
| D8 | Pending from M3: commit M3 work; accept or regenerate the six modified baseline PNGs | Commit M3 after review; decide the PNGs explicitly |
| D9 | Whether to keep `research_quad` as the development control aircraft | Keep it: cheap, fully assumed, useful for regression and method development |
