# Simulation model (DJI Matrice 400 reference quadrotor)

Three distinct things exist; results are never presented above their level:

| level | what | where | evidence it provides |
|---|---|---|---|
| Python physics simulation (**research simulator**) | 6-DOF plant, propulsion, contact, battery, wind, atmosphere, sensors, ESKF, controllers, research framework | `src/graduation_project/` | the only basis for research results |
| Gazebo kinematic replay | static M400 model moved along logged Python poses | `simulation/gazebo/replay_run.py`, world `facade_complex_m400` | visualisation only |
| Gazebo-physics flight model | jointed rotors + gz `MulticopterMotorModel` + gz `MulticopterVelocityControl` | model `dji_matrice_400_flight`, world `m400_flight_test`, `simulation/gazebo/flight_test.py` | secondary check that the SDF (links, joints, inertia, rotor constants) flies in Gazebo's own physics with Gazebo's own controller; **not** evidence about the research framework |

No result has hardware validation. Software-in-the-loop with DJI interfaces has not been performed.

## 1. Frames and conventions

* World W: ENU (x east, y north, z up), origin at the take-off pad, gravity `−g e_z`.
* Body B: FLU (x forward, y left, z up), origin at the CoM. Quaternions Hamilton, scalar first, body→world.
* Rotors: hub positions at azimuths −45°, 45°, 135°, −135° (rotor 1 front-right, numbered counter-clockwise
  from above), radius 0.535 m, 0.0516 m above the CoM. Spin `s = [+1, −1, +1, −1]` (+1 = CCW from above).
* Thrust of rotor i acts along `+z_B` at the hub; reaction torque `−s_i Q_i z_B`.
* The facade (scenario and Gazebo worlds) is the plane y = 8 m, x ∈ [0, 12] m, outward normal −y; the
  aircraft faces it (yaw 90°) during coverage.

## 2. Rigid body and propulsion (`dynamics/multirotor.py`, `actuators/`)

    p' = v
    v' = (R F_prop,B + F_drag,W + F_contact,W + F_ext,W) / m − g e_z
    q' = ½ q ⊗ [0, ω]
    ω' = J⁻¹ (τ_prop + τ_contact + τ_ext − ω × J ω)
    Ω_i' = (sat(Ω_cmd,i(t − d)) − Ω_i) / (τ_m λ_i)

* `T_i = η_i ρ_r k_f Ω_i²`, `Q_i = η_i ρ_r k_m Ω_i²` with effectiveness `η_i` (degradation), density ratio
  `ρ_r` (site altitude/temperature) and the spec-derived `k_f`, `k_m` (SPECIFICATION §3).
* Saturation `[Ω_min, Ω_max · s_V]` where `s_V = min(1, V_battery / V_nominal)` models the loss of thrust
  headroom as the battery voltage sags (equivalent model).
* `d` is an integer-step transport delay (response-delay fault); `λ_i` scales the motor lag.
* Disarmed motors spin down (`Ω' = −Ω/τ_m`).
* Allocation: 4×4 effectiveness matrix of the nominal geometry; saturation priority thrust > roll/pitch > yaw.

## 3. Aerodynamics, atmosphere and wind (`environment/`)

* Airframe drag: per body axis `F_j = −c_j |v_rel,j| v_rel,j`, `v_rel = R^T (v − w)`, `c = [0.107, 0.107, 0.30]`
  N·s²/m² (horizontal derived from the published 25 m/s at 35°, an upper-bound argument).
* ISA troposphere with temperature offset (ISO 2533 relations) for density and barometer pressure.
* Wind = sum of: power-law mean profile with optional rotating direction; seeded Dryden-type turbulence
  (first-order Gauss–Markov per axis with MIL-F-8785C low-altitude intensities and length scales; exact
  for u, an approximation of the second-order v/w spectra); 1-cosine discrete gusts.
* Not modelled: rotor inflow/advance-ratio effects, blade flapping and rotor drag, translational lift,
  ground and wall effects, building-induced flow (ASSUMPTIONS_AND_LIMITATIONS.md).

## 4. Ground contact (`dynamics/contact.py`)

Four skid-end contact points (from the spec's landing-gear assumptions) with a penalty spring-damper
(20 kN/m total, damping ratio 0.7) and regularised Coulomb friction (μ = 0.6). The model starts resting
on the pad (static compression 4.8 mm) and lands on it; the contact force is skipped above the
landing-gear reach (identical results, faster).

## 5. Battery (`energy/battery.py`)

Thevenin model with one RC branch: `V_t = OCV(SoC) − I R0 − V1`, `V1' = I/C1 − V1/(R1 C1)`, coulomb
counting. The current for a requested electrical power solves `P = I (OCV − V1 − I R0)` (smaller root);
above the maximum deliverable power the battery is power-limited. Electrical power = Σ shaft power / η +
avionics power (60 W assumed). Aged-battery scenarios scale capacity and R0.

## 6. Sensors (`sensors/models.py`) and estimator (`estimation/eskf.py`)

| sensor | rate | noise model (parameters in the spec, `simulation_assumptions.sensors`) |
|---|---|---|
| IMU | 250 Hz | white noise + bias random walk + rotor-vibration noise; turn-on bias |
| GNSS position / velocity | 10 Hz, 0.1 s latency with time of validity | RTK-class σ (1.5 / 2 cm), 5 cm/s |
| barometer | 50 Hz | ISA pressure → altitude, σ 0.1 m, bias random walk |
| heading | 10 Hz | σ 1° (published < 2°) |
| facade range | 20 Hz | σ 3 cm, valid 0.4–21 m in front of the facade only |
| battery telemetry | 10 Hz | V, I noise; SoC quantised to 1 % |

Faults per sensor and time window: noise scaling, bias, **unreported** delay, dropout, invalid samples
(NaN or outliers).

The 15-state error-state EKF propagates with the IMU and fuses GNSS position/velocity (delayed-state
handling), barometer, heading and facade range, with chi-square (99.9 %) gating and per-sensor NIS,
staleness and invalid-sample statistics. In closed-loop hover the position estimation error is about
1 cm RMS with NIS in the expected range (`tests/test_estimation_and_sysid.py`).

## 7. Integration and timing

Fixed-step RK4 at 4 ms; the controller and estimator run every step (250 Hz); the research framework at
50 Hz; logging at 25 Hz. Wind and commands are held over a step. Fourth-order convergence and the
analytic properties of Milestones 1–2 are still tested; new tests cover contact rest/lift-off/landing,
command delay, density scaling, ISA values, battery consistency and seeded wind reproducibility.
On the development machine one 100 s mission takes about 30 s on one core (~3.4× real time).

## 8. Verified flight behaviour (Python physics, ESKF feedback, `tests/test_m400_closed_loop.py`)

| behaviour | check (seed 11, nominal scenario) |
|---|---|
| start at rest on the pad | initial height = resting height within 1 cm |
| take-off | climbs > 2 m above the resting height |
| hover | max error < 0.2 m during the take-off hold (DJI publishes ±0.1 m RTK hover, real controller) |
| move in x, y, z | > 5 m range along x and z during coverage, pad-to-stand-off travel in y |
| rotate | turns to face the facade and holds yaw 90° ± 3° |
| track | position RMSE < 0.12 m, estimation RMSE < 0.05 m |
| land | lands back on the pad (< 0.3 m), touchdown speed < 1 m/s, disarms |

## 9. Gazebo models (`simulation/models/`, generated by `simulation/models/generate_models.py`)

* `dji_matrice_400` — static, single link, spec mass/inertia, rotor frames, M400-inspired placeholder
  visuals (body, battery, folding arms, motors, CW/CCW discs, tall gear, front gimbal and FPV camera, four
  vision pods, top LiDAR, dual GNSS antennas) and the translucent published envelope. Used for replay.
* `dji_matrice_400_flight` — base link plus four rotor links on revolute joints; total mass and composite
  inertia equal the spec values (tested). Four `MulticopterMotorModel` plugins use the spec `k_f`,
  `k_m/k_f`, `τ_m`, `Ω_max` and spin directions; `MulticopterVelocityControl` gains are derived from the
  Python PID design (attitude torque gain ≈ J·rate_p·att_p, rate damping ≈ J·rate_p, velocity force gain ≈
  m·K_d). No airframe drag, no estimator, no battery.
* Worlds: `facade_complex_m400` (replay), `m400_flight_test` (flight model on the pad of the flat facade).

Gazebo flight test (`python3 simulation/gazebo/flight_test.py`) results are reported in PROJECT_STATUS.md.
Command durations in that test are wall-clock; when the CPU is loaded Gazebo can run slower than real
time, so displacement thresholds are set to 30–50 % of the commanded integrals (they test behaviour, not
tracking accuracy).
