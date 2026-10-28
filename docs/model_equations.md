# Model equations

Frames and sign conventions: [coordinate_frames.md](coordinate_frames.md). Code: `src/graduation_project/`.
Every coefficient below is named in the aircraft YAML files with units and a `source` tag
(`manufacturer` / `standard` / `literature` / `derived` / `assumed`).

## 1. State

`x = [p_W (3), v_W (3), q (4), ω_B (3), Ω (n)]`. The rotor speeds Ω are part of the true
state, so commanded thrust, actual thrust, actuator state and rigid-body response stay separate:

| Quantity | Symbol | Where |
|---|---|---|
| Commanded wrench (controller output) | `u = [T_cmd, τ_cmd]` | `ControlOutput.wrench_cmd` |
| Allocated rotor thrusts (nominal model) | `T_i,alloc` | `AllocationResult.thrusts` |
| Rotor-speed command | `Ω_cmd,i = √(T_i,alloc / k_f,nom)` | `ControlOutput.omega_cmd` |
| Actual rotor speed (lagged, saturated) | `Ω_i` | plant state `x[13:]` |
| Actual thrust (true effectiveness) | `T_i = η_i k_f Ω_i²` | `Wrench.thrusts` |

## 2. Rigid body (`dynamics/multirotor.py`)

The body origin is at the CoM, so translation and rotation decouple:

```
ṗ = v
v̇ = (R(q) F_prop,B + F_drag,W + F_ext,W) / m − g e_z
q̇ = ½ q ⊗ [0, ω_B]
ω̇_B = J⁻¹ (τ_prop,B + τ_ext,B − ω_B × J ω_B)
```

with `m` and `J` (about the CoM, body axes) taken from the *true* mass properties, which
experiments may change at run time. Gyroscopic torques from rotor angular momentum and the
reaction torque from rotor acceleration (`J_rotor Ω̇`) are neglected. This is a documented
simplification, see [limitations.md](limitations.md).

## 3. Propulsion (`actuators/rotor.py`)

### 3.1 Rotor model

```
Ω̇_i = (sat(Ω_cmd,i) − Ω_i) / (τ_m λ_i),   sat(·) = clip(·, Ω_min, Ω_max)
T_i = η_i k_f Ω_i²              (along +z_B)
Q_i = η_i k_m Ω_i²              (reaction torque on airframe: −s_i Q_i z_B)
F_prop,B = [0, 0, Σ T_i]
τ_prop,B = Σ r_i × (T_i e_z) − Σ s_i Q_i e_z = [Σ y_i T_i,  −Σ x_i T_i,  −Σ s_i Q_i]
```

`η_i ∈ [0,1]` (effectiveness) and `λ_i > 0` (lag factor) are hidden truth parameters used to
inject actuator changes. `r_i` are hub positions relative to the CoM.

### 3.2 Coefficients from propeller coefficients (`config.py`)

With rotational speed in rev/s, `n = Ω / 2π`:

```
T = C_T ρ n² D⁴   ⇒ k_f = C_T ρ D⁴ / (2π)²
P = C_P ρ n³ D⁵ = Q Ω   ⇒ k_m = C_P ρ D⁵ / (2π)³
T_max = TWR · m_ref · g / n_rotors,  Ω_max = √(T_max / k_f),  Ω_min = f_idle Ω_max
```

### 3.3 Allocation (`actuators/allocation.py`)

`u = B T`, where row 1 of `B` is all ones, row 2 is `y_i`, row 3 is `−x_i` and row 4 is
`−s_i k_m/k_f`. For the X quadrotor `B` is square and invertible. When `B⁻¹u` violates
`[T_min, T_max]`, yaw torque is scaled down first (bisection on a factor in [0, 1]), then
roll/pitch torque, and anything still violating is clipped. Collective thrust has priority.

## 4. Aerodynamic drag (`environment/aero.py`)

```
v_rel,B = Rᵀ (v_W − w_W)
F_drag,B,j = −c_lin,j v_rel,B,j − c_quad,j |v_rel,B,j| v_rel,B,j,   j ∈ {x, y, z}
F_drag,W = κ_drag R F_drag,B
```

`κ_drag` is a truth multiplier for drag changes. Not modelled: thrust-dependent rotor
(blade-flapping) drag, induced-velocity changes in translational flight, ground effect,
wall effect. Wall effect is deferred to Milestone 4 as an explicitly simplified disturbance.

## 5. Power and energy (`energy/power.py`)

```
P_shaft = Σ η_i k_m Ω_i³
P_el = P_shaft / η_prop + P_avionics
E(t) = ∫₀ᵗ P_el dt       (trapezoidal rule at the plant step)
```

`η_prop` lumps motor and ESC efficiency into a constant. Ideal momentum-theory hover power,
`T^{3/2}/√(2ρA)`, is used only for the figure-of-merit sanity check (0.56 for both configurations).

## 6. Numerical integration

* Classical RK4 with fixed step `dt_sim` (default 4 ms). The rotor-speed command and wind are
  held constant over each step (ZOH). The quaternion is renormalised after each step.
* The controller runs at 250 Hz (every `dt_control / dt_sim` steps).
* The fastest modelled dynamics are motor lag (1/τ_m = 25 s⁻¹ for the research quad) and the
  rate loop (ω_n ≈ 17 rad/s). That gives `λ dt ≈ 0.1`, far inside RK4's stability limit (≈ 2.8).
* Verified empirically. Open-loop error falls more than 10× per halving of dt (fourth order).
  Closed-loop circle trajectories at 4 ms and 1 ms differ by less than 2 mm (`tests/test_closed_loop.py`).
* State validation rejects non-finite states, quaternion drift above 1e-3 per step,
  |ω| > 50 rad/s and |v| > 100 m/s. A run that trips validation is recorded as `failed`.

## 7. Validated properties (unit tests)

| Property | Test | Tolerance |
|---|---|---|
| Hover: Σ T = m g, zero linear and angular acceleration | `test_hover_equilibrium_thrust_equals_weight` | 1e-12 relative |
| Open-loop hover does not drift (2 s) | `test_open_loop_hover_does_not_drift` | 1e-9 m |
| Free fall matches `p₀ + v₀t − ½gt²` | `test_free_fall_matches_analytic_parabola` | 1e-9 m |
| Torque-free asymmetric body conserves world angular momentum and kinetic energy (10 s) | `test_torque_free_rotation_…` | 1e-6 relative |
| Torque sign conventions for every rotor pair | `test_torque_sign_conventions` | sign exact, cross-axis < 1e-9 |
| First-order motor lag reaches 1 − e⁻¹ at t = τ | `test_motor_lag_is_first_order` | 1e-8 relative |
| Speed saturation, lag change, effectiveness change, mass change | `test_dynamics.py` | see tests |
| Fourth-order convergence | `test_rk4_converges_at_fourth_order` | ratio > 10 |
