# Baseline controller design

Code: `src/graduation_project/controllers/pid.py`. Gains: `config/controllers/pid_baseline.yaml`.

## 1. Structure

Cascaded PID with acceleration feed-forward. It runs at 250 Hz on a `StateEstimate`.
In Milestones 1–2 that estimate is the true state (`feedback_mode = ideal_truth`, recorded in
every run's metadata).

1. **Position/velocity (world):** `a_cmd = a_ref + Kp e_p + Kd e_v + Ki ∫e_p`.
2. **Force and thrust:** `F_des = m_nom (a_cmd + g e_z)`. Two constraints are applied:
   `F_des,z ≥ 0.2 m_nom g`, and the horizontal part is scaled so the tilt of `F_des` stays
   within `max_tilt` (35°). Collective thrust is the projection of `F_des` onto the current
   body z axis.
3. **Desired attitude:** `b3 = F_des/|F_des|`; `b2 = b3 × [cos ψ_ref, sin ψ_ref, 0]` (normalised);
   `b1 = b2 × b3`. Because tilt stays below 90°, `b2` is always well defined.
4. **Attitude (quaternion):** `q_e = q* ⊗ q_des`;
   `ω_des = 2 K_q sign(q_e,w) q_e,vec + Rᵀ ψ̇_ref e_z`. The sign term selects the
   shortest rotation. `ω_des` is clipped to the body-rate limits.
5. **Rate:** `τ = J_nom (K_ω e_ω + K_ωi ∫e_ω) + ω × J_nom ω`.
6. **Allocation:** `[T, τ] → T_i` (prioritised, see model_equations §3.3), then `Ω_cmd,i = √(T_i/k_f,nom)`.

The controller holds only **nominal** parameters (`m_nom`, `J_nom`, `k_f`, `k_m`, geometry
from the aircraft YAML). It never reads the plant's true, possibly modified, parameters. The
tests inject a true mass change and check that the controller has to reject it as a disturbance.

### Anti-windup

* Integrators are frozen while the allocator reports saturation (conditional integration).
* Each integrator is clamped so its contribution stays below a per-axis bound. The position
  bound is `[2, 2, 6] m/s²` for both aircraft. The vertical bound must exceed
  `Δm/m · g` for the largest mass change to be rejected. The M400 maximum payload
  (6 kg on 9.74 kg, +62 %) needs about 6 m/s². A first version used 1 m/s², which left a
  0.26 m steady altitude error after +20 % mass. The test caught it, and the predicted value
  `0.96 / Kp,z = 0.26 m` matched the simulation exactly.

## 2. What is fed forward and what is not

The controller feeds forward reference acceleration and yaw rate. It does **not** feed
forward jerk (body-rate feed-forward) or snap. Tracking a curving path therefore has a
phase-lag error that grows with path curvature and speed. On the 3 m / 2 m/s circle this
dominates the RMSE (see validation_report.md). It is a known, deliberate baseline property.
Differential-flatness feed-forward is the obvious improvement and will be evaluated as a
separate controller, never mixed into the baseline.

## 3. Gain derivation

**Rate loop.** Each body axis with motor lag is modelled as `J ω̇ = τ_actual`,
`τ_m τ̇_actual + τ_actual = τ_cmd`, with `τ_cmd = J K_ω (ω_des − ω)`. The closed loop is
`τ_m s² + s + K_ω = 0`, so `ω_n = √(K_ω/τ_m)` and `ζ = 1/(2√(K_ω τ_m))`.
For ζ ≈ 0.7, `K_ω ≈ 0.51/τ_m`:

| | τ_m | K_ω (roll/pitch) | ω_n (rate) | K_q (attitude) | ratio |
|---|---|---|---|---|---|
| research_quad | 0.04 s | 12 s⁻¹ | 17.3 rad/s | 5.0 s⁻¹ | 3.5 |
| m400_reference_quad | 0.06 s | 8 s⁻¹ | 11.5 rad/s | 3.3 s⁻¹ | 3.5 |

Yaw gains are about half the roll/pitch gains, because yaw authority from reaction torque is
small (`k_m/k_f` ≈ 0.018 m for the research quad, 0.045 m for the M400 reference).

**Position loop.** With ideal inner loops each axis is a triple integrator in `ξ = ∫e`. The
poles are placed at `(s + a)(s² + 2ζω_n s + ω_n²)`:
`Kp = ω_n² + 2ζω_n a`, `Kd = 2ζω_n + a`, `Ki = a ω_n²`
(`position_gains_from_poles`, checked by `test_pole_placement_gains_reproduce_requested_poles`).

| | axis | ω_n | ζ | a | Kp | Kd | Ki |
|---|---|---|---|---|---|---|---|
| research_quad | xy | 1.2 | 0.8 | 0.5 | 2.40 | 2.42 | 0.72 |
| research_quad | z | 1.5 | 0.8 | 0.6 | 3.69 | 3.00 | 1.35 |
| m400_reference_quad | xy | 0.9 | 0.8 | 0.4 | 1.39 | 1.84 | 0.32 |
| m400_reference_quad | z | 1.2 | 0.8 | 0.5 | 2.40 | 2.42 | 0.72 |

**Tuning history (recorded honestly).** The first gain set (`Kp = 1, Kd = 1.8, Ki = 0.1`)
was stable but had an integrator pole near −0.11 s⁻¹. It left 5.7 cm of hover error 10 s
after an offset and gave a 0.20 m circle RMSE. The pole-placement redesign above replaced
it before any comparative experiment was run. The gains have not been optimised for any
scenario and are fixed for all experimental conditions on a given aircraft.

## 4. Fairness rules for later comparisons

Every method is compared on the same aircraft file, initial conditions, reference
trajectory, disturbance realisations (seeds), `dt_sim` and control rate. No method is tuned
on the test conditions. Tuning and evaluation scenario sets will be kept separate
(experiment_protocol.md).
