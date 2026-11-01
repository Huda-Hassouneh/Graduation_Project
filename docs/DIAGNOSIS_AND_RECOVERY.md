# Diagnosis, active diagnosis and safe recovery

Research question: *how can a quadrotor determine when its current model or measurements are no longer
sufficiently reliable, obtain additional information when necessary, and select an appropriate recovery
action while respecting safety and energy constraints?* This document describes the implemented
mechanisms. No novelty is claimed: the literature review (docs/literature_review.md) has not been
conducted, and every method here is a standard building block (RLS, CUSUM, chi-square gating,
Bayesian grid posteriors, Fisher-information experiment design) combined for this application.

## 1. Online identification (`sysid/online.py`) and identifiability

| parameter | model | identifiable when | not separable from |
|---|---|---|---|
| thrust gain θ | `f_z = θ · LPF_τ(T_cmd) / m_nom` (RLS, 4 s memory) | always in flight | **mass vs. uniform thrust loss** (θ = η m_nom/m) |
| effective lag τ_eff | grid posterior over τ, per-candidate least-squares gain (shape-only likelihood); valid only if fit ratio ≤ 2 and excitation ≥ 3 | thrust varies at ~0.3–3 Hz (manoeuvres, vertical probe) | motor lag vs. pure transport delay (both increase τ_eff) |
| drag c and wind w (per horizontal axis) | `a = −c (v−w)|v−w|` on a wind grid; residual-scaled noise + accelerometer-bias floor | ground speed varies (passes, speed probe) | at constant speed any (c, w) on `c(v−w)² = F` fits; a residual accelerometer bias mimics wind |
| GNSS latency δ | `r_GNSS = −v δ` on innovations ≤ 1.5 m; R² ≥ 0.5 required | ground speed > ~0.3 m/s | noise bursts (rejected by the R² test) |

Each estimator reports its uncertainty and returns to its prior bound without excitation, so the framework
can say "not identifiable now" instead of producing a confident wrong value. The drag/wind estimator is
the weakest: the M400 drag acceleration at 3–4 m/s airspeed (~0.13 m/s²) is comparable to residual
accelerometer bias (~0.02 m/s²) and model mismatch.

## 2. Residuals, detection and hypotheses (`diagnosis/diagnoser.py`)

Residual channels, each normalised so nominal flight gives z < 1: thrust gain vs. expected (site density
known to the operator), roll/pitch and yaw torque disturbance `τ_d = J ω̇ + ω×Jω − τ_cmd,lagged`, horizontal
external force from the accelerometer, tracking error, identified lag (only when valid), GNSS latency
(only when significant), and sensor health (staleness, invalid samples, NIS consistency).

* Detection: one-sided CUSUM per channel (`g = max(0, g + z − 1)`, alarm at g > 12).
* Persistence: an alarm lasting ≥ 4 s is PERSISTENT; one that clears earlier is TRANSIENT.
* Hypotheses (evidence → probability with an "unexplained" mass): wind gust, steady wind, drag change,
  mass-or-thrust change, actuator-degradation-or-CoM-shift, response delay, GNSS delay, sensor dropout,
  invalid measurements, sensor inconsistency, unexplained.
* A report is DETERMINED only if the best probability ≥ 0.6 and exceeds the runner-up by ≥ 0.25;
  otherwise it states: **"An abnormal change was detected, but its cause cannot be determined
  confidently."** Grouped hypotheses (mass-or-thrust, degradation-or-CoM-shift) are deliberate: the
  available signals cannot separate their members.
* Temporary vs persistent disturbance: transient force channel → gust; persistent → steady wind or drag
  change (separable only with speed variation). Sensor noise/delay/missing/invalid: from estimator
  statistics, latency regression, staleness and validity checks. Actuator degradation: persistent torque
  residual (asymmetric loss), with a thrust-gain drop as supporting evidence.

## 3. Active diagnosis (`active_diagnosis/`)

Candidate probes are bounded, smooth offsets applied during an inserted dwell (no reference jump):
vertical excitation (0.10 m, 0.5 Hz, 6 s), along-facade speed excitation (0.8 m, 0.15 Hz, 12 s) and
hover-and-observe (5 s). For each candidate:

* **expected information gain** `IG = Σ_h P(h) · relevance(probe, h) · IG_param` [nats], with RLS
  information `½ log det(P_prior P_post⁻¹)` for θ, (c, w) and δ, the expected entropy reduction of the lag
  posterior, and — the only heuristic term — `ln 2 (1 − e^(−T/3 s))` for the transient-vs-persistent
  question of hover-and-observe;
* **energy cost** `ΔE = P_pred T (1 + 3/8 δ²)` (momentum-theory P ∝ T^1.5 averaged over the sinusoid);
* **utility** `U = IG − λ_E ΔE[kJ] − λ_T T` (λ_E = 0.02 nat/kJ, λ_T = 0.01 nat/s; λ_E = 0 without energy awareness);
* **hard constraints** (any violation rejects): facade clearance ≥ 0.5 m beyond vehicle radius
  (0.853 m) and 3σ position uncertainty; altitude ≥ 2 m; along-facade excursion inside the facade;
  peak speed ≤ 2 m/s and acceleration ≤ 2 m/s² (conservative bounds); hover throttle ≤ 0.70 and peak
  throttle ≤ 0.95 for vertical excitation; energy after the probe still covers the safe exit route plus
  reserve; allowed phase; 12 s cooldown; at most 3 probes per mission.

A probe runs only if IG ≥ 0.25 nats and U > 0. Rejections and their reasons are logged (deduplicated).

## 4. Safe recovery and replanning (`recovery/supervisor.py`, `planning/mission.py`)

Actions: continue, update model (controller thrust gain; estimator GNSS-latency compensation), reduce
speed (re-time the same path), modify trajectory (stand-off +0.5/1.0 m, max +2 m), diagnostic manoeuvre
(request to §3), replan remaining (drop passes that are not affordable), abort (safe exit route to the
pad), safe termination (retreat if needed, descend, land in place).

Explicit safety constraints (not penalty weights):

1. All references respect speed/acceleration limits **by construction** (segment time-scaling).
2. Replanned waypoints are **projected** to the minimum stand-off (vehicle radius + 0.5 m) from the facade.
3. Safety monitor rules override diagnosis: clearance below the margin → retreat (or terminate without
   replanning); position σ > 0.3 m → slow down and widen stand-off; σ > 1.0 m → land in place.
4. Energy: affordable-time check against remaining usable energy minus a reserve (§ ENERGY_MODEL.md);
   infeasible exit → land now; infeasible mission → truncate or abort.
5. Probes obey the hard constraints of §3.

Assumptions behind these safety statements: the building map is known and correct; the estimator
covariance is representative (verified in hover, not under all faults); the vehicle radius covers the
rotors; the speed/acceleration limits are achievable by the controller; the 0.5 m margin covers transient
deviations — **which the 15 %+ rotor-loss runs show is not always true** (minimum clearance 0.37–0.47 m in
the first seconds after onset, before any recovery can act). No formal guarantee (reachability, CBF) is
provided.

## 5. Ablation switches (`framework/core.py`)

`adaptation` (fixed model vs online update), `diagnosis_mode` (event-triggered vs periodic every 10 s with
scheduled probes), `active` (passive vs active), `energy_aware`, `replanning`. Results:
EXPERIMENTAL_METHODOLOGY.md and PROJECT_STATUS.md.
