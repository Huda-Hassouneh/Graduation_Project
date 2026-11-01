# Energy model and energy-aware decisions

Supersedes docs/energy_model.md (Milestone 2 bookkeeping), which remains valid for the
`research_quad` / `m400_reference_quad` baseline runs.

## 1. Quantities kept separate

| quantity | where | definition |
|---|---|---|
| shaft power | plant side | `Σ η_i ρ_r k_m Ω_i³` |
| propulsion electrical power | plant side | shaft power / η (η = 0.80 assumed) |
| avionics / computation power | plant side | 60 W constant (assumed; includes onboard computation, sensors, link) |
| battery electrical energy (truth) | plant side | `∫ V_t I dt` from the equivalent-circuit battery |
| measured electrical energy | framework | `∫ V_meas I_meas dt` from battery telemetry |
| remaining usable energy (estimate) | framework | min( energy implied by reported SoC through the nominal OCV curve down to the usable floor, initial estimate − measured consumption ) |
| predicted power | framework | max( measured 8 s average, nominal hover power / θ^1.5 ) |
| energy for safe termination | framework | predicted power × duration of the safe exit route × 1.15 |
| mission energy by phase | both | buckets `nominal`, `diagnostic` (probe dwells), `recovery` (segments created by a recovery decision), `termination` (abort / safe landing) |

Motor energy and avionics energy are not separable from pack telemetry on a real aircraft (single bus);
the simulator reports both truth and framework estimates so the estimation error can be evaluated
(`energy_remaining_kj` vs `energy_remaining_estimate_kj`).

## 2. DJI-specific calibration and limitations

The DJI power model is not published. The equivalent model is calibrated to the published 53 min hover
claim at 9.74 kg with an assumed 95 % usable energy (C_P = 0.03774; range 0.033–0.045 covers usable
fraction 0.90–1.00 and η 0.75–0.85). Consequences:

* hover power 1050.7 W, about 21.8 A at 48 V (4C limit: 81 A);
* the advertised 59 min flight time is not a simulation endurance; forward-flight efficiency (translational
  lift) is not modelled, so cruise power is over-estimated relative to a real multirotor;
* battery OCV curve, internal resistance and polarisation are generic Li-ion assumptions; temperature
  and ageing enter only as capacity and R0 scale factors.

## 3. How energy influences decisions

* **Recovery** (energy-aware): every decision cycle checks
  `remaining − energy(remaining mission) − reserve ≥ 0` (reserve = 5 % of the usable full-charge energy,
  ≈ 168 kJ, about SoC 10 %). If violated in steady flight, the remaining coverage is truncated to the
  affordable time (re-evaluated at most every 10 s); if even the safe exit is unaffordable, the aircraft
  lands in place.
* **Active diagnosis**: each probe's energy cost enters its utility, and a probe is rejected if the energy
  after it would not cover the safe exit plus reserve (this hard constraint stays on even without
  energy awareness).
* **Stock behaviour** (framework disabled, or `energy_aware = False`): only a low-battery failsafe —
  abort to the exit route when the reported SoC reaches 10 % — analogous to a user-set RTH reserve (the
  M400 has a configurable RTH battery reserve; its exact logic is not published).
* **Model update**: an identified thrust-gain drop raises the predicted power (P ∝ θ^−1.5), which shortens
  the affordable time.

## 4. Logged energy metrics

`energy_total_kj`, `energy_diagnostic_kj`, `energy_recovery_kj`, `energy_remaining_kj` (truth, above the
usable floor), `energy_remaining_estimate_kj` (framework), `soc_end`, `energy_per_coverage_kj`.
