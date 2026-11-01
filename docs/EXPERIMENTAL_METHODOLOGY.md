# Experimental methodology (Phases G, I, J)

## 1. Mission

3D facade coverage (`planning/mission.py`): take-off from the pad at the world origin, climb 2.5 m,
transit to the first pass, three horizontal passes at 3, 6 and 9 m height at a 3 m stand-off from the
facade (inset 1 m from its edges), vertical transitions between passes, then the exit route (retreat to
4 m stand-off, fly back above the pad, descend, slow final descent, land, disarm).
Limits: cruise 1.5 m/s, transit 2.0 m/s, climb 1.0 m/s, descent 0.8 m/s (final 0.3 m/s), acceleration
1.0 m/s². Nominal duration ≈ 100 s. The facade world (`facade_complex`) adds a recessed block, a
protruding wing and two balcony levels; building solids define the vehicle-size-aware clearance used by
the evaluation and the safety rules (nominal minimum clearance ≈ 0.93 m, set by the balcony railings).

## 2. Scenarios (`config/scenarios/*.yaml`)

| scenario | injected change (truth only) | label (evaluation only) |
|---|---|---|
| nominal | none | none |
| wind_gust | 1-cosine gust, 6 m/s toward the facade, 4 s | wind_gust |
| mass_change | mass ×1.20 (inertia scaled) | mass_or_thrust_change |
| drag_change | drag ×3 with a 4 m/s steady cross wind (present from t = 0) | drag_change |
| response_delay | +80 ms rotor-command delay | response_delay |
| sensor_noise | GNSS position noise ×15 | sensor_inconsistency |
| sensor_delay | +0.3 s unreported GNSS latency | gnss_delay |
| sensor_dropout | GNSS position, velocity and heading lost for 20 s | sensor_dropout |
| invalid_measurements | 30 % of GNSS samples NaN or 5 m outliers for 30 s | invalid_measurements |
| actuator_degradation | rotor 1 effectiveness ×0.85 | actuator_degradation_or_com_shift |
| energy_limited | start at 13 % SoC | none |
| compound | turbulent 3 m/s wind + mass ×1.15 + GNSS noise burst | mass_or_thrust_change |
| generalization | unseen: mass ×1.35 later, gust train, rotating wind, aged battery, 1500 m / ISA+10 | mass_or_thrust_change |

Per seed, event onsets are shifted by up to ±8–12 s and magnitudes by up to ±10–20 % (reproducible from
the seed). Turbulence, sensor noise and initial estimator errors are also seeded.

## 3. Conditions and ablations (`experiments/campaign.py`)

| | controller | framework |
|---|---|---|
| A | cascaded PID | disabled (stock low-battery failsafe only) |
| B | DOB-assisted PID | disabled |
| C | cascaded PID | enabled |
| D | DOB-assisted PID | enabled |

Ablations (PID + framework, reference C): `fixed_model`, `periodic`, `passive`, `no_energy`,
`no_replanning` (one switch each, see DIAGNOSIS_AND_RECOVERY.md §5).

Fairness: identical initial conditions, trajectory, disturbance sequences (same seed), safety limits and
evaluation for every condition. Both controllers use the same PID gains (the DOB adds one block). No
per-scenario retuning.

Seeds: development and threshold setting used **seed 0 only**; evaluation uses seeds 1000–1029 (main) and
1000–1009 (ablations). The generalisation scenario uses magnitudes and sequences never used during
development. Thresholds and estimator details were adjusted on seed 0 only, to remove nominal false
alarms and clearly wrong behaviour (the list of such changes is in PROJECT_STATUS.md §3); nothing was
tuned on evaluation seeds.

## 4. Metrics (`metrics/mission.py`)

* Trajectory: position RMSE, MAE, max error, velocity RMSE, attitude error (mean/max), facade stand-off
  error RMSE (coverage phase), estimation error RMSE. Window: airborne phases excluding take-off/landing.
* Diagnosis (labels used only here): detection = an anomaly report within 30 s after onset; detection
  delay; false alarm = any anomaly report before onset (in fault-free scenarios: any report);
  classification correct = a determined report naming the labelled class within 30 s; first-class
  correct; number of "undetermined" reports. Steady wind present from t = 0 (drag_change, compound) is a
  genuine disturbance, so reports before the labelled onset count as false alarms **with respect to the
  label** — interpret with care.
* Recovery: time from onset until position error ≤ 0.3 m for 3 s; maximum deviation after onset;
  safe outcome (completed or landed early, no safety violation, no crash); coverage fraction.
* Energy: total, diagnostic, recovery+termination, remaining (truth) and remaining (framework estimate).
* Safety: minimum vehicle-envelope clearance; violation time for clearance < 0.5 m, speed > 3 m/s
  horizontal or 2 m/s vertical, acceleration > 3 m/s², geofence; crash; landing impact speed.
* Computation: estimator+controller+framework time per 4 ms cycle (mean, p99, max), missed deadlines,
  CPU seconds per simulated second, resident memory. These are Python-prototype numbers on a desktop CPU
  (shared by parallel runs), not embedded-target estimates.

## 5. Statistics (`metrics/stats.py`, `scripts/analyze_campaign.py`)

Mean with 95 % percentile-bootstrap CI (5000 resamples, seeded) per (scenario, condition); paired
differences over matched seeds with bootstrap CI and Wilcoxon signed-rank p-values. With n = 30 a
proportion near 0.5 has a CI half-width of about ±0.18; differences smaller than that are reported as
unresolved. Multiple comparisons are not corrected; p-values are descriptive.

## 6. Reproduction

```bash
python3 scripts/run_campaign.py --suite smoke                    # 52 runs, 1 seed (~15 min on 8 cores)
python3 scripts/run_campaign.py --suite main --seeds 30          # 1560 runs (several hours; resumable)
python3 scripts/run_campaign.py --suite ablations --seeds 10     # 360 runs
python3 scripts/analyze_campaign.py results/campaign/main_30seeds
python3 scripts/analyze_campaign.py results/campaign/ablations_10seeds --reference C
python3 scripts/run_mission.py --scenario mass_change --seed 3   # one run + overview figure
```

Each campaign stores `campaign.json` (scenarios, conditions, seeds, git commit + dirty flag, SHA-256 of
`src/` and `config/`, software versions), `summary.csv`, and per run `metrics.json` + `events.csv`
(time series for the first two seeds of each cell).
