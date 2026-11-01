# DJI Matrice 400 — technical specification (company platform, Path A)

**Status:** documented from public sources on 2026-11-01. The aircraft has not been delivered and no
measurement of it has been made. The simulation model built from this document is a **reference
quadrotor model, not a digital twin**: DJI does not publish the propulsion curves, inertia, centre of
gravity, IMU specifications or flight-control internals that a digital twin would require.

* Machine-readable database: `config/aircraft/dji_matrice_400.yaml` (162 parameters, schema v2) and its
  export `config/aircraft/dji_matrice_400_parameters.json`.
* Full generated table: [DJI_MATRICE_400_PARAMETER_TABLE.md](DJI_MATRICE_400_PARAMETER_TABLE.md).
* Sources: [DJI_MATRICE_400_SOURCE_REGISTER.md](DJI_MATRICE_400_SOURCE_REGISTER.md).
* Validation: `python3 scripts/export_m400_spec.py --check` and `tests/test_m400_spec.py`.

| category | meaning | count |
|---|---|---|
| VERIFIED_OFFICIAL | read on a DJI page/document | 73 |
| VERIFIED_SECONDARY | DJI content seen only indirectly, or derived unambiguously from official values | 8 |
| ENGINEERING_ESTIMATE | value with documented derivation/rationale, range and sensitivity | 16 |
| SIMULATION_ASSUMPTION | modelling choice with no manufacturer counterpart (range, rationale, sensitivity) | 45 |
| UNKNOWN | searched, not found (`value: null`) | 6 |
| NOT_PUBLICLY_DISCLOSED | proprietary / not published (`value: null`) | 14 |

The validator rejects any UNKNOWN or NOT_PUBLICLY_DISCLOSED value that is not `null`, any estimate
without range, rationale and sensitivity, and any source id not in the register. A null parameter that
the simulation needs names its stand-in (`simulation_value`), so no specification is silently replaced.

## 1. Configuration and geometry

| quantity | value | category |
|---|---|---|
| Configuration | four-rotor symmetric X, foldable arms | VERIFIED_SECONDARY |
| Diagonal wheelbase | 1070 mm | VERIFIED_OFFICIAL |
| Unfolded L×W×H (with landing gear) | 980 × 760 × 480 mm | VERIFIED_OFFICIAL |
| Folded L×W×H (with landing gear and gimbal) | 490 × 490 × 480 mm | VERIFIED_OFFICIAL |
| Hub radius (arm length) | 535 mm (= wheelbase / 2) | ENGINEERING_ESTIMATE |
| Rotor numbering | 1 front-right, counter-clockwise from above | SIMULATION_ASSUMPTION (secondary evidence) |
| Spin directions | [+1, −1, +1, −1] (diagonals equal) | SIMULATION_ASSUMPTION (unverified sign) |
| Rotor plane height above CoM frame | 0.05 m (0–0.12) | ENGINEERING_ESTIMATE |
| Landing gear | bottom −0.33 m in G, track 0.50 m, skid 0.60 m | SIMULATION_ASSUMPTION |
| Frontal reference area, CoG | not published | NOT_PUBLICLY_DISCLOSED |

**Rotor count.** The specification page gives no rotor count. Two independent arguments support four:
(i) the maintenance-manual summary numbers motors 1–4; (ii) geometry — a square X with a 1070 mm
diagonal places adjacent hubs 757 mm apart, matching the published 760 mm width, whereas a coplanar
hexagon would place adjacent hubs 535 mm apart, less than the 635 mm disc diameter. The company also
describes the aircraft as a quadrotor. A first-hand quotation from the manual is still pending.

**Envelope consistency.** With 45 mm motor bells the generated airframe extent is 913 × 847 × 480 mm.
The width exceeds the published 760 mm because the motor housings at 757 mm hub spacing protrude; the
published length (980 mm) presumably includes front payload hardware. This is a consequence of the
symmetric-X assumption and placeholder shapes, not a claimed property of the aircraft.

## 2. Mass properties

| quantity | value | category |
|---|---|---|
| Take-off mass with battery (with propellers) | 9.74 kg (±0.04) | VERIFIED_OFFICIAL |
| Mass without battery | 5.02 kg (±0.02) | VERIFIED_OFFICIAL |
| TB100 battery mass | 4.72 kg (±0.02) | VERIFIED_OFFICIAL |
| Maximum take-off mass | 15.8 kg | VERIFIED_OFFICIAL |
| Maximum payload (documentation only) | 6 kg at the third gimbal connector, sea level; decreases with altitude | VERIFIED_OFFICIAL |
| Simulation baseline mass | 9.74 kg (one TB100, no payload) | VERIFIED_OFFICIAL |
| Inertia | diag(0.332, 0.368, 0.646) kg·m² from assumed lumped components, ±30 % | NOT_PUBLICLY_DISCLOSED → ENGINEERING_ESTIMATE |
| CoM | follows from the assumed layout (−1.6 mm below the geometric frame) | NOT_PUBLICLY_DISCLOSED → assumption |

Consistency: 5.02 + 4.72 = 9.74 kg exactly; therefore one TB100 per aircraft (VERIFIED_SECONDARY).
These masses are kept distinct: aircraft mass ≠ MTOW ≠ payload ≠ battery ≠ accessories.

## 3. Propulsion

| quantity | value | category |
|---|---|---|
| Motors | 4, model "7510" | VERIFIED_SECONDARY (count), VERIFIED_OFFICIAL (model) |
| Motor manufacturer, KV, R, torque constant, ESC, max thrust, RPM, propeller pitch | not published | NOT_PUBLICLY_DISCLOSED |
| Propeller | 2510F, 25 in (0.635 m), folding, CW/CCW pairs | VERIFIED_OFFICIAL |
| Propulsion voltage | 13S Li-ion, 48.23 V nominal | VERIFIED_OFFICIAL |
| Thrust coefficient C_T | 0.10 (0.08–0.12) | ENGINEERING_ESTIMATE |
| Power coefficient C_P | 0.03774 (0.033–0.045), **calibrated** | ENGINEERING_ESTIMATE |
| Thrust-to-weight at MTOW | 1.6 (1.4–2.2) | ENGINEERING_ESTIMATE |
| Motor time constant | 0.06 s (0.03–0.12) | ENGINEERING_ESTIMATE |
| Electrical-to-shaft efficiency | 0.80 (0.70–0.90) | ENGINEERING_ESTIMATE |

**Equivalent propulsion model.** Because no DJI motor data exist, thrust `T = k_f Ω²` and torque
`Q = k_m Ω²` use `k_f = C_T ρ D⁴ / (2π)²` and `k_m = C_P ρ D⁵ / (2π)³`. C_P is calibrated
(`scripts/calibrate_m400_power.py`) so that the modelled hover electrical power equals
0.95 × 977 Wh / 53 min = 1050.7 W at 9.74 kg. Derived values: hover 2078 rpm, maximum 3347 rpm,
62.0 N maximum thrust per rotor (hover uses 38.5 %), hover figure of merit 0.67. The calibration uses a
manufacturer **claim**; it cannot also validate the model.

## 4. Battery and energy

| quantity | value | category |
|---|---|---|
| Battery | TB100 Intelligent Flight Battery, Li-ion 13S | VERIFIED_OFFICIAL |
| Capacity / energy | 20 254 mAh / 977 Wh | VERIFIED_OFFICIAL |
| Nominal / max charge voltage | 48.23 V / 54.6 V | VERIFIED_OFFICIAL |
| Discharge / charge rate | 4C / 2C (max), 400 cycles | VERIFIED_OFFICIAL |
| Temperatures | discharge −20…75 °C, charge 5…45 °C, self-heating | VERIFIED_OFFICIAL |
| Battery firmware | v16.00.00.63; tethered TB100C exists | VERIFIED_OFFICIAL |
| Station | BS100: 11.8 kg, charges 3 TB100 + 2 WB37, ~45 min fast / ~110 min silent | VERIFIED_OFFICIAL |
| Dimensions, parallel count, internal resistance, OCV curve | not published | UNKNOWN / NOT_PUBLICLY_DISCLOSED |
| Max flight time / hover time / distance | 59 min / 53 min / 49 km (no wind; manufacturer claims) | VERIFIED_OFFICIAL |
| Equivalent circuit | R0 = 0.06 Ω (0.03–0.12), 1-RC 0.03 Ω / 2000 F, generic Li-ion OCV, 3.0 V cutoff | ENGINEERING_ESTIMATE / SIMULATION_ASSUMPTION |

The OCV curve integrates to 985 Wh (published 977 Wh, +0.8 %). **Endurance claims are not simulation
guarantees**: a constant-hover discharge of the model reaches cutoff after 52.6 min only because C_P
was calibrated to the 53 min claim; mission endurance in the simulator includes wind, manoeuvres,
probes and a 5 % usable-energy reserve. The 59 min flight-time figure involves forward flight
(translational lift), which the model does not represent, so it cannot be used as an independent check.

## 5. Sensors and navigation

| sensor | published | raw access via interfaces |
|---|---|---|
| GNSS | GPS + Galileo + BeiDou (+ GLONASS with RTK); RTK fix 1 cm + 1 ppm H, 1.5 cm + 1 ppm V | fused position/velocity telemetry; raw observables not confirmed |
| RTK heading | better than 2° (dual antenna) | fused attitude/heading |
| Hover accuracy | RTK ±0.1 m H/V; vision ±0.1 m V, ±0.3 m H; GNSS ±0.5 m | closed-loop figure (DJI controller) |
| Vision | omnidirectional binocular fisheye; measurement 0.4–21 m (fwd/bwd), 0.6–21 m (lateral), 0.5–19 m (down) | not confirmed for M400 |
| Rotating LiDAR | 0.5–100 m, 360° × 58°, 520 000 pts/s, 905 nm Class 1 | PSDK point-cloud subscription reported only by a mirror README → UNKNOWN |
| Upper LiDAR / downward IR | 0.5–25 m ToF / 0.3–8 m, 60° FOV | not confirmed |
| mmWave radar | six directions; power-line detection 36 m (12 mm) / 50 m (21.6 mm), ±45° | not confirmed |
| FPV camera | 1080p/30, 150° DFOV, starlight | video stream via SDK (standard feature) |
| ADS-B In | dual antenna, up to 20 km | — |
| IMU, barometer | not published | NOT_PUBLICLY_DISCLOSED |
| Compass | not stated | UNKNOWN |

Simulated sensors (`sensors/models.py`) use the published accuracies where they exist (RTK position,
heading) and class-representative MEMS values elsewhere, all labelled SIMULATION_ASSUMPTION. The
"facade range" sensor is a simulation abstraction of a distance-to-facade measurement within the
published 0.4–21 m vision/LiDAR range; whether such a measurement is available through a DJI interface is
UNKNOWN (DJI_MATRICE_400_INTEGRATION_PLAN.md).

## 6. Flight control and communication

| item | value | category |
|---|---|---|
| Remote controller | DJI RC Plus 2 Enterprise Enhanced: 7.02 in 1920×1200 1400-nit screen, 46.8 Wh internal (3.8 h), WB37 external (3.2 h), 1.15 kg, −20…50 °C, 8 GB/128 GB | VERIFIED_OFFICIAL |
| Video link | O4 Enterprise Enhanced, 3 × 1080p/30, 40 km FCC / 20 km CE, 80 Mbps | VERIFIED_OFFICIAL |
| Firmware (2026-05-07) | aircraft v17.01.05.25, RC v01.64.08.12, Pilot 2 v17.1.5.14, Manifold 3 v17.00.01.01 | VERIFIED_OFFICIAL |
| Mobile SDK V5 | M400 listed (5.18.0) | VERIFIED_OFFICIAL |
| Payload SDK | M400 from 3.12.0; 3.16.0 adds remaining-flight-time / RTH-threshold queries | VERIFIED_SECONDARY |
| Cloud API | M400 from 1.14.0 | VERIFIED_SECONDARY |
| Virtual Stick (MSDK V5) | velocity/angle (roll, pitch), angle/rate (yaw), velocity/position (vertical), 5–25 Hz | VERIFIED_SECONDARY (generic; M400 specifics unknown) |
| Low-level motor / attitude-loop access | not offered by any reviewed public interface | NOT_PUBLICLY_DISCLOSED |
| T-mode / Cleaning mode | close-range tasks: min 0.5 m/s, min horizontal obstacle distance 1.5 m; Cleaning mode uses mmWave radar only | VERIFIED_OFFICIAL |
| RTH battery reserve, Delivery mode via PSDK | present | VERIFIED_OFFICIAL |
| Failsafe defaults, flight-mode names | user manual not obtained | UNKNOWN |

## 7. Environmental and performance limits

| quantity | value | category |
|---|---|---|
| Operating temperature | −20…50 °C (no solar radiation) | VERIFIED_OFFICIAL |
| Ingress protection | IP55 | VERIFIED_OFFICIAL |
| Wind resistance | 12 m/s (take-off and landing) | VERIFIED_OFFICIAL — operating reference, not a simulated guarantee |
| Max take-off altitude | 7000 m | VERIFIED_OFFICIAL |
| Max speeds | 25 m/s horizontal, 10 m/s ascent, 8 m/s descent | VERIFIED_OFFICIAL |
| Max pitch / yaw rate | 35° / 100 °/s | VERIFIED_OFFICIAL |
| Roll/pitch rate limit | 120 °/s (60–200) | SIMULATION_ASSUMPTION |

## 8. What would turn this into a validated model

Thrust-stand data for the 7510/2510F set (k_f, k_m, τ, current), a pendulum inertia measurement, a CoM
measurement, battery discharge curves (OCV, R0), and logged flights with DJI telemetry (hover power,
step responses). See ASSUMPTIONS_AND_LIMITATIONS.md §6 and DJI_MATRICE_400_INTEGRATION_PLAN.md §10.
