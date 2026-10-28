# Aircraft reference: DJI Matrice 400 and the research configurations

## 1. Three model classes (do not conflate)

| Class | File | What it is | What it is **not** |
|---|---|---|---|
| **A. Research quadrotor** | `config/aircraft/research_quad.yaml` | A generic 1.5 kg, 450 mm X-quadrotor with entirely assumed parameters. Used to develop and validate algorithms. | A model of any commercial aircraft. |
| **B. Aircraft-reference geometry** | `config/aircraft/m400_reference_quad.yaml` | A quadrotor whose mass, wheelbase, propeller diameter, battery energy and limits come from DJI's published Matrice 400 specifications. Its propulsion, inertia and drag are documented assumptions or derivations. | A digital twin. Its dynamic response is not validated against the real aircraft. |
| **C. Validated digital twin** | — | **Not available.** It would need measured inertia, motor/propeller curves, controller behaviour and flight-log validation. | — |

Results from class A must not be presented as M400 results. Results from class B describe a
*hypothetical* vehicle with M400-like size and mass and must be labelled that way.

## 2. Manufacturer specifications (verified)

Source: DJI Enterprise, *Matrice 400 – Specs*, <https://enterprise.dji.com/matrice-400/specs>,
accessed 2026-10-28 (quotations transcribed through a page-fetch tool). Tolerances are DJI's own.

| Quantity | Published value | Used as | Notes |
|---|---|---|---|
| Dimensions, unfolded (L×W×H, with landing gear) | 980 × 760 × 480 mm | Envelope for the 3D model (Milestone 3) | Not used in dynamics |
| Dimensions, folded (with landing gear and gimbal) | 490 × 490 × 480 mm | — | |
| Diagonal wheelbase | 1070 mm | `arm_length = 0.535 m` (derived) | Hub-to-hub across the diagonal |
| Propeller | 25 in, model 2510F | `rotor_diameter = 0.635 m` | |
| Motor model | 7510 | — | No published torque/speed curve was found |
| Takeoff weight, without batteries (with propellers) | 5020 ± 20 g | consistency check | |
| Takeoff weight, with batteries | 9740 ± 40 g | `total_mass = 9.74 kg` | Excludes payload |
| Max takeoff weight | 15.8 kg | reference mass for the maximum-thrust assumption | |
| Max payload | 6 kg | payload-change range | Measured by DJI at the third gimbal connector at sea level |
| Battery (TB100) | 20254 mAh, 48.23 V, 977 Wh, 4720 ± 20 g | `battery_energy_wh`, battery mass | Mounting position and size assumed |
| Max hover time (no wind) | 53 min | consistency check only | |
| Max flight time | 59 min (with H30T only, 10 670 g total) | not used | |
| Max wind resistance | 12 m/s (takeoff and landing) | operating reference only | Not evidence that the simulated vehicle is safe at 12 m/s |
| Max horizontal speed | 25 m/s (sea level, no wind) | drag derivation (see §4) | |
| Max ascent / descent speed | 10 / 8 m/s | not yet used | |
| Max pitch angle | 35° | `max_tilt_deg` | Used as the tilt limit on both axes |
| Max yaw angular velocity | 100°/s | `max_yaw_rate_deg_s` | Roll/pitch rate limits are not published |
| Hovering accuracy (RTK) | ±0.1 m vertical and horizontal | future comparison reference | Vision: ±0.1 m V, ±0.3 m H; GNSS: ±0.5 m |
| Ingress protection | IP55 | — | |
| Operating temperature | −20 to 50 °C | — | |

Consistency checks run on these numbers: 5.02 kg airframe + 4.72 kg battery = 9.74 kg, which
matches the published takeoff weight exactly.

## 3. Rotor configuration (inferred, partially verified)

* **Rotor count: four (strong secondary evidence, no first-hand quotation).** The DJI spec page
  gives no rotor count. A web search summarising the official *Matrice 400 Maintenance Manual
  v1.0* (<https://dl.djicdn.com/downloads/Matrice_400/MTM/Matrice_400_Maintenance_Manual_v1.0_en.pdf>)
  reports motors numbered 1–4, with motor 1 front-right and numbering counter-clockwise. I
  downloaded the PDF, but could not extract its text in this environment (CID-encoded fonts,
  no poppler). **Action:** read the propeller/motor page of the manual and quote it here.
* **Geometry is consistent with a square X.** For a square X the hub spacing per axis is
  1070 / √2 ≈ 757 mm, which matches the published 760 mm unfolded width. The 980 mm length
  presumably includes forward-mounted gimbal or sensor hardware. This is an inference.
* **Spin directions: unverified.** The same search summary contradicts itself (it gives both
  "CCW propellers on motors 1 and 3" and "motors 1 and 3 rotate clockwise"). The model uses
  `s = [+1, −1, +1, −1]` (rotors 1 and 3 counter-clockwise from above). Diagonal pairs must
  share a direction for yaw balance, so the only open question is the global sign. That sign
  only changes the sign of the yaw allocation row and does not affect any result.

## 4. Assumed and derived parameters (configuration B)

| Parameter | Value | Source | Rationale / sensitivity plan |
|---|---|---|---|
| Thrust coefficient C_T | 0.10 | assumed | Order of magnitude typical of fixed-pitch multirotor propellers. Gives hover at about 2080 rpm, which is plausible for 25 in propellers. Sweep 0.08–0.12. |
| Power coefficient C_P | 0.045 | assumed | Gives a hover figure of merit of 0.56. Sweep 0.035–0.055. |
| Max thrust / weight at MTOW | 1.6 | assumed | 62 N per rotor. Hover at 9.74 kg uses 38.5 % of max thrust. |
| Motor time constant | 0.06 s | assumed | Larger rotors respond more slowly. Sweep 0.03–0.12 s. |
| Idle speed | 15 % of max | assumed | |
| Inertia | diag(0.332, 0.368, 0.646) kg·m² | derived from assumed lumped components | Battery 4.72 kg box (assumed size) at the centre. Motors+propellers 0.40 kg at each hub. Arms 0.25 kg each at half-span. Central body takes the remainder of 9.74 kg. Sweep ±30 %. |
| Horizontal quadratic drag | 0.107 N·s²/m² | derived | `m g tan(35°) / (25 m/s)²`: assumes the published top speed is drag-limited at the published maximum pitch. This is a crude upper-bound argument, since top speed may be power-limited. |
| Vertical quadratic drag | 0.30 N·s²/m² | assumed | |
| Propulsion efficiency (electrical to shaft) | 0.80 | assumed | |
| Avionics power | 60 W | assumed | |

**Hover-endurance consistency check (not validation).** The model predicts a hover electrical
power of 1241 W. At 100 % usable energy, 977 Wh then lasts about 47 min. DJI publishes a
maximum hover time of 53 min, so the model is about 11 % pessimistic. This is plausible for
assumed coefficients, but it does not validate the propulsion model. A 53 min hover would
need about 1106 W (977 Wh / 0.883 h), which could be met, for example, with C_P ≈ 0.040 or an
efficiency of about 0.90.

## 5. Not modelled (and why)

DJI motor curves, ESC dynamics, flight-controller internals and sensor-fusion internals are
proprietary or unpublished. They are not reproduced or guessed beyond the assumptions above.
Raw motor control, hose handling and spraying are outside the research scope.
