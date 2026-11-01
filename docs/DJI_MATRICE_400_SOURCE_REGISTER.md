# DJI Matrice 400 — source register

Every parameter in `config/aircraft/dji_matrice_400.yaml` cites one or more of the source ids below
(enforced by `tests/test_m400_spec.py`). Access dates are the dates on which the content was read in this
project. "How read" states exactly what was seen, because several official pages are JavaScript-rendered
or are PDFs that could not be machine-read here.

| id | title | publisher | URL | accessed | type | how read / reliability |
|---|---|---|---|---|---|---|
| DJI-SPEC | Matrice 400 – Specs | DJI | https://enterprise.dji.com/matrice-400/specs | 2026-10-28, 2026-10-30, 2026-11-01 | official | Page fetched and summarised by a fetch tool (values reproduced with units). Primary source for mass, dimensions, propeller, motor model, battery, sensing, transmission, RC Plus 2 and limits. Does **not** state the rotor count, motor ratings, IMU, ESC, CoM or inertia. |
| DJI-RN | Matrice 400 Release Notes (released 2026.05.07) | DJI | https://dl.djicdn.com/downloads/Matrice_400/RN/20260507/Matrice_400_Release_Notes_en.pdf | 2026-11-01 | official | PDF text extracted locally (zlib-inflated content streams). Firmware versions, T-mode / Cleaning mode, RTH battery reserve, Delivery mode via PSDK, TB100/TB100C firmware. 11 release dates from 2025-06-10 to 2026-05-07. |
| DJI-MTM | Matrice 400 Maintenance Manual v1.0 | DJI | https://dl.djicdn.com/downloads/Matrice_400/MTM/Matrice_400_Maintenance_Manual_v1.0_en.pdf | 2026-10-28 (download), 2026-11-01 (retry) | secondary | The PDF uses CID-encoded fonts; neither the local extractor nor the fetch tool could read it. Its content (motors numbered 1–4, motor 1 front-right, numbering counter-clockwise) is known only from a **search-engine summary** that also contradicts itself on spin directions. Treated as secondary evidence. |
| DJI-STORE-PROP | DJI Matrice 400 2510F Propeller | DJI Store | https://store.dji.com/hu/product/dji-matrice-m400-2510f-propeller | 2026-10-30 | official | Seen via search results: 25 in folding propeller sold as a CW/CCW pair, compatible only with M400. |
| DJI-MSDK-GH | DJI Mobile SDK for Android V5 (README) | DJI (dji-sdk GitHub organisation) | https://github.com/dji-sdk/Mobile-SDK-Android-V5 | 2026-11-01 | official | Fetched. "Supported Product" list includes **Matrice 400**; latest version 5.18.0. Remote controllers are not listed. |
| DJI-MSDK-VS | MSDK V5 API reference: VirtualStickFlightControlParam / IVirtualStickManager | DJI | https://developer.dji.com/api-reference-v5/android-api/Components/IVirtualStickManager/Value_FlightController_Struct_VirtualStickFlightControlParam.html | 2026-11-01 | secondary | JavaScript-rendered; content known from search-engine extracts of the official page: roll/pitch VELOCITY or ANGLE, yaw ANGLE or ANGULAR_VELOCITY, vertical VELOCITY or POSITION, recommended send rate 5–25 Hz, obstacle avoidance only in velocity modes. Generic V5 behaviour; **not confirmed for M400**. |
| DJI-DEV-RN | DJI Developer portal release entries (PSDK, Cloud API, MSDK) | DJI | https://developer.dji.com | 2026-11-01 | secondary | JavaScript-rendered; search-engine extracts of the official portal: PSDK 3.12.0 (2025-06-26) adds Matrice 400; PSDK 3.16.0 (2026-03-31) lists M400 with remaining-flight-time / low-battery-RTH queries; Cloud API 1.14.0 (2025-04-15) adds Matrice 400. |
| DJI-PSDK-GH | DJI Payload SDK (GitHub README, V3.17.0 badge) and `dji_flight_controller.h` | DJI (dji-sdk GitHub organisation) | https://github.com/dji-sdk/Payload-SDK | 2026-11-01 | official | README fetched (no aircraft list in it). Header/guide extracts: joystick authority must be obtained; position/velocity joystick modes need GPS health ≥ 3; RC gear change, RC RTH, RC pause and low-battery RTH revoke authority (**documented for M300/M350**, not stated for M400). |
| ISO-2533 | ISO 2533:1975 Standard Atmosphere | ISO | https://www.iso.org/standard/7472.html | 2026-11-01 | standard | Standard troposphere relations used in `environment/atmosphere.py`; one table value (1000 m pressure) checked in `tests/test_m400_physics.py`. Full tables to be checked in the literature review. |
| DERIVED | Derivations in this repository | this project | docs/DJI_MATRICE_400_TECHNICAL_SPECIFICATION.md | 2026-11-01 | derived | Arithmetic from cited values; each derived leaf states its `derivation`. |
| ENG | Engineering judgement | this project | docs/ASSUMPTIONS_AND_LIMITATIONS.md | 2026-11-01 | assumption | Each leaf states `rationale`, `range` and `sensitivity`. Never presented as manufacturer data. |

## Items searched for and NOT found (recorded as UNKNOWN / NOT_PUBLICLY_DISCLOSED)

* A first-hand statement of the rotor count and spin directions (manual not machine-readable here).
* Motor manufacturer, KV, resistance, torque constant, ESC, maximum thrust, RPM, propeller pitch.
* IMU, barometer and compass specifications; raw-sensor access through the SDKs on M400.
* CoM position, inertia tensor, battery dimensions, internal resistance, cell format.
* The M400 user manual (RTH / low-battery / signal-loss failsafe defaults, flight-mode names).
* Whether MSDK V5 runs on the DJI RC Plus 2 with M400, and M400-specific Virtual Stick / joystick limits.
* Video-link latency.

## Related secondary results seen but not used as values

Retailer listings and third-party SDK mirrors (e.g. GitHub forks of the Payload SDK README mentioning
LiDAR/mmWave point-cloud subscription on M400) were seen during the search. They are not cited as values;
where they motivated an UNKNOWN entry this is stated in its note.
