# DJI Matrice 400 — physical integration plan (preparation only)

**Scope:** simulation and integration preparation. No command has been or will be sent to a real
aircraft from this repository; no live-flight operation is part of this work. All interface statements
cite DJI_MATRICE_400_SOURCE_REGISTER.md; "verified" means read on a DJI source, not tested on hardware.

## 1. Components independent of DJI

Run entirely off-aircraft, with no DJI dependency: mission planning and trajectory generation
(`planning/`), online identification (`sysid/`), diagnosis (`diagnosis/`), active diagnosis
(`active_diagnosis/`), recovery decisions (`recovery/`), energy management (`energy/manager.py`), the
simulator, metrics and analysis. These consume only telemetry-like inputs.

## 2. Components that could communicate with the aircraft

A bridge (not implemented) would map DJI telemetry into `FrameworkInputs` and mission references into DJI
setpoints:

| research input | DJI source (expected) | status |
|---|---|---|
| position, velocity, attitude | fused flight-controller state via MSDK/PSDK telemetry | standard SDK feature; M400 field list and rate **to verify** |
| body rates, accelerations | PSDK/MSDK telemetry (fused); raw IMU not confirmed | **UNKNOWN** for M400 |
| battery V, I, SoC, remaining flight time | battery telemetry; PSDK 3.16 adds remaining flight time / RTH threshold | VERIFIED_SECONDARY (PSDK); fields/rates to verify |
| distance to facade | vision/LiDAR measurement exists (0.4–21 m); SDK access for M400 not confirmed (PSDK point cloud only in a mirror README) | **UNKNOWN** |
| commanded collective thrust / wrench | not available (DJI inner loops) | **NOT PUBLICLY DISCLOSED** → framework must use commanded velocity + reported attitude instead (§7) |

## 3. Official interfaces

| interface | M400 support | evidence | typical use |
|---|---|---|---|
| Mobile SDK V5 (Android) | listed, latest 5.18.0 | DJI GitHub README (VERIFIED_OFFICIAL) | ground app: telemetry, Virtual Stick, waypoint missions, camera |
| Payload SDK | from 3.12.0 (2025-06-26); 3.16.0 (2026-03-31), README badge 3.17.0 | developer-portal extracts (VERIFIED_SECONDARY) | onboard computer (e.g. Manifold 3 on E-Port V2): telemetry, joystick control, payload |
| Cloud API | from 1.14.0 (2025-04-15) | developer-portal extracts (VERIFIED_SECONDARY) | fleet/cloud integration (FlightHub-style), not real-time control |
| DJI Pilot 2 + RC Plus 2 | the operator interface | spec page / release notes | manual flight, missions, RTH reserve setting |

## 4. Versions and firmware (2026-05-07 release)

Aircraft v17.01.05.25, RC Plus 2 v01.64.08.12, DJI Pilot 2 (Android) v17.1.5.14, Manifold 3
v17.00.01.01, TB100 battery v16.00.00.63. SDK/firmware compatibility matrices for these versions must be
confirmed on the DJI developer portal before integration (the portal pages could not be read directly).

## 5. Telemetry that can be obtained (expected, to confirm on hardware)

Fused position/velocity/attitude, flight mode/state, GNSS/RTK status, battery state, gimbal/camera state,
obstacle-sensing status. Rates, latency and timestamp quality on M400 are **UNKNOWN** and must be measured;
they determine whether the 50 Hz framework rate is achievable (it likely is not over MSDK; a 10–25 Hz
framework is plausible and must be re-evaluated in simulation first).

## 6. Commands that can be issued (documented capabilities, M400 behaviour to verify)

* MSDK V5 Virtual Stick: roll/pitch as velocity or angle, yaw as angle or rate, vertical as velocity or
  position; recommended 5–25 Hz; obstacle avoidance only in velocity modes.
* PSDK joystick: velocity/position/thrust modes after obtaining joystick authority; position/velocity
  modes require GNSS health ≥ 3; RC gear change, RC RTH, RC pause and low-battery RTH revoke authority
  (documented for M300/M350).
* Missions: waypoint missions (MSDK/Pilot 2 / Cloud API); take-off, landing, return-to-home commands.

## 7. Can custom trajectory tracking be implemented?

**Yes, at the velocity/position-setpoint level, not at the motor or attitude-loop level.** The research
trajectory generator and recovery logic can stream velocity (or position) setpoints through Virtual Stick
or PSDK joystick at ≤ 25 Hz; DJI's flight controller remains the inner-loop controller. Consequences for
the research framework (to be re-simulated before any hardware work):

* the PID/DOB controllers of this repository become **outer-loop** velocity planners (or are replaced by
  DJI's controller entirely); comparisons A–D on hardware would compare outer loops only;
* thrust-gain and lag identification must use commanded velocity → measured acceleration (a closed-loop
  model including DJI's controller) instead of commanded thrust → specific force;
* actuator-level diagnosis (rotor asymmetry) is only possible through indirect symptoms (attitude bias,
  DJI warnings), not from allocated wrench;
* active probes are expressed as bounded velocity sequences and must respect DJI's own obstacle avoidance
  (T-mode / Cleaning mode use a 1.5 m minimum horizontal distance).

## 8. Not controllable through public interfaces

Per-motor speeds or thrusts, the attitude/rate loops and their gains, the allocation, raw ESC commands,
the onboard estimator and its sensor fusion, DJI's failsafe logic (its parameters can only be set through
the provided settings, e.g. RTH battery reserve).

## 9. Hardware and access required

Matrice 400 + RC Plus 2 with current firmware; a DJI developer account and app key; an Android build
environment for MSDK (and confirmation that MSDK apps run on RC Plus 2 with M400) **or** an onboard
computer (Manifold 3 or a PSDK-supported Linux board) with an E-Port V2 adapter; RTK base (D-RTK 3) for
RTK-level positioning; a safe test site and the operator's flight authorisation.

## 10. Before any real flight

1. Re-run the research framework in simulation with a velocity-setpoint interface model (latency, rate,
   DJI-like inner loop) matching measured SDK behaviour.
2. Bench test: read-only telemetry logging (no commands) to measure rates, latency, timestamp quality.
3. Software-in-the-loop with DJI's simulator facilities (if available for M400 — **UNKNOWN**).
4. Verify authority handover: RC sticks, RTH button and failsafes must always override the software.
5. Hard limits in the bridge: geofence, speed, altitude, minimum facade distance, watchdog that stops
   commanding on telemetry loss; dry runs far from structures.
6. Gather identification data (hover power, step responses) to replace assumptions in the spec.
7. Only then, supervised tests by a qualified pilot, starting with passive diagnosis (no probes).

## 11. DJI RC Plus 2

7.02-inch 1920×1200 1400-nit Android-based controller, 46.8 Wh internal battery (3.8 h) plus WB37 hot-swap
(3.2 h), O4 Enterprise Enhanced link. It is the pilot interface and the probable host of an MSDK app
(**to confirm**). It does not provide unrestricted low-level control; it always retains manual override.
