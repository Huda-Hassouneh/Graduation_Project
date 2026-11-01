# Assumptions and limitations

Applies to the DJI Matrice 400 company-platform work. Earlier limitations of the research-quad baseline
remain in docs/limitations.md.

## 1. Evidence level

* Everything is **simulation**. No physical validation, no hardware-in-the-loop, no software-in-the-loop
  with DJI interfaces. The Gazebo flight model is a secondary check in another simulator, not validation.
* The M400 model is a reference model consistent with published mass, geometry, battery and limits; it
  is **not a digital twin**. Propulsion, inertia, CoM, drag, sensors and flight control are assumptions or
  calibrations (DJI_MATRICE_400_TECHNICAL_SPECIFICATION.md).
* The power coefficient is calibrated to a manufacturer endurance claim; that claim therefore cannot
  validate the energy model.

## 2. Aircraft model assumptions (all in the spec with ranges and sensitivities)

Four-rotor symmetric X (secondary evidence + geometry); spin assignment sign unverified; C_T 0.10;
C_P 0.03774 (calibrated); TWR 1.6 at MTOW; motor lag 60 ms; inertia from lumped boxes (±30 %); CoM from
assumed layout; airframe drag from a top-speed argument; avionics 60 W; battery R0 60 mΩ, generic Li-ion
OCV, 3.0 V cutoff, 95 % usable energy; landing gear geometry and contact parameters; sensor noise of
class-representative MEMS/RTK units.

## 3. Physics not modelled

Rotor inflow / advance-ratio dependence of thrust and torque, blade flapping and rotor drag (H-force),
translational lift, vortex-ring state, ground and wall effect, rotor–rotor interaction, building-induced
flow (only the mean profile, turbulence and gusts are modelled), motor electrical dynamics and ESC
behaviour, motor/battery temperature, propeller gyroscopic and acceleration torques, structural flexibility,
payload dynamics, communication-link effects on control.

## 4. Estimation, diagnosis and control limitations

* Mass change and uniform thrust loss are indistinguishable (θ = η m_nom/m); CoM shift and asymmetric rotor
  loss are grouped; lag and transport delay are combined; drag and wind are separable only with speed
  variation and remain weakly identifiable for the heavy M400 (drag acceleration ~0.13 m/s² at 3–4 m/s vs.
  ~0.02 m/s² residual accelerometer bias).
* Thresholds (tolerances, CUSUM h, persistence, probability margins) were set on development seed 0 for the
  nominal and single-fault scenarios; they are not optimised and their transfer to the real aircraft is
  unknown.
* The information-gain values are model-based expectations (the lag term ignores noise realisations and
  slightly over-estimates; the hover term is a heuristic).
* The safety layer has no formal guarantee: limits are enforced by construction and projection, the
  0.5 m clearance margin is not sufficient for sudden actuator losses ≥ 15 % at 3 m stand-off next to
  balconies (observed clearance 0.37–0.47 m), and a 30 % loss crashed the stock PID in the severity sweep.
* The baseline PID's rate loop is limited by the assumed 60 ms motor lag; the DOB compensates translational
  disturbances only.
* Computation times are for a Python prototype on a shared desktop CPU and include parallel-run contention;
  missed 4 ms deadlines in these logs do not predict embedded performance.

## 5. DJI integration limitations

On the real aircraft the inner loops, allocation, estimator and failsafes are DJI's; only velocity/position
setpoints at ≤ 25 Hz are expected to be available. The framework's actuator-level residuals (allocated
wrench, specific force vs commanded thrust) would have to be replaced by closed-loop models
(DJI_MATRICE_400_INTEGRATION_PLAN.md §7). SDK telemetry rates, latency and raw-sensor access on M400 are
unknown.

## 6. Unresolved items (need human action or hardware)

1. First-hand quotation of rotor numbering and spin directions from the M400 maintenance/user manual.
2. M400 user manual: failsafe defaults, flight modes, RTH logic.
3. Whether MSDK V5 apps run on RC Plus 2 with M400; M400-specific Virtual Stick / PSDK joystick limits.
4. Telemetry field list, rates, latency and timestamping on M400.
5. Measurements: thrust stand (k_f, k_m, τ, current), inertia (pendulum), CoM, battery discharge (OCV, R0),
   hover power and step responses from logged flights.
6. Literature review before claiming any contribution or choosing further algorithms (NMPC, CBF, etc.).
7. Approval for new dependencies if NMPC is pursued (CasADi/acados).
8. Path B (own drone): hardware not selected; nothing modelled.
