# Limitations (current state)

## Model fidelity
* The M400 reference (configuration B) is not a digital twin. Propulsion coefficients,
  inertia, motor lag and drag are assumptions or crude derivations (aircraft_reference.md §4).
* Rotor count (4) and layout (X) are inferred from secondary evidence. Spin-direction
  assignment is unverified (it affects only the yaw-row sign).
* Not modelled: rotor gyroscopic and acceleration torques, blade flapping and rotor drag,
  thrust changes with inflow or airspeed, ground and wall effect, landing-gear contact and
  ground contact (runs start airborne), battery voltage sag, ESC dynamics.
* Drag is a per-axis lumped linear + quadratic model.

## Control and evaluation
* All Milestone 2 results use **ideal state feedback** (true state). They are not
  representative of sensor-based performance.
* The baseline has no jerk or body-rate feed-forward, which causes phase-lag error on
  curved paths.
* No stochastic disturbances yet. All baseline results come from a single deterministic run
  per scenario (no statistics).

## Software and environment
* Python fixed-step simulation runs at about 0.5–1× real time on the development machine
  while Gazebo is running concurrently. Large batches will need either acceptance of long
  runtimes or vectorisation.
* pandas and pip are unavailable. Data is plain CSV.
* No GitHub remote exists (`gh` is not installed).
