# Coordinate frames and conventions

All quantities are SI (m, s, kg, rad, N, N·m, W, J) unless a name ends in `_deg`, `_rpm` or `_wh`.

## Frames

| Frame | Symbol | Axes | Origin | Notes |
|---|---|---|---|---|
| World | W | ENU: x east, y north, z up | Fixed take-off reference point at ground level | Matches ROS REP 103 `map`/`odom` convention and Gazebo's default world frame |
| Body | B | FLU: x forward, y left, z up | Vehicle centre of mass (CoM) | Matches ROS REP 103 `base_link` |
| Geometric | G | parallel to B | Geometric centre of the airframe | Used only in YAML files to place mass components and rotor hubs. Loading shifts every position by `-r_CoM`, so the simulator always uses B. |

Gravity is `g_W = [0, 0, -g]` with `g = 9.80665 m/s²`.

## Attitude

* Unit quaternion `q = [w, x, y, z]` (Hamilton product, scalar first) rotating **body to world**:
  `v_W = R(q) v_B`. `R(q)` is `frames.quat_to_rotmat`.
* Kinematics: `q̇ = ½ q ⊗ [0, ω_B]` with body-frame angular velocity `ω_B`.
* Integration does not use Euler angles. The quaternion is renormalised after every RK4 step.
  If the norm drifts by more than 1e-3 within a single step, the state is rejected
  (`StateValidationError`).
* Euler angles (roll φ, pitch θ, yaw ψ; intrinsic Z-Y-X, `R = Rz(ψ) Ry(θ) Rx(φ)`) appear
  **only** in logs and plots. They are singular at |θ| = 90°.
* In FLU, **positive pitch tilts the nose down** (rotation about +y_B, which points left).
  Positive roll raises the left side. Positive yaw turns the nose counter-clockwise seen from
  above (from east toward north). Unit tests confirm all three (`tests/test_frames.py`,
  `tests/test_dynamics.py::test_torque_sign_conventions`).

## Rotors

* X quadrotor. Rotors are numbered counter-clockwise seen from above, starting front-right:
  1 front-right (azimuth −45°), 2 front-left (+45°), 3 rear-left (+135°), 4 rear-right (−135°).
* Spin direction `s_i = +1` means counter-clockwise seen from above (rotor angular velocity
  along +z_B). Diagonal rotors (1, 3) and (2, 4) share a direction. Both configurations use
  `s = [+1, −1, +1, −1]`. **For the Matrice 400 reference this absolute assignment is
  unverified.** The opposite assignment only flips the sign of the yaw row of the allocation
  matrix.
* Thrust acts along +z_B. The reaction torque of rotor i on the airframe is `−s_i Q_i z_B`.

## Wind

Wind `w_W` is the velocity of the air mass in W. "Wind from the west" (blowing toward the east)
is `w_W = [+u, 0, 0]`. Drag depends on the air-relative velocity `v_rel = v_W − w_W`.

## Gazebo / ROS interfaces (planned, Milestone 3)

Gazebo Harmonic uses a right-handed, z-up world frame, the same as ENU, so no world conversion
is needed. Links in the SDF model will use the FLU body convention with the link origin at the
CoM. If external data uses NED/FRD (for example PX4 conventions), convert with
`p_ENU = [p_N→y, p_E→x, −p_D]`, i.e. `p_ENU = [p_E, p_N, −p_D]`, and
`v_B,FLU = [v_F, −v_R, −v_D]`. Any such conversion will live in one module with its own tests.
