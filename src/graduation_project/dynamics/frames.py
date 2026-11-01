"""Rotation and quaternion utilities.

Conventions (see docs/coordinate_frames.md):

* World frame W: ENU (x east, y north, z up). Gravity is ``-g * e_z``.
* Body frame B: FLU (x forward, y left, z up), origin at the centre of mass.
* Quaternions are Hamilton, scalar-first ``[w, x, y, z]`` and represent the body-to-world
  rotation: ``v_W = R(q) @ v_B``.
* Euler angles are used only for logging/plots: intrinsic Z-Y-X (yaw, pitch, roll).
  In FLU a *positive* pitch rotates the nose *down*.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]

QUAT_IDENTITY: FloatArray = np.array([1.0, 0.0, 0.0, 0.0])


def skew(v: ArrayLike) -> FloatArray:
    """Return the cross-product matrix ``[v]x`` such that ``[v]x @ u == v x u``."""
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def cross3(a: FloatArray, b: FloatArray) -> FloatArray:
    """Cross product of two 3-vectors (much faster than ``np.cross`` for single vectors)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def norm(v: FloatArray) -> float:
    """Euclidean norm of a 1-D float array (same arithmetic as ``np.linalg.norm`` for 1-D input, less overhead)."""
    return math.sqrt(float(np.dot(v, v)))


def quat_normalize(q: ArrayLike) -> FloatArray:
    q = np.asarray(q, dtype=float)
    n = norm(q)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError(f"cannot normalise quaternion {q}")
    return q / n


def quat_multiply(p: ArrayLike, q: ArrayLike) -> FloatArray:
    """Hamilton product ``p (x) q``."""
    pw, px, py, pz = np.asarray(p, dtype=float).tolist()
    qw, qx, qy, qz = np.asarray(q, dtype=float).tolist()
    return np.array(
        [
            pw * qw - px * qx - py * qy - pz * qz,
            pw * qx + px * qw + py * qz - pz * qy,
            pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw,
        ]
    )


def quat_conjugate(q: ArrayLike) -> FloatArray:
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([w, -x, -y, -z])


def quat_to_rotmat(q: ArrayLike) -> FloatArray:
    """Rotation matrix ``R`` (body-to-world) of a unit quaternion."""
    w, x, y, z = np.asarray(q, dtype=float).tolist()
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotmat_to_quat(R: ArrayLike) -> FloatArray:
    """Unit quaternion (``w >= 0``) of a rotation matrix, using Shepperd's method."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(1.0 + tr)
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q = quat_normalize(q)
    return q if q[0] >= 0.0 else -q


def quat_from_axis_angle(axis: ArrayLike, angle: float) -> FloatArray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle
    return np.concatenate(([np.cos(half)], np.sin(half) * axis))


def quat_exp_increment(omega_body: ArrayLike, dt: float) -> FloatArray:
    """Quaternion of a rotation by ``omega_body * dt`` (exponential map)."""
    rotvec = np.asarray(omega_body, dtype=float) * dt
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return quat_normalize(np.concatenate(([1.0], 0.5 * rotvec)))
    return quat_from_axis_angle(rotvec / angle, angle)


def quat_derivative(q: ArrayLike, omega_body: ArrayLike) -> FloatArray:
    """Kinematics ``q_dot = 0.5 * q (x) [0, omega_B]`` for body-frame angular velocity."""
    return 0.5 * quat_multiply(q, np.concatenate(([0.0], np.asarray(omega_body, dtype=float))))


def quat_from_euler_zyx(yaw: float, pitch: float, roll: float) -> FloatArray:
    """Quaternion from intrinsic Z-Y-X Euler angles [rad]."""
    qz = quat_from_axis_angle([0.0, 0.0, 1.0], yaw)
    qy = quat_from_axis_angle([0.0, 1.0, 0.0], pitch)
    qx = quat_from_axis_angle([1.0, 0.0, 0.0], roll)
    return quat_multiply(quat_multiply(qz, qy), qx)


def euler_zyx_from_quat(q: ArrayLike) -> FloatArray:
    """Return ``[roll, pitch, yaw]`` [rad] (intrinsic Z-Y-X). For logging only: singular at |pitch| = 90 deg."""
    R = quat_to_rotmat(q)
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def rotation_angle_between(q_a: ArrayLike, q_b: ArrayLike) -> float:
    """Geodesic angle [rad] of the relative rotation between two unit quaternions."""
    dot = abs(float(np.dot(quat_normalize(q_a), quat_normalize(q_b))))
    return 2.0 * float(np.arccos(np.clip(dot, -1.0, 1.0)))
