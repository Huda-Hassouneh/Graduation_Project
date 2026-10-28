"""Rotation conventions: body FLU -> world ENU, Hamilton scalar-first quaternions."""

import numpy as np
import pytest

from graduation_project.dynamics import frames as fr

RNG = np.random.default_rng(0)


def random_quat() -> np.ndarray:
    return fr.quat_normalize(RNG.normal(size=4))


def test_yaw_90_maps_body_forward_to_world_north() -> None:
    q = fr.quat_from_axis_angle([0, 0, 1], np.pi / 2)
    np.testing.assert_allclose(fr.quat_to_rotmat(q) @ [1, 0, 0], [0, 1, 0], atol=1e-15)


def test_positive_pitch_is_nose_down_in_flu() -> None:
    q = fr.quat_from_euler_zyx(0.0, np.deg2rad(10), 0.0)
    nose_w = fr.quat_to_rotmat(q) @ [1, 0, 0]
    assert nose_w[2] < 0


def test_rotation_matrix_is_orthonormal_and_proper() -> None:
    for _ in range(50):
        R = fr.quat_to_rotmat(random_quat())
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-12)


def test_quaternion_matrix_round_trip_and_product_composition() -> None:
    for _ in range(50):
        q, p = random_quat(), random_quat()
        q_rt = fr.rotmat_to_quat(fr.quat_to_rotmat(q))
        assert fr.rotation_angle_between(q, q_rt) < 1e-7
        np.testing.assert_allclose(
            fr.quat_to_rotmat(fr.quat_multiply(p, q)), fr.quat_to_rotmat(p) @ fr.quat_to_rotmat(q), atol=1e-12
        )


def test_euler_round_trip_away_from_singularity() -> None:
    angles = np.array([0.3, -0.4, 1.2])  # roll, pitch, yaw
    q = fr.quat_from_euler_zyx(angles[2], angles[1], angles[0])
    np.testing.assert_allclose(fr.euler_zyx_from_quat(q), angles, atol=1e-12)


def test_skew_matches_cross_product() -> None:
    a, b = RNG.normal(size=3), RNG.normal(size=3)
    np.testing.assert_allclose(fr.skew(a) @ b, np.cross(a, b), atol=1e-15)


def test_exponential_increment_matches_axis_angle() -> None:
    omega = np.array([0.0, 0.0, 2.0])
    q = fr.quat_exp_increment(omega, 0.5)
    np.testing.assert_allclose(q, fr.quat_from_axis_angle([0, 0, 1], 1.0), atol=1e-15)


def test_normalize_rejects_zero() -> None:
    with pytest.raises(ValueError):
        fr.quat_normalize([0, 0, 0, 0])
