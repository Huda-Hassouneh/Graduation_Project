"""Replay adapter: reading run logs and resampling (no Gazebo needed)."""

import json

import numpy as np
import pytest

from graduation_project.dynamics.frames import quat_from_euler_zyx
from graduation_project.experiments.runner import log_columns
from graduation_project.gazebo.replay import frame_indices, load_replay


def _write_run(run_dir, n=251, dt=0.004, q=None):
    cols = log_columns(4)
    data = np.zeros((n, len(cols)))
    t = np.arange(n) * dt
    data[:, cols.index("t")] = t
    data[:, cols.index("p_x")] = t
    data[:, cols.index("p_z")] = 5.0
    data[:, cols.index("ref_p_x")] = t + 0.1
    qq = quat_from_euler_zyx(0.3, 0.1, -0.2) if q is None else np.asarray(q)
    for k, c in enumerate("wxyz"):
        data[:, cols.index(f"q_{c}")] = qq[k]
    run_dir.mkdir()
    np.savetxt(run_dir / "timeseries.csv", data, delimiter=",", header=",".join(cols), comments="", fmt="%.9g")
    (run_dir / "metadata.json").write_text(json.dumps(
        {"scenario": "synthetic", "feedback_mode": "ideal_truth", "aircraft": {"name": "research_quad"}}))
    return t, qq


def test_load_replay_passes_frames_through_unchanged(tmp_path) -> None:
    t, q = _write_run(tmp_path / "run")
    d = load_replay(tmp_path / "run")
    assert d.aircraft == "research_quad" and d.scenario == "synthetic"
    np.testing.assert_allclose(d.position[:, 0], t, atol=1e-8)  # ENU in, ENU out
    np.testing.assert_allclose(d.position[:, 2], 5.0)
    np.testing.assert_allclose(d.quat_wxyz[0], q, atol=1e-8)  # scalar-first preserved
    np.testing.assert_allclose(d.ref_position[:, 0], t + 0.1, atol=1e-8)


def test_non_unit_quaternion_is_rejected(tmp_path) -> None:
    _write_run(tmp_path / "run", q=[1.0, 0.1, 0.0, 0.0])
    with pytest.raises(ValueError):
        load_replay(tmp_path / "run")


def test_frame_indices_cover_log_at_requested_rate() -> None:
    t = np.arange(0, 2501) * 0.004  # 10 s at 250 Hz
    idx = frame_indices(t, 30.0)
    assert idx[0] == 0 and idx[-1] == len(t) - 1 or t[-1] - t[idx[-1]] < 1 / 30
    assert np.all(np.diff(idx) > 0)
    assert len(idx) == pytest.approx(10 * 30 + 1, abs=1)
    assert np.all(np.diff(t[idx]) == pytest.approx(1 / 30, abs=0.004 + 1e-9))
    with pytest.raises(ValueError):
        frame_indices(t, 0.0)


def test_pose_msg_maps_quaternion_fields() -> None:
    pytest.importorskip("gz.transport13")
    from graduation_project.gazebo.replay import pose_msg

    m = pose_msg("quad", [1.0, 2.0, 3.0], [0.5, 0.5, -0.5, 0.5])
    assert (m.name, m.position.x, m.position.y, m.position.z) == ("quad", 1.0, 2.0, 3.0)
    assert (m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z) == (0.5, 0.5, -0.5, 0.5)
