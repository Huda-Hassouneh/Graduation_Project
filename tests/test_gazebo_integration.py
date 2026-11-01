"""Gazebo tool checks: `gz sdf -k` syntax validation and a headless `gz sim` load + replay test.

Skipped when the `gz` CLI is not installed. The headless test runs on a private gz-transport
partition so it cannot interfere with a user's Gazebo session. These tests check loading and
kinematic replay only; they say nothing about flight dynamics.
"""

import os
import shutil
import subprocess
import sys
import time
import uuid

import numpy as np
import pytest

from graduation_project.gazebo import GZ_PATH_VARS, MODELS_DIR, SIMULATION_DIR, WORLDS_DIR, gazebo_env

pytestmark = pytest.mark.skipif(shutil.which("gz") is None, reason="gz CLI not installed")

SDF_FILES = sorted(MODELS_DIR.glob("*/model.sdf")) + sorted(WORLDS_DIR.glob("*.sdf"))


@pytest.mark.parametrize("path", SDF_FILES, ids=lambda p: f"{p.parent.name}/{p.name}")
def test_gz_sdf_check(path) -> None:
    r = subprocess.run(["gz", "sdf", "-k", str(path)], capture_output=True, text=True, env=gazebo_env(), timeout=60)
    assert r.returncode == 0 and "Valid." in r.stdout, r.stdout + r.stderr


def _clean_env() -> dict[str, str]:
    return {k: v for k, v in os.environ.items() if k not in GZ_PATH_VARS}


def test_world_include_needs_model_path() -> None:
    """Guards the reason env.sh / gazebo_env exist: a clean shell cannot resolve model://research_quad."""
    r = subprocess.run(["gz", "sdf", "-k", str(WORLDS_DIR / "facade_flat.sdf")], capture_output=True, text=True,
                       env=_clean_env(), timeout=60)
    assert "Valid." not in r.stdout and "model://research_quad" in r.stdout + r.stderr


def test_env_sh_configures_a_clean_shell(tmp_path) -> None:
    env = {**_clean_env(), "GZ_SIM_RESOURCE_PATH": "/opt/other"}
    script = (f'source "{SIMULATION_DIR}/gazebo/env.sh"; source "{SIMULATION_DIR}/gazebo/env.sh"; '
              'echo "$GZ_SIM_RESOURCE_PATH"; echo "$SDF_PATH"; gz sdf -k "$1"')
    r = subprocess.run(["bash", "--norc", "-c", script, "_", str(WORLDS_DIR / "facade_complex.sdf")], cwd=tmp_path,
                       capture_output=True, text=True, env=env, timeout=60)
    lines = r.stdout.splitlines()
    assert lines[0] == f"{MODELS_DIR}:/opt/other"  # prepended once, existing entry kept
    assert lines[1] == str(MODELS_DIR)
    assert r.returncode == 0 and "Valid." in r.stdout, r.stdout + r.stderr


@pytest.fixture(scope="module")
def gz_env():
    partition = f"gp_test_{uuid.uuid4().hex[:8]}"
    old = {k: os.environ.get(k) for k in ("GZ_PARTITION", *GZ_PATH_VARS)}
    os.environ.update(gazebo_env())
    os.environ["GZ_PARTITION"] = partition  # also used by in-process gz.transport nodes
    yield dict(os.environ)
    for k, v in old.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v


@pytest.mark.slow
@pytest.mark.parametrize("world", ["facade_flat", "facade_complex"])
def test_headless_load_and_kinematic_replay(world: str, gz_env, tmp_path) -> None:
    pytest.importorskip("gz.transport13")
    from graduation_project.gazebo.replay import GazeboReplayer, ReplayData, replay

    server = subprocess.Popen(["gz", "sim", "-s", "-r", str(WORLDS_DIR / f"{world}.sdf")], env=gz_env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        rep = GazeboReplayer(world, timeout_ms=2000)
        deadline = time.monotonic() + 30.0
        names: list[str] = []
        while time.monotonic() < deadline:
            try:
                names = rep.model_names()
                break
            except RuntimeError:
                time.sleep(0.5)
        assert {"ground_plane", "takeoff_pad", "facade_building", "keepout_zones", "research_quad"} <= set(names)

        # Synthetic 1 s log for the aircraft not in the world: exercises spawn + set_pose_vector.
        t = np.linspace(0.0, 1.0, 251)
        pos = np.column_stack([2.0 * t, 5.0 + 0 * t, 3.0 + t])
        q = np.tile([np.cos(0.25), 0.0, 0.0, np.sin(0.25)], (len(t), 1))  # yaw 0.5 rad
        data = ReplayData(tmp_path, "m400_reference_quad", "synthetic", "ideal_truth", t, pos, q, pos + 0.1)
        assert replay(data, rep, fps=20.0, speed=4.0) == 21
        time.sleep(0.5)
        out = subprocess.run(["gz", "model", "-m", "m400_reference_quad", "-p"], env=gz_env,
                             capture_output=True, text=True, timeout=30).stdout
        rows = [ln.strip().strip("[]").split() for ln in out.splitlines() if ln.strip().startswith("[")]
        xyz, rpy = np.array(rows[0], float), np.array(rows[1], float)
        np.testing.assert_allclose(xyz, [2.0, 5.0, 4.0], atol=1e-4)
        np.testing.assert_allclose(rpy, [0.0, 0.0, 0.5], atol=1e-4)

        # The aircraft included by the world (static, via model://) must also follow a replay.
        data = ReplayData(tmp_path, "research_quad", "synthetic", "ideal_truth", t, pos - 1.0, q, None)
        assert replay(data, rep, fps=20.0, speed=4.0) == 21
        time.sleep(0.5)
        out = subprocess.run(["gz", "model", "-m", "research_quad", "-p"], env=gz_env,
                             capture_output=True, text=True, timeout=30).stdout
        rows = [ln.strip().strip("[]").split() for ln in out.splitlines() if ln.strip().startswith("[")]
        np.testing.assert_allclose(np.array(rows[0], float), [1.0, 4.0, 3.0], atol=1e-4)
    finally:
        server.terminate()
        try:
            server.wait(timeout=15)
        except subprocess.TimeoutExpired:
            server.kill()


@pytest.mark.slow
def test_m400_gazebo_physics_flight() -> None:
    """Gazebo-physics flight model: spawn, stable, take-off, hover, x/y/z motion, rotation, landing
    (simulation/gazebo/flight_test.py; Gazebo's own controller - a cross-check of the generated SDF)."""
    pytest.importorskip("gz.transport13")
    r = subprocess.run([sys.executable, str(SIMULATION_DIR / "gazebo" / "flight_test.py")], capture_output=True, text=True,
                       timeout=240)
    assert r.returncode == 0 and "GAZEBO FLIGHT TEST PASSED" in r.stdout, r.stdout + r.stderr
