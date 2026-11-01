#!/usr/bin/env python3
"""Validate the Gazebo models and facade worlds from a clean shell (no environment setup needed).

Usage (from the repository root):
    python3 simulation/gazebo/check_worlds.py                       # both worlds
    python3 simulation/gazebo/check_worlds.py --world facade_complex
    python3 simulation/gazebo/check_worlds.py --replay results/runs/<experiment_id> [--speed 20]

Steps: (1) `gz sdf -k` on every model.sdf and world .sdf with model:// resolution configured;
(2) per world, a headless `gz sim -s -r` server on a private gz-transport partition (cannot touch
a running user session), checked for every top-level model and the included aircraft pose;
(3) with --replay, a KINEMATIC replay of the given run into each world, checking that the
aircraft ends at the last logged pose. This checks loading and replay only, not flight dynamics.
Exit status 0 only if every check passes.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.gazebo import MODELS_DIR, WORLDS_DIR, gazebo_env, repo_relative  # noqa: E402

WORLDS = ["facade_flat", "facade_complex", "facade_complex_m400", "m400_flight_test"]
DYNAMIC_POSE_TOL = 0.02  # [m, rad] models with physics settle on their landing gear


def _vec(a) -> str:
    return "none" if a is None else "[" + " ".join(f"{x:.4f}" for x in np.asarray(a) + 0.0) + "]"


def check_sdf(path: Path, env: dict[str, str]) -> bool:
    r = subprocess.run(["gz", "sdf", "-k", str(path)], capture_output=True, text=True, env=env, timeout=60)
    ok = r.returncode == 0 and "Valid." in r.stdout
    print(f"  [{'PASS' if ok else 'FAIL'}] gz sdf -k {repo_relative(path)}")
    if not ok:
        print("    " + (r.stdout + r.stderr).strip().replace("\n", "\n    "))
    return ok


def expected_entities(world_sdf: Path) -> tuple[set[str], dict[str, np.ndarray]]:
    """Top-level model names and included-model poses (x y z roll pitch yaw) declared in the world."""
    world = ET.parse(world_sdf).getroot().find("world")
    names = {m.get("name") for m in world.findall("model")}
    poses = {}
    for inc in world.findall("include"):
        name = inc.findtext("name") or inc.findtext("uri").removeprefix("model://")
        names.add(name)
        poses[name] = np.array([float(x) for x in (inc.findtext("pose") or "0 0 0 0 0 0").split()])
    return names, poses


def model_pose(name: str, env: dict[str, str]) -> np.ndarray | None:
    """Pose from `gz model -m <name> -p` as [x, y, z, roll, pitch, yaw], or None if unavailable."""
    out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True, text=True, env=env,
                         timeout=30).stdout
    rows = [ln.strip().strip("[]").split() for ln in out.splitlines() if ln.strip().startswith("[")]
    return np.array(rows[0] + rows[1], dtype=float) if len(rows) >= 2 else None


def list_models(env: dict[str, str]) -> set[str]:
    try:
        out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, env=env, timeout=5).stdout
    except subprocess.TimeoutExpired:
        return set()
    return {ln.strip()[2:] for ln in out.splitlines() if ln.strip().startswith("- ")}


def check_world(world: str, env: dict[str, str], timeout_s: float, hold_s: float, replay_run: Path | None,
                speed: float) -> bool:
    sdf = WORLDS_DIR / f"{world}.sdf"
    names, poses = expected_entities(sdf)
    env = {**env, "GZ_PARTITION": f"gp_check_{uuid.uuid4().hex[:8]}"}
    t0 = time.monotonic()
    server = subprocess.Popen(["gz", "sim", "-s", "-r", str(sdf)], env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True)
    ok = True
    try:
        found: set[str] = set()
        while time.monotonic() - t0 < timeout_s and server.poll() is None:
            found = list_models(env)
            if names <= found:
                break
            time.sleep(0.5)
        loaded = names <= found
        print(f"  [{'PASS' if loaded else 'FAIL'}] headless load {world}: {len(found & names)}/{len(names)} models "
              f"in {time.monotonic() - t0:.1f} s ({', '.join(sorted(found)) or 'none'})")
        if not loaded:
            missing = names - found
            print(f"    missing: {sorted(missing)}; server {'exited' if server.poll() is not None else 'running'}")
            return False
        for name, want in poses.items():
            got = model_pose(name, env)
            tol = DYNAMIC_POSE_TOL if name.endswith("_flight") else 1e-4
            match = got is not None and np.allclose(got, want, atol=tol)
            ok &= match
            print(f"  [{'PASS' if match else 'FAIL'}] {name} pose {_vec(got)} (world file: {_vec(want)})")
        time.sleep(hold_s)
        alive = server.poll() is None
        ok &= alive
        print(f"  [{'PASS' if alive else 'FAIL'}] server still running after {hold_s:g} s")
        if replay_run is not None:
            ok &= check_replay(world, replay_run, env, speed)
    finally:
        server.terminate()
        try:
            out, _ = server.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            server.kill()
            out, _ = server.communicate()
        errors = [ln for ln in out.splitlines() if "[Err]" in ln or "Error" in ln]
        if errors:
            ok = False
            print("  [FAIL] server reported errors:\n    " + "\n    ".join(errors[:10]))
    return ok


def check_replay(world: str, run: Path, env: dict[str, str], speed: float) -> bool:
    from graduation_project.gazebo.replay import GazeboReplayer, load_replay, replay

    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]  # the in-process gz-transport node reads this
    data = load_replay(run)
    sent = replay(data, GazeboReplayer(world), fps=30.0, speed=speed)
    time.sleep(0.5)
    got = model_pose(data.aircraft, env)
    w, x, y, z = data.quat_wxyz[-1]
    rpy = [np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
           np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))]
    want = np.concatenate([data.position[-1], rpy])
    match = got is not None and np.allclose(got, want, atol=1e-3)
    print(f"  [{'PASS' if match else 'FAIL'}] kinematic replay {data.scenario}/{data.aircraft}: {sent} frames; "
          f"final pose {_vec(got)} vs last logged {_vec(want)}")
    return match


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", choices=WORLDS, action="append")
    ap.add_argument("--timeout", type=float, default=60.0, help="seconds to wait for each world to load")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds the server must keep running after loading")
    ap.add_argument("--replay", type=Path, help="also replay this results/runs/<experiment_id> into each world")
    ap.add_argument("--speed", type=float, default=20.0, help="replay speed factor")
    args = ap.parse_args()

    env = gazebo_env()
    print(f"model path: {MODELS_DIR}")
    print("SDF validation:")
    ok = all([check_sdf(p, env) for p in sorted(MODELS_DIR.glob("*/model.sdf")) + sorted(WORLDS_DIR.glob("*.sdf"))])
    for world in args.world or WORLDS:
        print(f"world {world}:")
        ok &= check_world(world, env, args.timeout, args.hold, args.replay, args.speed)
    print("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
