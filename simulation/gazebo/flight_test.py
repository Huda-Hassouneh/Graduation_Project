#!/usr/bin/env python3
"""Headless Gazebo-PHYSICS flight test of the dji_matrice_400_flight model (secondary cross-check).

Starts ``gz sim -s -r simulation/worlds/m400_flight_test.sdf`` on a private gz-transport partition,
subscribes to the world pose stream, and commands the model through Gazebo's MulticopterVelocityControl:

    spawn -> idle 2 s (stability on the pad) -> enable -> climb 1 m/s 4 s -> hover 5 s ->
    +x 1 m/s 3 s -> +y 1 m/s 3 s -> -z 0.5 m/s 2 s -> yaw 0.5 rad/s 3 s -> hover 3 s ->
    descend 0.6 m/s until on the pad -> disable

Pass criteria are printed for each behaviour (spawn, stable, take-off, hover, x/y/z motion, rotation,
landing). This exercises the generated SDF (links, joints, inertia, rotor plugins) in Gazebo's own physics
with Gazebo's own controller. It is NOT a test of the research controllers or framework, which run in the
Python simulator (docs/SIMULATION_MODEL.md).

Usage (from the repository root, no environment setup needed):
    python3 simulation/gazebo/flight_test.py [--csv results/gazebo_flight_test.csv]
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.gazebo import WORLDS_DIR, gazebo_env  # noqa: E402
from graduation_project.gazebo.sdf_model import FLIGHT_NAMESPACE  # noqa: E402

WORLD = "m400_flight_test"
MODEL = "dji_matrice_400_flight"


def yaw_of(q) -> float:
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", type=Path, help="write the pose log here")
    ap.add_argument("--timeout", type=float, default=90.0)
    args = ap.parse_args()

    env = gazebo_env()
    env["GZ_PARTITION"] = f"gp_flight_{uuid.uuid4().hex[:8]}"
    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]
    server = subprocess.Popen(["gz", "sim", "-s", "-r", str(WORLDS_DIR / f"{WORLD}.sdf")], env=env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    import gz.transport13  # noqa: F401  (must precede gz.msgs10 imports; see gazebo/replay.py)
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node

    node = Node()
    log: list[tuple[float, float, float, float, float]] = []
    lock = threading.Lock()
    t_start = time.monotonic()

    def on_pose(msg: Pose_V) -> None:
        for p in msg.pose:
            if p.name == MODEL:
                with lock:
                    log.append((time.monotonic() - t_start, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))

    node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", on_pose)
    twist_pub = node.advertise(f"/{FLIGHT_NAMESPACE}/gazebo/command/twist", Twist)
    enable_pub = node.advertise(f"/{FLIGHT_NAMESPACE}/enable", Boolean)

    def last():
        with lock:
            return log[-1] if log else None

    def send(vx=0.0, vy=0.0, vz=0.0, wz=0.0, duration=1.0, until=None) -> tuple[float, float]:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = vx, vy, vz, wz
        t0 = time.monotonic() - t_start
        end = time.monotonic() + duration
        while time.monotonic() < end:
            twist_pub.publish(msg)
            if until is not None and last() is not None and until(last()):
                break
            time.sleep(0.05)
        return t0, time.monotonic() - t_start

    results: list[tuple[str, bool, str]] = []
    try:
        deadline = time.monotonic() + 40.0
        while last() is None and time.monotonic() < deadline:
            time.sleep(0.2)
        if last() is None:
            print("[FAIL] spawn: model pose never received")
            return 1
        results.append(("spawn", True, f"pose received after {last()[0]:.1f} s"))
        time.sleep(2.0)
        seg = {}
        seg["idle"] = (last()[0] - 2.0, last()[0])
        b = Boolean()
        b.data = True
        for _ in range(5):
            enable_pub.publish(b)
            time.sleep(0.1)
        seg["climb"] = send(vz=1.0, duration=4.0)
        seg["hover"] = send(duration=5.0)
        seg["x"] = send(vx=1.0, duration=3.0)
        seg["settle_x"] = send(duration=2.0)
        seg["y"] = send(vy=1.0, duration=3.0)
        seg["settle_y"] = send(duration=2.0)
        seg["down"] = send(vz=-0.5, duration=2.0)
        seg["yaw"] = send(wz=0.5, duration=3.0)
        seg["hover2"] = send(duration=3.0)
        z_rest = log[0][3]
        seg["land"] = send(vz=-0.6, duration=25.0, until=lambda s: s[3] < z_rest + 0.03)
        seg["touch"] = send(duration=3.0)
        b.data = False
        enable_pub.publish(b)
        time.sleep(0.5)
    finally:
        server.terminate()
        try:
            server.wait(timeout=15)
        except subprocess.TimeoutExpired:
            server.kill()

    L = np.array(log)
    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(args.csv, L, delimiter=",", header="t,x,y,z,yaw", comments="", fmt="%.6f")

    def window(name):
        a, b = seg[name]
        return L[(L[:, 0] >= a) & (L[:, 0] <= b)]

    def at(t):
        return L[np.argmin(np.abs(L[:, 0] - t))]

    z0 = L[0, 3]
    idle = window("idle")
    results.append(("stable on pad", float(np.ptp(idle[:, 3])) < 0.01 and float(np.ptp(idle[:, 1])) < 0.01,
                    f"idle z range {np.ptp(idle[:, 3]) * 1000:.1f} mm"))
    climb_end = at(seg["climb"][1])
    # Command durations are wall-clock; under CPU load Gazebo may run below real time, so displacement
    # thresholds are set well below the nominal command integral (they test behaviour, not tracking).
    results.append(("take-off", climb_end[3] - z0 > 1.5, f"climbed {climb_end[3] - z0:.2f} m in 4 s at 1 m/s command"))
    w = window("hover")
    hv = w[w[:, 0] >= w[0, 0] + 2.0]  # after 2 s of settling from the climb
    drift = float(np.max(np.linalg.norm(hv[:, 1:4] - hv[0, 1:4], axis=1))) if len(hv) else float("nan")
    results.append(("hover", drift < 0.3, f"max drift {drift:.3f} m over the last 3 s of the hover window"))
    for axis, name, col in (("x", "x", 1), ("y", "y", 2)):
        d = at(seg[f"settle_{name}"][1])[col] - at(seg[name][0])[col]
        results.append((f"move {axis}", d > 1.5, f"displacement {d:.2f} m for 3 s at 1 m/s"))
    dz = at(seg["down"][1])[3] - at(seg["down"][0])[3]
    results.append(("move z (down)", dz < -0.3, f"displacement {dz:.2f} m for 2 s at -0.5 m/s"))
    dyaw = np.unwrap(window("yaw")[:, 4])
    dy = float(dyaw[-1] - dyaw[0]) if len(dyaw) else float("nan")
    results.append(("rotate", dy > 0.5, f"yaw change {np.degrees(dy):.0f} deg for 3 s at 0.5 rad/s"))
    end = L[-1]
    results.append(("land", abs(end[3] - z0) < 0.05, f"final height above rest {1000 * (end[3] - z0):.0f} mm"))
    ok = all(r[1] for r in results)
    for name, passed, detail in results:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
    print("GAZEBO FLIGHT TEST PASSED" if ok else "GAZEBO FLIGHT TEST FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
