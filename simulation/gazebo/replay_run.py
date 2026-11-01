#!/usr/bin/env python3
"""KINEMATIC REPLAY of a Python-simulator run in a running Gazebo world (visualisation only).

Gazebo does not simulate the flight: the logged truth pose is applied to a static model.
A replay is not evidence that the flight dynamics are correct.

Usage (from the repository root):
    simulation/gazebo/launch_world.sh facade_flat &          # GUI; add --headless for server only
    python3 simulation/gazebo/replay_run.py --world facade_flat \\
        --run results/runs/<experiment_id> [--fps 30] [--speed 2] [--no-reference]
    python3 simulation/gazebo/replay_run.py --run results/runs/<experiment_id> --dry-run

If the run's aircraft is not in the world it is spawned as a static model. A green sphere
shows the reference position (ref_p_*) unless --no-reference is given.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.gazebo.replay import GazeboReplayer, frame_indices, load_replay, replay  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True, help="results/runs/<experiment_id> directory")
    ap.add_argument("--world", default="facade_flat", help="name of the running Gazebo world")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--no-reference", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="load and resample only; do not contact Gazebo")
    args = ap.parse_args()

    data = load_replay(args.run)
    n = len(frame_indices(data.t, args.fps))
    print(f"KINEMATIC REPLAY (no Gazebo dynamics): {data.scenario} / {data.aircraft} "
          f"[{data.feedback_mode}], {data.t[-1]:.2f} s log, {n} frames at {args.fps:g} fps")
    if args.dry_run:
        return 0
    sent = replay(data, GazeboReplayer(args.world), args.fps, args.speed, not args.no_reference)
    print(f"sent {sent} frames to world '{args.world}'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
