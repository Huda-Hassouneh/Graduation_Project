#!/usr/bin/env python3
"""Generate the facade-inspection Gazebo worlds (deterministic).

Usage (from the repository root):
    python3 simulation/worlds/generate_facade_world.py                 # facade_flat/complex + M400 worlds
    python3 simulation/worlds/generate_facade_world.py --name my_wall --variant complex \\
        --facade-height 15 --keepout-margin 1.5 --aircraft m400_reference_quad --out-dir /tmp/worlds

Writes <out-dir>/<name>.sdf and <out-dir>/<name>.geometry.json. The scenario facade
(origin, normal, tangent, width) defaults to graduation_project.experiments.scenarios; change it
only together with the scenarios. Worlds include the aircraft via model://<aircraft>; launch with
    simulation/gazebo/launch_world.sh facade_flat
or run `source simulation/gazebo/env.sh` before calling `gz sim` / `gz sdf` directly.
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.gazebo import WORLDS_DIR  # noqa: E402
from graduation_project.gazebo.facade_world import DEFAULT_WORLDS, M400_WORLDS, FacadeWorldSpec, write_world  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", help="world name; omit to generate the default worlds")
    ap.add_argument("--variant", choices=["flat", "complex"], default="flat")
    ap.add_argument("--facade-height", type=float)
    ap.add_argument("--building-depth", type=float)
    ap.add_argument("--keepout-margin", type=float)
    ap.add_argument("--aircraft", default="research_quad", help="model to place on the pad, or 'none'")
    ap.add_argument("--dynamic-aircraft", action="store_true",
                    help="include the aircraft as a non-static model (a *_flight model is flown by Gazebo's own controller; others only rest on the pad)")
    ap.add_argument("--out-dir", type=Path, default=WORLDS_DIR)
    args = ap.parse_args()

    overrides = {k: v for k, v in {"facade_height": args.facade_height, "building_depth": args.building_depth,
                                   "keepout_margin": args.keepout_margin}.items() if v is not None}
    if args.name:
        overrides["aircraft"] = None if args.aircraft == "none" else args.aircraft
        overrides["aircraft_static"] = not args.dynamic_aircraft
        specs = [FacadeWorldSpec(name=args.name, variant=args.variant, **overrides)]
    else:  # default set: research worlds + company-platform (M400) worlds, each with its own aircraft
        specs = [dataclasses.replace(s, **overrides) for s in DEFAULT_WORLDS + M400_WORLDS]
    for spec in specs:
        for p in write_world(spec, args.out_dir):
            print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
