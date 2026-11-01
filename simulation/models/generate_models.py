#!/usr/bin/env python3
"""Regenerate the Gazebo SDF models from the aircraft and visual YAML files (deterministic).

Usage (from the repository root):
    python3 simulation/models/generate_models.py                      # both aircraft
    python3 simulation/models/generate_models.py --aircraft research_quad
    python3 simulation/models/generate_models.py --out-dir /tmp/models # write elsewhere

Writes simulation/models/<aircraft>/{model.sdf,model.config}. Inputs:
config/aircraft/<aircraft>.yaml (mass, inertia, rotors) and config/visual/<aircraft>.yaml (shapes).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.gazebo import MODELS_DIR  # noqa: E402
from graduation_project.gazebo.sdf_model import build_geometry, write_flight_model, write_model  # noqa: E402

AIRCRAFT = ["research_quad", "m400_reference_quad", "dji_matrice_400"]
FLIGHT_MODELS = ["dji_matrice_400"]  # also generate <name>_flight (Gazebo-physics variant)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", choices=AIRCRAFT, action="append")
    ap.add_argument("--out-dir", type=Path, default=MODELS_DIR)
    args = ap.parse_args()
    for name in args.aircraft or AIRCRAFT:
        for p in write_model(name, args.out_dir) + (write_flight_model(name, args.out_dir) if name in FLIGHT_MODELS else []):
            print(f"wrote {p}")
        geo = build_geometry(name)
        lo, hi = geo.airframe_extent()
        size = hi - lo
        print(f"  {name}: mass {geo.aircraft.mass:g} kg, airframe extent (excl. rotor disks) "
              f"{size[0] * 1000:.0f} x {size[1] * 1000:.0f} x {size[2] * 1000:.0f} mm")
        if geo.envelope is not None:
            e = geo.envelope.size * 1000
            print(f"  published envelope {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} mm (drawn as a translucent box)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
