#!/usr/bin/env python3
"""Print the model coefficients derived from each aircraft YAML file (k_f, k_m, inertia, hover point)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from graduation_project.config import derived_summary, load_aircraft  # noqa: E402

for name in sys.argv[1:] or ["research_quad", "m400_reference_quad"]:
    ac = load_aircraft(name)
    print(f"{name}  ({ac.model_class})")
    for key, val in derived_summary(ac).items():
        print(f"  {key:28s} {val:.6g}")
    for i, (pos, s) in enumerate(zip(ac.rotor.positions, ac.rotor.spin_dirs)):
        print(f"  rotor {i + 1}: r_B = [{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}] m  spin {'CCW' if s > 0 else 'CW'}")
