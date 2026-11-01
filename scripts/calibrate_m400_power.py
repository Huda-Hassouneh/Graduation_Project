#!/usr/bin/env python3
"""Reproduce the calibration of the M400 power coefficient C_P (ENGINEERING_ESTIMATE in the spec).

Target: modelled hover electrical power = usable_energy_fraction * 977 Wh / 53 min (published max hover
time, no wind) at the published 9.74 kg take-off mass. With the assumed C_T, propulsion efficiency and
avionics power:

    Omega_h = sqrt(m g / (n k_f)),  k_f = C_T rho D^4 / (2 pi)^2
    shaft   = (P_el - P_avionics) * eta
    k_m     = shaft / (n Omega_h^3),   C_P = k_m (2 pi)^3 / (rho D^5)

This is a CALIBRATION to a manufacturer endurance claim, not a validation: the same claim cannot be
used to check the result. The 59 min flight-time claim involves forward flight (translational lift),
which the model does not represent, so it is not a usable independent check either.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.aircraft_spec import load_spec, sval  # noqa: E402


def calibrated_cp(spec: dict, usable: float | None = None) -> dict[str, float]:
    rho = float(sval(spec, "simulation_assumptions.air_density_sea_level"))
    g = float(sval(spec, "simulation_assumptions.gravity"))
    D = float(sval(spec, "propulsion.rotor_diameter"))
    ct = float(sval(spec, "propulsion.thrust_coefficient"))
    eta = float(sval(spec, "propulsion.propulsion_efficiency"))
    p_av = float(sval(spec, "simulation_assumptions.avionics_power"))
    m = float(sval(spec, "mass_properties.takeoff_mass_with_battery"))
    n = int(sval(spec, "geometry.n_rotors"))
    e_wh = float(sval(spec, "battery.energy"))
    t_h = float(sval(spec, "battery.max_hover_time")) / 60.0
    u = float(sval(spec, "simulation_assumptions.usable_energy_fraction")) if usable is None else usable
    kf = ct * rho * D**4 / (2 * np.pi) ** 2
    w_h = np.sqrt(m * g / (n * kf))
    p_el = u * e_wh / t_h
    shaft = (p_el - p_av) * eta
    km = shaft / (n * w_h**3)
    cp = km * (2 * np.pi) ** 3 / (rho * D**5)
    fm = (m * g) ** 1.5 / np.sqrt(2 * rho * n * np.pi * (D / 2) ** 2) / shaft
    return {"usable_fraction": u, "hover_electrical_W": p_el, "hover_shaft_W": shaft, "omega_hover_rpm": w_h * 60 / (2 * np.pi),
            "k_f": kf, "k_m": km, "C_P": cp, "figure_of_merit": fm}


def main() -> int:
    spec = load_spec("dji_matrice_400")
    for u in (0.90, None, 1.0):
        r = calibrated_cp(spec, u)
        print("  ".join(f"{k}={v:.5g}" for k, v in r.items()))
    print(f"spec value C_P = {sval(spec, 'propulsion.power_coefficient')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
