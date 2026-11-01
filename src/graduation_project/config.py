"""Aircraft parameter files: loading, provenance and derivation of model coefficients.

Every scalar in an aircraft YAML file is written as ``{value, units, source, note}`` where
``source`` is one of ``manufacturer``, ``standard``, ``literature``, ``derived`` or ``assumed``.
Derived model coefficients are computed here (see docs/model_equations.md, section 3.2):

* ``k_f = C_T * rho * D^4 / (2 pi)^2``  from ``T = C_T rho n^2 D^4`` with ``n = Omega / 2 pi``
* ``k_m = C_P * rho * D^5 / (2 pi)^3``  from ``P = C_P rho n^3 D^5 = Q Omega``
* ``Omega_max = sqrt(T_max / k_f)``, ``T_max = TWR * m_ref * g / n_rotors``
* ``Omega_min = idle_fraction * Omega_max``
* inertia from :func:`lumped_mass_properties` of the listed components.
"""

from __future__ import annotations

import copy
import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from graduation_project.actuators.rotor import RotorConfig, x_config_positions
from graduation_project.dynamics.frames import FloatArray
from graduation_project.dynamics.mass_properties import Component, MassProperties, lumped_mass_properties
from graduation_project.energy.power import PowerParams
from graduation_project.environment.aero import DragParams

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "config"
VALID_SOURCES = {"manufacturer", "standard", "literature", "derived", "assumed"}
WH_TO_J = 3600.0


def value(node: Any) -> Any:
    """Return the numeric payload of a provenance-annotated YAML node."""
    if isinstance(node, dict) and "value" in node:
        source = node.get("source")
        if source not in VALID_SOURCES:
            raise ValueError(f"parameter {node} lacks a valid 'source' ({sorted(VALID_SOURCES)})")
        return node["value"]
    return node


def load_yaml(path: str | Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


@dataclass(frozen=True)
class Limits:
    max_tilt: float  # [rad]
    max_body_rate: float  # roll/pitch rate [rad/s]
    max_yaw_rate: float  # [rad/s]


@dataclass(frozen=True)
class AircraftParams:
    name: str
    model_class: str
    gravity: float  # [m/s^2]
    air_density: float  # [kg/m^3]
    mass_properties: MassProperties
    rotor: RotorConfig
    rotor_diameter: float  # [m]
    drag: DragParams
    power: PowerParams
    limits: Limits
    raw: dict[str, Any]  # unmodified YAML content, saved with every run

    @property
    def mass(self) -> float:
        return self.mass_properties.mass

    @property
    def hover_thrust_per_rotor(self) -> float:
        return self.mass * self.gravity / self.rotor.n_rotors

    @property
    def hover_speed(self) -> float:
        """Rotor speed at which a symmetric configuration's thrust equals its weight [rad/s]."""
        return float(np.sqrt(self.hover_thrust_per_rotor / self.rotor.k_f))


def _components(node: dict[str, Any], hubs: FloatArray) -> list[Component]:
    """Build mass components.

    * ``at_rotors: f`` replicates the component at ``f * hub_position`` for every rotor
      (``f = 1`` motor + propeller at the hub, ``f = 0.5`` arm centroid).
    * ``mass: remainder`` assigns ``total_mass - sum(other masses)`` so the total equals
      the documented ``total_mass`` exactly.
    """
    total = float(value(node["total_mass"]))
    zero = {"value": [0.0, 0.0, 0.0], "source": "assumed"}
    fixed: list[Component] = []
    remainder: dict[str, Any] | None = None
    for n in node["components"]:
        if value(n["mass"]) == "remainder":
            remainder = n
            continue
        size = tuple(float(x) for x in value(n.get("size", zero)))
        if "at_rotors" in n:
            for i, hub in enumerate(hubs):
                pos = tuple(float(x) for x in float(n["at_rotors"]) * hub)
                fixed.append(Component(f"{n['name']}_{i + 1}", float(value(n["mass"])), pos, size))
        else:
            pos = tuple(float(x) for x in value(n["position"]))
            fixed.append(Component(n["name"], float(value(n["mass"])), pos, size))
    if remainder is not None:
        rest = total - sum(c.mass for c in fixed)
        if rest <= 0.0:
            raise ValueError("component masses exceed total_mass")
        fixed.append(
            Component(
                remainder["name"],
                rest,
                tuple(float(x) for x in value(remainder["position"])),
                tuple(float(x) for x in value(remainder.get("size", zero))),
            )
        )
    if not np.isclose(sum(c.mass for c in fixed), total, rtol=1e-9):
        raise ValueError("component masses do not sum to total_mass")
    return fixed


def aircraft_from_dict(d: dict[str, Any]) -> AircraftParams:
    env = d["environment"]
    g = float(value(env["gravity"]))
    rho = float(value(env["air_density"]))

    geo = d["geometry"]
    if geo["layout"] != "x_quad":
        raise ValueError(f"unsupported layout {geo['layout']!r}")
    # Hub positions about the geometric centre, then shifted so the body origin is the CoM.
    hubs = x_config_positions(float(value(geo["arm_length"])), float(value(geo["rotor_height"])))
    mp, com = lumped_mass_properties(_components(d["mass_properties"], hubs))
    positions = hubs - com

    prop = d["propulsion"]
    diameter = float(value(prop["rotor_diameter"]))
    c_t = float(value(prop["thrust_coefficient"]))
    c_p = float(value(prop["power_coefficient"]))
    k_f = c_t * rho * diameter**4 / (2.0 * np.pi) ** 2
    k_m = c_p * rho * diameter**5 / (2.0 * np.pi) ** 3
    n_rotors = positions.shape[0]
    t_max = float(value(prop["max_thrust_to_weight"])) * float(value(prop["max_thrust_reference_mass"])) * g / n_rotors
    omega_max = float(np.sqrt(t_max / k_f))
    rotor = RotorConfig(
        positions=positions,
        spin_dirs=np.asarray(geo["spin_dirs"], dtype=float),
        k_f=k_f,
        k_m=k_m,
        tau_motor=float(value(prop["motor_time_constant"])),
        omega_min=float(value(prop["idle_speed_fraction"])) * omega_max,
        omega_max=omega_max,
    )

    drag = DragParams(np.asarray(value(d["drag"]["linear"]), float), np.asarray(value(d["drag"]["quadratic"]), float))
    pw = d["power"]
    power = PowerParams(
        propulsion_efficiency=float(value(pw["propulsion_efficiency"])),
        avionics_power=float(value(pw["avionics_power"])),
        battery_energy=float(value(pw["battery_energy_wh"])) * WH_TO_J,
    )
    lim = d["limits"]
    limits = Limits(
        max_tilt=np.deg2rad(float(value(lim["max_tilt_deg"]))),
        max_body_rate=np.deg2rad(float(value(lim["max_body_rate_deg_s"]))),
        max_yaw_rate=np.deg2rad(float(value(lim["max_yaw_rate_deg_s"]))),
    )
    return AircraftParams(
        name=d["name"],
        model_class=d["model_class"],
        gravity=g,
        air_density=rho,
        mass_properties=mp,
        rotor=rotor,
        rotor_diameter=diameter,
        drag=drag,
        power=power,
        limits=limits,
        raw=copy.deepcopy(d),
    )


def load_aircraft(name_or_path: str | Path) -> AircraftParams:
    """Load ``config/aircraft/<name>.yaml`` or an explicit path."""
    path = Path(name_or_path)
    if not path.suffix:
        path = CONFIG_DIR / "aircraft" / f"{name_or_path}.yaml"
    raw = load_yaml(path)
    if raw.get("schema_version") == 2:  # traceable specification database (aircraft_spec.py)
        from graduation_project.aircraft_spec import load_spec, spec_to_v1

        spec = load_spec(path)
        return dataclasses.replace(aircraft_from_dict(spec_to_v1(spec)), raw=spec)
    return aircraft_from_dict(raw)


def derived_summary(ac: AircraftParams) -> dict[str, float]:
    """Human-readable derived quantities, used by scripts/derive_parameters.py and the docs."""
    hover_power_shaft = ac.rotor.n_rotors * ac.rotor.k_m * ac.hover_speed**3
    return {
        "mass_kg": ac.mass,
        "Jxx_kgm2": float(ac.mass_properties.inertia[0, 0]),
        "Jyy_kgm2": float(ac.mass_properties.inertia[1, 1]),
        "Jzz_kgm2": float(ac.mass_properties.inertia[2, 2]),
        "k_f_N_per_radps2": ac.rotor.k_f,
        "k_m_Nm_per_radps2": ac.rotor.k_m,
        "omega_hover_radps": ac.hover_speed,
        "omega_hover_rpm": ac.hover_speed * 60.0 / (2.0 * np.pi),
        "omega_max_rpm": ac.rotor.omega_max * 60.0 / (2.0 * np.pi),
        "thrust_max_per_rotor_N": ac.rotor.thrust_max,
        "hover_thrust_fraction": ac.hover_thrust_per_rotor / ac.rotor.thrust_max,
        "hover_shaft_power_W": hover_power_shaft,
        "hover_electrical_power_W": hover_power_shaft / ac.power.propulsion_efficiency + ac.power.avionics_power,
    }
