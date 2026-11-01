"""Traceable aircraft specification database (schema v2): validation, export and model building.

A schema-v2 aircraft file (``config/aircraft/dji_matrice_400.yaml``) stores every parameter as a
leaf ``{value, units, category, sources, confidence, ...}``. This module

* walks and validates the leaves (provenance rules below),
* exports them as a flat, deterministic JSON parameter list, and
* converts the subset the dynamics need into the schema-v1 dictionary consumed by
  :func:`graduation_project.config.aircraft_from_dict`, so the verified Milestone 1 model code is
  reused unchanged.

Provenance rules enforced by :func:`validate_spec`:

* ``category`` is one of :data:`CATEGORIES`; ``units`` and ``confidence`` are present.
* every id in ``sources`` exists in the file's ``sources`` section.
* ``UNKNOWN`` / ``NOT_PUBLICLY_DISCLOSED`` leaves have ``value: null`` (never a guess). If the
  simulation needs a stand-in, ``simulation_value`` names the estimate/assumption that provides it.
* every other category has a non-null value.
* numeric ``ENGINEERING_ESTIMATE`` / ``SIMULATION_ASSUMPTION`` leaves have ``range: [min, max]``
  (element-wise for vectors) containing the value, plus ``rationale`` and ``sensitivity``.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterator

from graduation_project.config import CONFIG_DIR, load_yaml

VERIFIED_OFFICIAL = "VERIFIED_OFFICIAL"
VERIFIED_SECONDARY = "VERIFIED_SECONDARY"
ENGINEERING_ESTIMATE = "ENGINEERING_ESTIMATE"
SIMULATION_ASSUMPTION = "SIMULATION_ASSUMPTION"
UNKNOWN = "UNKNOWN"
NOT_PUBLICLY_DISCLOSED = "NOT_PUBLICLY_DISCLOSED"
CATEGORIES = (VERIFIED_OFFICIAL, VERIFIED_SECONDARY, ENGINEERING_ESTIMATE, SIMULATION_ASSUMPTION, UNKNOWN,
              NOT_PUBLICLY_DISCLOSED)
NULL_CATEGORIES = {UNKNOWN, NOT_PUBLICLY_DISCLOSED}
ESTIMATE_CATEGORIES = {ENGINEERING_ESTIMATE, SIMULATION_ASSUMPTION}
CONFIDENCE = {"high", "medium", "low"}
# Mapping onto the schema-v1 `source` vocabulary used by config.value().
_V1_SOURCE = {VERIFIED_OFFICIAL: "manufacturer", VERIFIED_SECONDARY: "manufacturer",
              ENGINEERING_ESTIMATE: "derived", SIMULATION_ASSUMPTION: "assumed"}


@dataclass(frozen=True)
class SpecParameter:
    path: str
    value: Any
    units: str
    category: str
    sources: tuple[str, ...]
    confidence: str
    range: tuple[float, float] | None = None
    rationale: str | None = None
    sensitivity: str | None = None
    derivation: str | None = None
    note: str | None = None
    simulation_value: str | None = None


def is_parameter(node: Any) -> bool:
    return isinstance(node, dict) and "category" in node


def _walk(node: Any, path: str) -> Iterator[tuple[str, dict[str, Any]]]:
    if is_parameter(node):
        yield path, node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from _walk(v, f"{path}.{k}" if path else str(k))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            tag = v.get("name", str(i)) if isinstance(v, dict) else str(i)
            yield from _walk(v, f"{path}[{tag}]")


def iter_parameters(spec: dict[str, Any]) -> Iterator[SpecParameter]:
    for section, node in spec.items():
        if section in ("sources", "schema_version", "name", "model_class", "display_name"):
            continue
        for path, leaf in _walk(node, section):
            rng = leaf.get("range")
            yield SpecParameter(
                path=path, value=leaf.get("value"), units=str(leaf.get("units", "")), category=leaf["category"],
                sources=tuple(leaf.get("sources", ())), confidence=str(leaf.get("confidence", "")),
                range=tuple(float(x) for x in rng) if rng is not None else None,
                rationale=leaf.get("rationale"), sensitivity=leaf.get("sensitivity"), derivation=leaf.get("derivation"),
                note=leaf.get("note"), simulation_value=leaf.get("simulation_value"),
            )


def _numeric(v: Any) -> bool:
    if isinstance(v, bool):
        return False
    if isinstance(v, (int, float)):
        return True
    return isinstance(v, list) and len(v) > 0 and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)


def resolve_path(spec: dict[str, Any], dotted: str) -> Any:
    node: Any = spec
    for key in dotted.split("."):
        if not isinstance(node, dict) or key not in node:
            raise KeyError(dotted)
        node = node[key]
    return node


def validate_spec(spec: dict[str, Any]) -> list[str]:
    """Return a list of provenance violations (empty if the file is valid)."""
    errors: list[str] = []
    if spec.get("schema_version") != 2:
        errors.append("schema_version must be 2")
    known = set(spec.get("sources", {}))
    for p in iter_parameters(spec):
        where = p.path
        if p.category not in CATEGORIES:
            errors.append(f"{where}: invalid category {p.category!r}")
            continue
        if not p.units:
            errors.append(f"{where}: missing units")
        if p.confidence not in CONFIDENCE:
            errors.append(f"{where}: confidence must be one of {sorted(CONFIDENCE)}")
        if not p.sources:
            errors.append(f"{where}: no sources")
        for s in p.sources:
            if s not in known:
                errors.append(f"{where}: unknown source id {s!r}")
        if p.category in NULL_CATEGORIES:
            if p.value is not None:
                errors.append(f"{where}: {p.category} must have value null (no guessed values)")
        elif p.value is None:
            errors.append(f"{where}: {p.category} requires a value")
        if p.simulation_value is not None:
            try:
                resolve_path(spec, p.simulation_value)
            except KeyError:
                errors.append(f"{where}: simulation_value {p.simulation_value!r} does not resolve")
        if p.category in ESTIMATE_CATEGORIES and _numeric(p.value):
            if p.range is None or len(p.range) != 2 or p.range[0] > p.range[1]:
                errors.append(f"{where}: numeric estimate needs range [min, max]")
            else:
                vals = p.value if isinstance(p.value, list) else [p.value]
                if any(not (p.range[0] - 1e-12 <= float(v) <= p.range[1] + 1e-12) for v in vals):
                    errors.append(f"{where}: value {p.value} outside range {list(p.range)}")
            if not p.rationale:
                errors.append(f"{where}: estimate needs a rationale")
            if not p.sensitivity:
                errors.append(f"{where}: estimate needs a sensitivity statement")
    return errors


def load_spec(name_or_path: str | Path) -> dict[str, Any]:
    path = Path(name_or_path)
    if not path.suffix:
        path = CONFIG_DIR / "aircraft" / f"{name_or_path}.yaml"
    spec = load_yaml(path)
    errors = validate_spec(spec)
    if errors:
        raise ValueError(f"{path}: invalid specification:\n  " + "\n  ".join(errors))
    return spec


def sval(spec: dict[str, Any], dotted: str) -> Any:
    """Value of a parameter leaf addressed by a dotted path (raises if the leaf is null)."""
    leaf = resolve_path(spec, dotted)
    if not is_parameter(leaf):
        raise KeyError(f"{dotted} is not a parameter leaf")
    if leaf["value"] is None:
        raise ValueError(f"{dotted} is {leaf['category']}: use its simulation_value ({leaf.get('simulation_value')})")
    return leaf["value"]


def _v1(leaf: dict[str, Any]) -> dict[str, Any]:
    src = "standard" if "ISO-2533" in leaf.get("sources", ()) else _V1_SOURCE[leaf["category"]]
    return {"value": leaf["value"], "units": leaf.get("units"), "source": src}


def spec_to_v1(spec: dict[str, Any]) -> dict[str, Any]:
    """Schema-v1 dictionary for :func:`graduation_project.config.aircraft_from_dict`."""
    g, mp, pr = spec["geometry"], spec["mass_properties"], spec["propulsion"]
    sa, lim = spec["simulation_assumptions"], spec["limits"]
    if sval(spec, "geometry.n_rotors") != 4:
        raise ValueError("only the four-rotor X layout is supported by the v1 model builder")
    comps = []
    for c in mp["components"]:
        out: dict[str, Any] = {"name": c["name"]}
        if "at_rotors" in c:
            out["at_rotors"] = c["at_rotors"]
        out["mass"] = {"value": "remainder", "source": "derived"} if c["mass"]["value"] == "remainder" else _v1(c["mass"])
        for key in ("position", "size"):
            if key in c:
                out[key] = _v1(c[key])
        comps.append(out)
    battery_wh = spec["battery"]["energy"]
    return {
        "name": spec["name"],
        "model_class": spec["model_class"],
        "environment": {"gravity": _v1(sa["gravity"]), "air_density": _v1(sa["air_density_sea_level"])},
        "mass_properties": {"total_mass": _v1(mp["total_mass"]), "components": comps},
        "geometry": {"layout": "x_quad", "arm_length": _v1(g["arm_length"]), "rotor_height": _v1(g["rotor_height"]),
                     "spin_dirs": [int(s) for s in g["spin_dirs"]["value"]]},
        "propulsion": {k: _v1(pr[k]) for k in ("rotor_diameter", "thrust_coefficient", "power_coefficient",
                                               "max_thrust_to_weight", "max_thrust_reference_mass",
                                               "idle_speed_fraction", "motor_time_constant")},
        "drag": {"linear": _v1(sa["drag_linear"]), "quadratic": _v1(sa["drag_quadratic"])},
        "power": {"propulsion_efficiency": _v1(pr["propulsion_efficiency"]), "avionics_power": _v1(sa["avionics_power"]),
                  "battery_energy_wh": _v1(battery_wh)},
        "limits": {k: _v1(lim[k]) for k in ("max_tilt_deg", "max_body_rate_deg_s", "max_yaw_rate_deg_s")},
    }


def export_parameters_json(spec: dict[str, Any]) -> str:
    """Flat, deterministic JSON list of every parameter (``config/aircraft/<name>_parameters.json``)."""
    params = [{k: v for k, v in asdict(p).items() if v is not None} for p in iter_parameters(spec)]
    doc = {
        "generated_by": "scripts/export_m400_spec.py",
        "aircraft": spec["name"],
        "schema_version": spec["schema_version"],
        "categories": list(CATEGORIES),
        "sources": spec["sources"],
        "category_counts": category_counts(spec),
        "parameters": params,
    }
    return json.dumps(doc, indent=2, sort_keys=False, ensure_ascii=False) + "\n"


def category_counts(spec: dict[str, Any]) -> dict[str, int]:
    counts = {c: 0 for c in CATEGORIES}
    for p in iter_parameters(spec):
        counts[p.category] += 1
    return counts
