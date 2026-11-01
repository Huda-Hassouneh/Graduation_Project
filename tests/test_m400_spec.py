"""DJI Matrice 400 specification database: provenance rules, verified values, derived model, JSON export."""

import copy
import importlib.util
import json

import numpy as np
import pytest

from graduation_project.aircraft_spec import (
    CATEGORIES,
    NULL_CATEGORIES,
    category_counts,
    export_parameters_json,
    iter_parameters,
    load_spec,
    sval,
    validate_spec,
)
from graduation_project.config import CONFIG_DIR, REPO_ROOT, derived_summary, load_aircraft


@pytest.fixture(scope="module")
def spec():
    return load_spec("dji_matrice_400")


def test_spec_is_valid_and_every_parameter_has_provenance(spec) -> None:
    assert validate_spec(spec) == []
    params = list(iter_parameters(spec))
    assert len(params) > 100
    for p in params:
        assert p.category in CATEGORIES and p.sources and p.units


def test_unknown_values_are_never_guessed(spec) -> None:
    for p in iter_parameters(spec):
        if p.category in NULL_CATEGORIES:
            assert p.value is None, p.path


def test_validator_rejects_guessed_or_unsourced_values(spec) -> None:
    bad = copy.deepcopy(spec)
    bad["propulsion"]["max_thrust"]["value"] = 70.0  # NOT_PUBLICLY_DISCLOSED with a guessed value
    bad["geometry"]["rotor_height"].pop("range")  # estimate without range
    bad["battery"]["energy"]["sources"] = ["NOPE"]
    errs = validate_spec(bad)
    assert any("max_thrust" in e for e in errs)
    assert any("rotor_height" in e and "range" in e for e in errs)
    assert any("unknown source id" in e for e in errs)


def test_verified_official_values_match_dji_specification(spec) -> None:
    expected = {"mass_properties.takeoff_mass_with_battery": 9.74, "mass_properties.mass_without_battery": 5.02,
                "mass_properties.max_takeoff_mass": 15.8, "geometry.diagonal_wheelbase": 1.070,
                "propulsion.rotor_diameter": 0.635, "battery.energy": 977.0, "battery.capacity": 20.254,
                "battery.nominal_voltage": 48.23, "battery.max_charge_voltage": 54.6, "battery.series_cells": 13,
                "limits.max_tilt_deg": 35.0, "limits.max_yaw_rate_deg_s": 100.0, "limits.max_wind_resistance": 12.0}
    for path, val in expected.items():
        leaf = spec
        for k in path.split("."):
            leaf = leaf[k]
        assert leaf["category"] == "VERIFIED_OFFICIAL", path
        assert sval(spec, path) == pytest.approx(val), path


def test_mass_consistency_and_quadrotor_geometry(spec) -> None:
    assert sval(spec, "mass_properties.mass_without_battery") + sval(spec, "battery.mass") == pytest.approx(
        sval(spec, "mass_properties.takeoff_mass_with_battery"))
    assert sval(spec, "geometry.n_rotors") == 4
    # coplanar hexagon at the published wheelbase would overlap the published propellers; square X fits
    r, d = sval(spec, "geometry.diagonal_wheelbase") / 2, sval(spec, "propulsion.rotor_diameter")
    assert 2 * r * np.sin(np.pi / 6) < d < 2 * r * np.sin(np.pi / 4)
    assert 2 * r * np.sin(np.pi / 4) == pytest.approx(sval(spec, "geometry.dimensions_unfolded_lwh")[1], abs=0.005)


def test_model_derivation_and_power_calibration(spec) -> None:
    ac = load_aircraft("dji_matrice_400")
    assert ac.mass == pytest.approx(9.74) and ac.rotor.n_rotors == 4
    np.testing.assert_allclose(np.linalg.norm(ac.rotor.positions[:, :2], axis=1), 0.535, atol=1e-9)
    s = derived_summary(ac)
    usable = sval(spec, "simulation_assumptions.usable_energy_fraction")
    target = usable * sval(spec, "battery.energy") / (sval(spec, "battery.max_hover_time") / 60.0)
    assert s["hover_electrical_power_W"] == pytest.approx(target, rel=2e-3)
    spec_mod = importlib.util.spec_from_file_location("cal", REPO_ROOT / "scripts" / "calibrate_m400_power.py")
    cal = importlib.util.module_from_spec(spec_mod)
    spec_mod.loader.exec_module(cal)
    assert cal.calibrated_cp(spec)["C_P"] == pytest.approx(sval(spec, "propulsion.power_coefficient"), rel=1e-3)


def test_reference_model_is_unchanged() -> None:
    """The new spec reuses the old m400_reference_quad lumped components: identical inertia."""
    a, b = load_aircraft("dji_matrice_400"), load_aircraft("m400_reference_quad")
    np.testing.assert_allclose(a.mass_properties.inertia, b.mass_properties.inertia)


def test_parameters_json_is_current(spec) -> None:
    committed = (CONFIG_DIR / "aircraft" / "dji_matrice_400_parameters.json").read_text(encoding="utf-8")
    assert committed == export_parameters_json(spec), "run: python3 scripts/export_m400_spec.py"
    doc = json.loads(committed)
    assert doc["category_counts"] == category_counts(spec)


def test_no_low_level_motor_interface_is_claimed(spec) -> None:
    leaf = spec["flight_control_interface"]["low_level_motor_control"]
    assert leaf["value"] is None and leaf["category"] == "NOT_PUBLICLY_DISCLOSED"
    assert "not a digital twin" in spec["display_name"]


def test_generated_parameter_table_is_current(spec) -> None:
    import importlib.util

    m = importlib.util.spec_from_file_location("exp", REPO_ROOT / "scripts" / "export_m400_spec.py")
    mod = importlib.util.module_from_spec(m)
    m.loader.exec_module(mod)
    assert mod.TABLE.read_text(encoding="utf-8") == mod.parameter_table(spec), "run: python3 scripts/export_m400_spec.py"
