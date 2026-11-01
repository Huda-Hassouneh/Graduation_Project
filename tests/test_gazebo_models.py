"""Gazebo SDF models: configuration-to-SDF mapping, provenance labels, determinism."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from graduation_project.config import VALID_SOURCES, load_aircraft, load_yaml
from graduation_project.gazebo import MODELS_DIR, VISUAL_CONFIG_DIR
from graduation_project.gazebo.sdf_model import build_geometry, model_config, model_sdf, write_model

AIRCRAFT = ["research_quad", "m400_reference_quad"]


def _floats(text: str) -> np.ndarray:
    return np.array([float(x) for x in text.split()])


@pytest.fixture(params=AIRCRAFT)
def name(request: pytest.FixtureRequest) -> str:
    return request.param


@pytest.fixture
def model(name: str) -> ET.Element:
    return ET.fromstring(model_sdf(build_geometry(name)).encode()).find("model")


def test_mass_and_inertia_come_from_aircraft_config(name: str, model: ET.Element) -> None:
    ac = load_aircraft(name)
    inertial = model.find("link/inertial")
    assert inertial.find("pose") is None  # link origin = CoM
    assert float(inertial.findtext("mass")) == pytest.approx(ac.mass, rel=1e-9)
    J = ac.mass_properties.inertia
    keys = {"ixx": (0, 0), "ixy": (0, 1), "ixz": (0, 2), "iyy": (1, 1), "iyz": (1, 2), "izz": (2, 2)}
    for k, (i, j) in keys.items():
        assert float(inertial.findtext(f"inertia/{k}")) == pytest.approx(J[i, j], rel=1e-8, abs=1e-12)


def test_rotor_frames_and_spin_directions_match_config(name: str, model: ET.Element) -> None:
    ac = load_aircraft(name)
    frames = {f.get("name"): f for f in model.findall("frame")}
    visuals = {v.get("name") for v in model.findall("link/visual")}
    for i, (hub, s) in enumerate(zip(ac.rotor.positions, ac.rotor.spin_dirs), start=1):
        f = frames[f"rotor_{i}"]
        assert f.get("attached_to") == "base_link"
        np.testing.assert_allclose(_floats(f.findtext("pose"))[:3], hub, atol=1e-8)
        assert f"rotor_{i}_{'ccw' if s > 0 else 'cw'}_disk" in visuals
    assert len([k for k in frames if k.startswith("rotor_")]) == ac.rotor.n_rotors


def test_rotor_disk_diameter_matches_config(name: str, model: ET.Element) -> None:
    ac = load_aircraft(name)
    disks = [v for v in model.findall("link/visual") if v.get("name").endswith("_disk")]
    assert len(disks) == ac.rotor.n_rotors
    for v in disks:
        assert 2 * float(v.findtext("geometry/cylinder/radius")) == pytest.approx(ac.rotor_diameter)


def test_geometric_centre_frame_is_minus_com(name: str, model: ET.Element) -> None:
    geo = build_geometry(name)
    f = next(f for f in model.findall("frame") if f.get("name") == "geometric_centre")
    np.testing.assert_allclose(_floats(f.findtext("pose"))[:3], -geo.com_geometric, atol=1e-8)


def test_landing_gear_is_lowest_point(name: str) -> None:
    geo = build_geometry(name)
    lo, _ = geo.airframe_extent()
    assert geo.gear_bottom_z == pytest.approx(lo[2])
    assert geo.gear_bottom_z < 0.0


def test_m400_reference_envelope_and_disclaimer() -> None:
    geo = build_geometry("m400_reference_quad")
    np.testing.assert_allclose(geo.envelope.size, [0.98, 0.76, 0.48])
    lo, hi = geo.airframe_extent()
    size = hi - lo
    assert size[2] == pytest.approx(0.48, abs=1e-9)  # gear to mast
    assert size[0] <= 0.98 + 1e-9
    # A square X with a 1070 mm diagonal puts hub centres 757 mm apart, so motor housings
    # necessarily exceed the 760 mm published width (documented in docs/gazebo_milestone3.md).
    assert 0.76 < size[1] < 0.76 * 1.15
    sdf = model_sdf(geo)
    assert "NOT a DJI Matrice 400 digital twin" in sdf
    assert "NOT a DJI Matrice 400 digital twin" in model_config(geo)


def test_research_quad_has_no_manufacturer_values() -> None:
    geo = build_geometry("research_quad")
    assert geo.envelope is None
    assert "manufacturer" not in (VISUAL_CONFIG_DIR / "research_quad.yaml").read_text()


@pytest.mark.parametrize("aircraft", AIRCRAFT)
def test_visual_config_declares_source_and_units(aircraft: str) -> None:
    def leaves(node):
        if isinstance(node, dict):
            if "value" in node:
                yield node
            else:
                for v in node.values():
                    yield from leaves(v)
        elif isinstance(node, list):
            for v in node:
                yield from leaves(v)

    found = list(leaves(load_yaml(VISUAL_CONFIG_DIR / f"{aircraft}.yaml")))
    assert len(found) >= 9
    for node in found:
        assert node["source"] in VALID_SOURCES and "units" in node


def test_static_flag(name: str) -> None:
    geo = build_geometry(name)
    assert "<static>true</static>" in model_sdf(geo, static=True)
    assert "<static>false</static>" in model_sdf(geo)


def test_generation_is_deterministic_and_committed_files_are_current(name: str, tmp_path) -> None:
    a = [p.read_bytes() for p in write_model(name, tmp_path / "a")]
    b = [p.read_bytes() for p in write_model(name, tmp_path / "b")]
    assert a == b
    committed = [(MODELS_DIR / name / f).read_bytes() for f in ("model.sdf", "model.config")]
    assert committed == a, "run: python3 simulation/models/generate_models.py"
