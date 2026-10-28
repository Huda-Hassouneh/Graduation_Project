"""Aircraft parameter files: provenance, manufacturer values, geometric consistency."""

import numpy as np
import pytest

from graduation_project.config import VALID_SOURCES, load_aircraft, load_yaml, CONFIG_DIR


def _leaves(node, path=""):
    if isinstance(node, dict):
        if "value" in node:
            yield path, node
        else:
            for k, v in node.items():
                yield from _leaves(v, f"{path}.{k}")
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from _leaves(v, f"{path}[{i}]")


@pytest.mark.parametrize("name", ["research_quad", "m400_reference_quad"])
def test_every_parameter_declares_source_and_units(name: str) -> None:
    d = load_yaml(CONFIG_DIR / "aircraft" / f"{name}.yaml")
    leaves = list(_leaves(d))
    assert len(leaves) > 20
    for path, node in leaves:
        assert node.get("source") in VALID_SOURCES, path
        if node["value"] != "remainder":
            assert "units" in node, path


def test_research_quad_is_not_labelled_as_reference_aircraft() -> None:
    d = load_yaml(CONFIG_DIR / "aircraft" / "research_quad.yaml")
    assert d["model_class"] == "research_quadrotor"
    assert all(node["source"] != "manufacturer" for _, node in _leaves(d))


def test_m400_reference_matches_published_specifications() -> None:
    ac = load_aircraft("m400_reference_quad")
    assert ac.model_class == "aircraft_reference_geometry"
    assert ac.mass == pytest.approx(9.74)  # takeoff weight with batteries [kg]
    assert ac.rotor_diameter == pytest.approx(25 * 0.0254)  # 25 in
    hubs = ac.rotor.positions
    diag = np.linalg.norm(hubs[0, :2] - hubs[2, :2])
    assert diag == pytest.approx(1.070, abs=1e-9)  # diagonal wheelbase [m]
    # Lateral hub spacing ~757 mm, consistent with the 760 mm published unfolded width.
    assert abs(hubs[1, 1] - hubs[0, 1]) == pytest.approx(0.757, abs=0.002)
    # Can hover at maximum takeoff weight with margin.
    assert 4 * ac.rotor.thrust_max > 15.8 * ac.gravity


def test_body_origin_is_centre_of_mass(aircraft) -> None:
    """Hub positions are expressed about the CoM: for the symmetric X layout x/y offsets cancel."""
    np.testing.assert_allclose(aircraft.rotor.positions[:, :2].sum(axis=0), 0.0, atol=1e-12)


def test_missing_source_is_rejected() -> None:
    from graduation_project.config import value

    with pytest.raises(ValueError):
        value({"value": 1.0, "units": "kg"})
