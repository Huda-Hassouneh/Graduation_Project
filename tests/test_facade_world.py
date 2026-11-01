"""Facade world generator: scenario-geometry consistency, keep-out regions, determinism."""

import dataclasses
import json
import os
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from graduation_project.experiments import scenarios
from graduation_project.gazebo import MODELS_DIR, WORLDS_DIR, gazebo_env
from graduation_project.gazebo.facade_world import (
    DEFAULT_WORLDS,
    FacadeWorldSpec,
    build_layout,
    geometry_json,
    world_sdf,
    write_world,
)

FACADE_SCENARIOS = ["facade_parallel", "vertical_scan", "repeated_coverage"]


@pytest.fixture(params=DEFAULT_WORLDS, ids=lambda s: s.name)
def spec(request: pytest.FixtureRequest) -> FacadeWorldSpec:
    return request.param


def _world_boxes(spec: FacadeWorldSpec) -> list[dict]:
    return json.loads(geometry_json(spec))["boxes"]


def test_defaults_come_from_scenarios() -> None:
    s = FacadeWorldSpec(name="x")
    assert s.facade_origin == scenarios.FACADE_ORIGIN == (0.0, 8.0, 0.0)
    assert s.facade_normal == scenarios.FACADE_NORMAL
    assert s.facade_tangent == scenarios.FACADE_TANGENT
    assert s.facade_width == scenarios.FACADE_WIDTH_M == 12.0
    assert s.facade_height > scenarios.SCAN_Z_MAX_M


def test_main_face_is_the_scenario_facade(spec: FacadeWorldSpec) -> None:
    main = next(b for b in _world_boxes(spec) if b["name"] == "main_block")
    c, size = np.array(main["centre"]), np.array(main["size"])
    assert main["yaw"] == pytest.approx(0.0)
    assert c[1] - size[1] / 2 == pytest.approx(8.0, abs=1e-12)  # face plane y = 8 m
    assert c[0] - size[0] / 2 == pytest.approx(0.0, abs=1e-12)  # x in [0, 12]
    assert c[0] + size[0] / 2 == pytest.approx(12.0, abs=1e-12)
    assert c[2] - size[2] / 2 == pytest.approx(0.0, abs=1e-12)


def _inside(p: np.ndarray, box: dict, eps: float = 1e-9) -> bool:
    assert box["yaw"] == pytest.approx(0.0)
    c, h = np.array(box["centre"]), 0.5 * np.array(box["size"])
    return bool(np.all(np.abs(p - c) < h - eps))


@pytest.mark.parametrize("scenario", FACADE_SCENARIOS)
def test_baseline_references_stay_outside_solids_and_keepouts(spec: FacadeWorldSpec, scenario: str) -> None:
    traj = scenarios.SCENARIOS[scenario]().trajectory
    pts = [traj.sample(t).position for t in np.arange(0.0, traj.duration + 1e-9, 0.05)]
    np.testing.assert_allclose([p[1] for p in pts], 8.0 - scenarios.STANDOFF_M)  # 3 m stand-off plane
    boxes = [b for b in _world_boxes(spec) if b["kind"] in ("solid", "railing", "keepout")]
    for p in pts:
        hits = [b["name"] for b in boxes if _inside(p, b)]
        assert not hits, f"{scenario} reference at {p} is inside {hits}"


def test_complex_variant_has_corners_and_balconies() -> None:
    names = {b.name for b in build_layout(FacadeWorldSpec(name="c", variant="complex")).boxes}
    assert {"setback_block", "wing_block"} <= names
    assert sum(n.startswith("balcony_") and n.endswith("_railing") for n in names) == 4
    assert sum(n.startswith("keepout_balcony_") for n in names) == 4


def test_flat_variant_is_one_wall() -> None:
    lay = build_layout(FacadeWorldSpec(name="f", variant="flat"))
    assert [b.name for b in lay.boxes if b.kind == "solid"] == ["main_block"]


def test_keepouts_are_visual_only_and_solids_collide(spec: FacadeWorldSpec) -> None:
    world = ET.fromstring(world_sdf(spec).encode()).find("world")
    models = {m.get("name"): m for m in world.findall("model")}
    for link in models["keepout_zones"].findall("link"):
        assert link.find("collision") is None and link.find("visual") is not None
        assert link.get("name").startswith("keepout_")
    for link in models["facade_building"].findall("link"):
        assert link.find("collision") is not None
    assert {"ground_plane", "takeoff_pad", "facade_building", "keepout_zones"} <= set(models)


def test_takeoff_pad_and_aircraft_include(spec: FacadeWorldSpec) -> None:
    world = ET.fromstring(world_sdf(spec).encode()).find("world")
    pad = next(m for m in world.findall("model") if m.get("name") == "takeoff_pad")
    assert np.allclose([float(x) for x in pad.findtext("pose").split()][:2], 0.0)
    inc = world.find("include")
    assert inc.findtext("uri") == "model://research_quad"
    assert inc.findtext("static") == "true"
    model_dir = MODELS_DIR / "research_quad"  # model:// resolves against simulation/models
    assert (model_dir / "model.sdf").is_file() and (model_dir / "model.config").is_file()


def test_gazebo_env_prepends_models_dir_once() -> None:
    env = gazebo_env({"GZ_SIM_RESOURCE_PATH": f"/opt/a{os.pathsep}{MODELS_DIR}", "PATH": "/usr/bin"})
    assert env["GZ_SIM_RESOURCE_PATH"] == os.pathsep.join([str(MODELS_DIR), "/opt/a"])
    assert env["SDF_PATH"] == str(MODELS_DIR)
    assert env["PATH"] == "/usr/bin"
    assert gazebo_env(env) == env


def test_required_systems_are_loaded(spec: FacadeWorldSpec) -> None:
    world = ET.fromstring(world_sdf(spec).encode()).find("world")
    plugins = {p.get("filename") for p in world.findall("plugin")}
    assert {"gz-sim-physics-system", "gz-sim-user-commands-system", "gz-sim-scene-broadcaster-system"} <= plugins


def test_rotated_facade_and_handedness_check() -> None:
    s = FacadeWorldSpec(name="r", facade_origin=(5.0, 0.0, 0.0), facade_tangent=(0.0, 1.0, 0.0),
                        facade_normal=(1.0, 0.0, 0.0), aircraft=None)
    lay = build_layout(s)
    np.testing.assert_allclose(lay.to_world([12.0, 0.0, 0.0]), [5.0, 12.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(lay.to_world([0.0, 1.0, 0.0]), [4.0, 0.0, 0.0], atol=1e-12)  # into the wall (-normal)
    with pytest.raises(ValueError):
        build_layout(dataclasses.replace(s, facade_normal=(-1.0, 0.0, 0.0)))


def test_generation_is_deterministic_and_committed_files_are_current(spec: FacadeWorldSpec, tmp_path) -> None:
    a = [p.read_bytes() for p in write_world(spec, tmp_path / "a")]
    b = [p.read_bytes() for p in write_world(spec, tmp_path / "b")]
    assert a == b
    committed = [(WORLDS_DIR / f"{spec.name}{ext}").read_bytes() for ext in (".sdf", ".geometry.json")]
    assert committed == a, "run: python3 simulation/worlds/generate_facade_world.py"
