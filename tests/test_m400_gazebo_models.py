"""Gazebo models of the DJI Matrice 400 reference: static replay model and Gazebo-physics flight model."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from graduation_project.config import load_aircraft
from graduation_project.gazebo import MODELS_DIR, WORLDS_DIR
from graduation_project.gazebo.facade_world import M400_WORLDS, write_world
from graduation_project.gazebo.sdf_model import build_geometry, flight_model_sdf, model_sdf, write_flight_model, write_model


@pytest.fixture(scope="module")
def geo():
    return build_geometry("dji_matrice_400")


def test_static_model_matches_spec(geo) -> None:
    ac = load_aircraft("dji_matrice_400")
    m = ET.fromstring(model_sdf(geo).encode()).find("model")
    assert float(m.findtext("link/inertial/mass")) == pytest.approx(9.74)
    frames = {f.get("name") for f in m.findall("frame")}
    assert {f"rotor_{i}" for i in range(1, 5)} <= frames
    names = {v.get("name") for v in m.findall("link/visual")}
    assert {"top_lidar", "gnss_antenna_left", "payload_gimbal", "fpv_camera", "published_envelope"} <= names
    assert "NOT a DJI Matrice 400 digital twin" in model_sdf(geo)
    lo, hi = geo.airframe_extent()
    assert hi[2] - lo[2] == pytest.approx(0.48, abs=1e-9)  # published height
    assert ac.rotor.n_rotors == 4


def test_flight_model_structure_mass_and_propulsion(geo) -> None:
    ac = geo.aircraft
    m = ET.fromstring(flight_model_sdf(geo).encode()).find("model")
    links = m.findall("link")
    joints = m.findall("joint")
    assert [l.get("name") for l in links] == ["base_link", "rotor_1", "rotor_2", "rotor_3", "rotor_4"]
    assert all(j.get("type") == "revolute" and j.findtext("axis/xyz") == "0 0 1" for j in joints) and len(joints) == 4
    total = sum(float(l.findtext("inertial/mass")) for l in links)
    assert total == pytest.approx(ac.mass, rel=1e-9)  # total mass preserved
    # composite inertia about the CoM equals the spec inertia (rotor point masses + disks re-added)
    J = np.zeros((3, 3))
    for l in links:
        I = l.find("inertial/inertia")
        Jl = np.array([[float(I.findtext(k)) for k in row] for row in (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz"))])
        pose = l.findtext("pose")
        r = np.array([float(x) for x in pose.split()[:3]]) if pose else np.zeros(3)
        mass = float(l.findtext("inertial/mass"))
        J += Jl + (mass * (r @ r * np.eye(3) - np.outer(r, r)) if pose else 0)
    disks = 4 * np.diag([0.05 * 0.3175**2 / 4, 0.05 * 0.3175**2 / 4, 0.05 * 0.3175**2 / 2])
    np.testing.assert_allclose(J - disks, ac.mass_properties.inertia, atol=1e-9)
    motors = [p for p in m.findall("plugin") if "MulticopterMotorModel" in p.get("name")]
    ctrl = [p for p in m.findall("plugin") if "MulticopterVelocityControl" in p.get("name")]
    assert len(motors) == 4 and len(ctrl) == 1
    for p, s in zip(motors, ac.rotor.spin_dirs):
        assert float(p.findtext("motorConstant")) == pytest.approx(ac.rotor.k_f, rel=1e-6)
        assert float(p.findtext("momentConstant")) == pytest.approx(ac.rotor.k_m / ac.rotor.k_f, rel=1e-6)
        assert float(p.findtext("maxRotVelocity")) == pytest.approx(ac.rotor.omega_max, rel=1e-6)
        assert float(p.findtext("timeConstantUp")) == pytest.approx(ac.rotor.tau_motor)
        assert p.findtext("turningDirection") == ("ccw" if s > 0 else "cw")
    dirs = [int(r.findtext("direction")) for r in ctrl[0].findall("rotorConfiguration/rotor")]
    assert dirs == [int(s) for s in ac.rotor.spin_dirs]


def test_generated_m400_files_are_current(tmp_path) -> None:
    a = [p.read_bytes() for p in write_model("dji_matrice_400", tmp_path) + write_flight_model("dji_matrice_400", tmp_path)]
    committed = [(MODELS_DIR / d / f).read_bytes() for d in ("dji_matrice_400", "dji_matrice_400_flight")
                 for f in ("model.sdf", "model.config")]
    assert committed == a, "run: python3 simulation/models/generate_models.py"
    for spec in M400_WORLDS:
        new = [p.read_bytes() for p in write_world(spec, tmp_path / "w")]
        old = [(WORLDS_DIR / f"{spec.name}{ext}").read_bytes() for ext in (".sdf", ".geometry.json")]
        assert old == new, "run: python3 simulation/worlds/generate_facade_world.py"
