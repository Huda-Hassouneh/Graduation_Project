"""Generate Gazebo SDF models of the aircraft configurations from their YAML files.

Mapping (single source of truth = ``config/aircraft/<name>.yaml``):

* ``<inertial>``: mass and full inertia tensor from :func:`graduation_project.config.load_aircraft`
  (lumped-component derivation). The link origin is the CoM, so the inertial pose is identity.
* Rotor frames ``rotor_<i>``: hub positions ``AircraftParams.rotor.positions`` (body FLU, about
  the CoM). Spin direction ``spin_dirs[i]`` is encoded in the visual names (``ccw``/``cw``) and
  colours. Rotor disk diameter is ``rotor_diameter``.
* Central-body and battery boxes: the positioned, sized components of ``mass_properties``.
* Everything else (arms, motors, landing gear, placeholders, published envelope) comes from
  ``config/visual/<name>.yaml`` and only affects appearance/contact in Gazebo.

The model has a single link. Rotors are SDF ``<frame>`` elements, not jointed links: nothing in
Gazebo actuates them (replay only), and splitting propeller inertia off the base link would need
parameters the configuration does not provide.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from graduation_project.config import CONFIG_DIR, AircraftParams, aircraft_from_dict, load_yaml, value
from graduation_project.gazebo import MODELS_DIR, SDF_VERSION, VISUAL_CONFIG_DIR, fmt, fmt_vec, repo_relative

# RGBA colours. Front arms red so heading is visible; CCW/CW rotors in different colours.
COLOURS = {
    "body": (0.15, 0.15, 0.17, 1.0),
    "battery": (0.25, 0.30, 0.45, 1.0),
    "arm_front": (0.80, 0.10, 0.10, 1.0),
    "arm_rear": (0.20, 0.20, 0.22, 1.0),
    "motor": (0.55, 0.55, 0.58, 1.0),
    "rotor_ccw": (0.20, 0.55, 0.95, 0.45),
    "rotor_cw": (0.95, 0.60, 0.15, 0.45),
    "gear": (0.10, 0.10, 0.10, 1.0),
    "extra": (0.40, 0.40, 0.40, 1.0),
    "sensor": (0.08, 0.08, 0.09, 1.0),
    "antenna": (0.85, 0.85, 0.82, 1.0),
    "envelope": (0.10, 0.80, 0.30, 0.12),
}


@dataclass(frozen=True)
class Box:
    """Axis-aligned (body frame) box: ``centre`` in the body frame B (origin = CoM) [m]."""

    name: str
    centre: np.ndarray
    size: np.ndarray
    yaw: float = 0.0
    colour: str = "extra"

    @property
    def lo(self) -> np.ndarray:
        return self.centre - self._half_extent()

    @property
    def hi(self) -> np.ndarray:
        return self.centre + self._half_extent()

    def _half_extent(self) -> np.ndarray:
        c, s = abs(np.cos(self.yaw)), abs(np.sin(self.yaw))
        lx, ly, lz = self.size
        return 0.5 * np.array([c * lx + s * ly, s * lx + c * ly, lz])


@dataclass(frozen=True)
class Cylinder:
    """Cylinder with its axis along +z_B."""

    name: str
    centre: np.ndarray
    radius: float
    length: float
    colour: str

    @property
    def lo(self) -> np.ndarray:
        return self.centre - np.array([self.radius, self.radius, 0.5 * self.length])

    @property
    def hi(self) -> np.ndarray:
        return self.centre + np.array([self.radius, self.radius, 0.5 * self.length])


@dataclass(frozen=True)
class ModelGeometry:
    aircraft: AircraftParams
    com_geometric: np.ndarray  # CoM position in frame G [m]
    airframe: list[Box | Cylinder]  # rigid airframe visuals (excludes rotor disks and envelope)
    rotor_disks: list[Cylinder]
    collisions: list[Box]
    envelope: Box | None
    sources: dict[str, str]  # file -> sha256
    flight: dict | None = None  # Gazebo-physics flight-model parameters (visual config), if any

    def airframe_extent(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.min([p.lo for p in self.airframe], axis=0)
        hi = np.max([p.hi for p in self.airframe], axis=0)
        return lo, hi

    @property
    def gear_bottom_z(self) -> float:
        """Lowest point of the landing gear in the body frame (negative) [m]."""
        return float(min(c.lo[2] for c in self.collisions))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _v(node: Any) -> np.ndarray:
    return np.asarray(value(node), dtype=float)


def build_geometry(name: str, aircraft_path: Path | None = None, visual_path: Path | None = None) -> ModelGeometry:
    aircraft_path = aircraft_path or CONFIG_DIR / "aircraft" / f"{name}.yaml"
    visual_path = visual_path or VISUAL_CONFIG_DIR / f"{name}.yaml"
    raw = load_yaml(aircraft_path)
    if raw.get("schema_version") == 2:  # traceable spec database -> the same v1 view the dynamics use
        from graduation_project.aircraft_spec import load_spec, spec_to_v1

        raw = spec_to_v1(load_spec(aircraft_path))
    ac = aircraft_from_dict(raw)
    vis = load_yaml(visual_path)
    if vis.get("aircraft") != ac.name:
        raise ValueError(f"{visual_path} is for {vis.get('aircraft')!r}, not {ac.name!r}")

    hubs = ac.rotor.positions  # body frame (about the CoM)
    # Every geometric-frame position p_G maps to p_B = p_G - r_CoM. Recover r_CoM from the hub
    # geometry rather than recomputing it, so this stays consistent with config.py by construction.
    hub_height_g = float(value(raw["geometry"]["rotor_height"]))
    com = np.array([0.0, 0.0, hub_height_g]) - np.array([hubs[:, 0].mean(), hubs[:, 1].mean(), hubs[0, 2]])
    to_b = lambda p_g: np.asarray(p_g, dtype=float) - com  # noqa: E731

    airframe: list[Box | Cylinder] = []
    for comp in raw["mass_properties"]["components"]:
        if "at_rotors" in comp or "size" not in comp:
            continue
        colour = "battery" if "battery" in comp["name"] else "body"
        airframe.append(Box(comp["name"], to_b(_v(comp["position"])), _v(comp["size"]), colour=colour))

    arm_w, arm_h = float(value(vis["arm"]["width"])), float(value(vis["arm"]["height"]))
    m_r, m_h = float(value(vis["motor"]["radius"])), float(value(vis["motor"]["height"]))
    disk_t = float(value(vis["rotor_disk"]["thickness"]))
    rotor_disks: list[Cylinder] = []
    for i, (hub, s) in enumerate(zip(hubs, ac.rotor.spin_dirs), start=1):
        sense = "ccw" if s > 0 else "cw"
        # Arm from the geometric centre (at rotor height) to the hub, below the motor.
        centre_g = to_b([0.0, 0.0, hub_height_g])
        mid = 0.5 * (centre_g + hub) - np.array([0.0, 0.0, 0.5 * arm_h])
        length = float(np.linalg.norm((hub - centre_g)[:2]))
        yaw = float(np.arctan2(hub[1] - centre_g[1], hub[0] - centre_g[0]))
        airframe.append(Box(f"arm_{i}", mid, np.array([length, arm_w, arm_h]), yaw,
                            "arm_front" if hub[0] > centre_g[0] else "arm_rear"))
        airframe.append(Cylinder(f"motor_{i}", hub - np.array([0.0, 0.0, 0.5 * m_h]), m_r, m_h, "motor"))
        rotor_disks.append(Cylinder(f"rotor_{i}_{sense}_disk", hub + np.array([0.0, 0.0, 0.5 * disk_t]),
                                    0.5 * ac.rotor_diameter, disk_t, f"rotor_{sense}"))

    lg = vis["landing_gear"]
    bottom, track = float(value(lg["bottom_z"])), float(value(lg["track"]))
    skid_l, sec = float(value(lg["skid_length"])), float(value(lg["skid_section"]))
    body = next(b for b in airframe if isinstance(b, Box) and b.colour == "body")
    body_bottom_g = body.lo[2] + com[2]
    strut_h = body_bottom_g - (bottom + sec)
    if strut_h <= 0:
        raise ValueError("landing_gear.bottom_z must lie below the central body")
    for side, y in (("left", 0.5 * track), ("right", -0.5 * track)):
        airframe.append(Box(f"skid_{side}", to_b([0.0, y, bottom + 0.5 * sec]), np.array([skid_l, sec, sec]),
                            colour="gear"))
        for end, x in (("front", 0.3 * skid_l), ("rear", -0.3 * skid_l)):
            airframe.append(Box(f"strut_{end}_{side}", to_b([x, y, bottom + sec + 0.5 * strut_h]),
                                np.array([sec, sec, strut_h]), colour="gear"))

    for extra in vis.get("extra_boxes") or []:
        airframe.append(Box(extra["name"], to_b(_v(extra["position"])), _v(extra["size"]), colour="extra"))
    for cyl in vis.get("extra_cylinders") or []:
        airframe.append(Cylinder(cyl["name"], to_b(_v(cyl["position"])), float(value(cyl["radius"])),
                                 float(value(cyl["length"])), cyl.get("colour", "extra")))

    collisions = [
        Box("body_collision", body.centre, body.size),
        Box("landing_gear_collision", to_b([0.0, 0.0, bottom + 0.5 * sec]), np.array([skid_l, track + sec, sec])),
    ]

    envelope = None
    env = vis.get("published_envelope")
    if env:
        size = _v(env["size"])
        cxy = _v(env["centre_xy"])
        envelope = Box("published_envelope", to_b([cxy[0], cxy[1], bottom + 0.5 * size[2]]), size, colour="envelope")

    return ModelGeometry(ac, com, airframe, rotor_disks, collisions, envelope,
                         {repo_relative(aircraft_path): _sha256(aircraft_path), repo_relative(visual_path): _sha256(visual_path)},
                         vis.get("flight_model"))


def _material(colour: str) -> str:
    r, g, b, a = COLOURS[colour]
    rgba = fmt_vec((r, g, b, a))
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.1 0.1 0.1 1</specular></material>"


def _shape_xml(p: Box | Cylinder) -> str:
    if isinstance(p, Box):
        return f"<box><size>{fmt_vec(p.size)}</size></box>"
    return f"<cylinder><radius>{fmt(p.radius)}</radius><length>{fmt(p.length)}</length></cylinder>"


def _pose(p: Box | Cylinder) -> str:
    yaw = p.yaw if isinstance(p, Box) else 0.0
    return f"<pose>{fmt_vec(p.centre)} 0 0 {fmt(yaw)}</pose>"


def _visual(p: Box | Cylinder, cast_shadows: bool = True) -> str:
    extra = "" if cast_shadows else "<cast_shadows>false</cast_shadows>"
    transparency = 1.0 - COLOURS[p.colour][3]
    tr = f"<transparency>{fmt(transparency)}</transparency>" if transparency > 0 else ""
    return (f'      <visual name="{p.name}">{_pose(p)}<geometry>{_shape_xml(p)}</geometry>'
            f"{_material(p.colour)}{tr}{extra}</visual>")


def model_sdf(geo: ModelGeometry, static: bool = False, include_envelope: bool = True) -> str:
    ac = geo.aircraft
    J = ac.mass_properties.inertia
    lo, hi = geo.airframe_extent()
    lines = [
        '<?xml version="1.0" ?>',
        "<!--",
        "  GENERATED FILE - do not edit. Regenerate with:",
        "    python3 simulation/models/generate_models.py",
        "  Sources (sha256):",
        *[f"    {k}  {v}" for k, v in sorted(geo.sources.items())],
        f"  Aircraft: {ac.name} (model_class: {ac.model_class})",
    ]
    if ac.model_class == "aircraft_reference_geometry":
        lines += [
            "  M400-INSPIRED QUADROTOR. NOT a DJI Matrice 400 digital twin. The real Matrice 400 is",
            "  modelled here with quadrotor assumptions; propulsion, inertia and drag are assumed or",
            "  derived (docs/aircraft_reference.md). Shapes other than the published envelope are placeholders.",
        ]
    else:
        lines += ["  Generic research quadrotor; all parameters are assumptions (not a commercial aircraft)."]
    lines += [
        "  Frame: link origin = centre of mass, axes FLU (x forward, y left, z up).",
        f"  Centre of mass in the geometric frame G: [{fmt_vec(geo.com_geometric)}] m",
        "  Inertial values are derived from assumed lumped components, not measured.",
        f"  Airframe extent (excl. rotor disks): x [{fmt(lo[0])}, {fmt(hi[0])}] y [{fmt(lo[1])}, {fmt(hi[1])}]"
        f" z [{fmt(lo[2])}, {fmt(hi[2])}] m",
        "  Rotors: SDF frames rotor_1..rotor_N (numbered CCW from front-right); visual names carry the",
        "  spin direction seen from above (ccw = +1, cw = -1). No rotor joints: Gazebo does not actuate them.",
        "-->",
        f'<sdf version="{SDF_VERSION}">',
        f'  <model name="{ac.name}">',
        f"    <static>{'true' if static else 'false'}</static>",
        '    <link name="base_link">',
        "      <inertial>",
        f"        <mass>{fmt(ac.mass)}</mass>",
        f"        <inertia><ixx>{fmt(J[0, 0])}</ixx><ixy>{fmt(J[0, 1])}</ixy><ixz>{fmt(J[0, 2])}</ixz>"
        f"<iyy>{fmt(J[1, 1])}</iyy><iyz>{fmt(J[1, 2])}</iyz><izz>{fmt(J[2, 2])}</izz></inertia>",
        "      </inertial>",
    ]
    for c in geo.collisions:
        lines.append(f'      <collision name="{c.name}">{_pose(c)}<geometry>{_shape_xml(c)}</geometry></collision>')
    lines += [_visual(p) for p in geo.airframe]
    lines += [_visual(p, cast_shadows=False) for p in geo.rotor_disks]
    if include_envelope and geo.envelope is not None:
        lines.append("      <!-- Published unfolded envelope (manufacturer), translucent reference box; visual only. -->")
        lines.append(_visual(geo.envelope, cast_shadows=False))
    lines.append("    </link>")
    for i, (hub, s) in enumerate(zip(ac.rotor.positions, ac.rotor.spin_dirs), start=1):
        lines.append(f'    <!-- rotor {i}: spin {"+1 (CCW)" if s > 0 else "-1 (CW)"} seen from above -->')
        lines.append(f'    <frame name="rotor_{i}" attached_to="base_link">'
                     f'<pose relative_to="base_link">{fmt_vec(hub)} 0 0 0</pose></frame>')
    lines.append('    <frame name="geometric_centre" attached_to="base_link">'
                 f'<pose relative_to="base_link">{fmt_vec(-geo.com_geometric)} 0 0 0</pose></frame>')
    lines += ["  </model>", "</sdf>", ""]
    return "\n".join(lines)


def model_config(geo: ModelGeometry) -> str:
    ac = geo.aircraft
    desc = ("M400-inspired reference quadrotor (NOT a DJI Matrice 400 digital twin); generated from "
            if ac.model_class == "aircraft_reference_geometry" else "Generic research quadrotor; generated from ")
    desc += f"config/aircraft/{ac.name}.yaml and config/visual/{ac.name}.yaml."
    return "\n".join([
        '<?xml version="1.0" ?>',
        "<!-- GENERATED FILE - do not edit. Regenerate with: python3 simulation/models/generate_models.py -->",
        "<model>",
        f"  <name>{ac.name}</name>",
        "  <version>1.0</version>",
        f'  <sdf version="{SDF_VERSION}">model.sdf</sdf>',
        "  <author><name>Graduation project (generated)</name></author>",
        f"  <description>{desc}</description>",
        "</model>",
        "",
    ])


def write_model(name: str, out_dir: Path = MODELS_DIR) -> list[Path]:
    geo = build_geometry(name)
    d = Path(out_dir) / name
    d.mkdir(parents=True, exist_ok=True)
    paths = [d / "model.sdf", d / "model.config"]
    paths[0].write_text(model_sdf(geo), encoding="utf-8")
    paths[1].write_text(model_config(geo), encoding="utf-8")
    return paths


FLIGHT_SUFFIX = "_flight"
FLIGHT_NAMESPACE = "m400"


def flight_model_sdf(geo: ModelGeometry, namespace: str = FLIGHT_NAMESPACE) -> str:
    """Gazebo-PHYSICS variant: base link + jointed rotor links + gz MulticopterMotorModel per rotor and
    gz MulticopterVelocityControl (twist commands on ``/<namespace>/gazebo/command/twist``, enable on
    ``/<namespace>/enable``).

    This is a SECONDARY cross-check of the geometry/propulsion parameters in Gazebo's own physics with
    Gazebo's own controller. It is not the research simulator (no estimator, no airframe drag, no battery)
    and its flights are not evidence about the research framework (docs/SIMULATION_MODEL.md).
    """
    if geo.flight is None:
        raise ValueError(f"no flight_model section in the visual config of {geo.aircraft.name}")
    ac = geo.aircraft
    fp = {k: value(v) for k, v in geo.flight.items()}
    m_r = float(fp["rotor_link_mass"])
    n = ac.rotor.n_rotors
    hubs = ac.rotor.positions
    J = ac.mass_properties.inertia.copy()
    for h in hubs:  # remove the rotor point masses (parallel axis) so the total model matches the spec
        J -= m_r * (float(h @ h) * np.eye(3) - np.outer(h, h))
    r = 0.5 * ac.rotor_diameter
    j_disk = (m_r * r * r / 4.0, m_r * r * r / 4.0, m_r * r * r / 2.0)
    km_over_kf = ac.rotor.k_m / ac.rotor.k_f
    name = ac.name + FLIGHT_SUFFIX
    lines = [
        '<?xml version="1.0" ?>',
        "<!--",
        "  GENERATED FILE - do not edit. Regenerate with:",
        "    python3 simulation/models/generate_models.py",
        "  Sources (sha256):",
        *[f"    {k}  {v}" for k, v in sorted(geo.sources.items())],
        f"  {name}: GAZEBO-PHYSICS FLIGHT MODEL of the M400-inspired quadrotor (NOT a DJI Matrice 400 digital twin).",
        "  Propulsion: gz MulticopterMotorModel with the spec-derived k_f, k_m/k_f, tau and omega_max;",
        "  control: gz MulticopterVelocityControl (Gazebo's controller, NOT the research controllers).",
        "  No airframe drag, no estimator, no battery: a secondary cross-check only.",
        f"  Commands: /{namespace}/enable (gz.msgs.Boolean), /{namespace}/gazebo/command/twist (gz.msgs.Twist).",
        "-->",
        f'<sdf version="{SDF_VERSION}">',
        f'  <model name="{name}">',
        "    <static>false</static>",
        '    <link name="base_link">',
        "      <inertial>",
        f"        <mass>{fmt(ac.mass - n * m_r)}</mass>",
        f"        <inertia><ixx>{fmt(J[0, 0])}</ixx><ixy>{fmt(J[0, 1])}</ixy><ixz>{fmt(J[0, 2])}</ixz>"
        f"<iyy>{fmt(J[1, 1])}</iyy><iyz>{fmt(J[1, 2])}</iyz><izz>{fmt(J[2, 2])}</izz></inertia>",
        "      </inertial>",
    ]
    for c in geo.collisions:
        lines.append(f'      <collision name="{c.name}">{_pose(c)}<geometry>{_shape_xml(c)}</geometry></collision>')
    lines += [_visual(p) for p in geo.airframe]
    lines.append("    </link>")
    rotor_cfg = []
    for i, (hub, s) in enumerate(zip(hubs, ac.rotor.spin_dirs), start=1):
        sense = "ccw" if s > 0 else "cw"
        lines += [
            f'    <link name="rotor_{i}">',
            f"      <pose>{fmt_vec(hub)} 0 0 0</pose>",
            f"      <inertial><mass>{fmt(m_r)}</mass><inertia><ixx>{fmt(j_disk[0])}</ixx><ixy>0</ixy><ixz>0</ixz>"
            f"<iyy>{fmt(j_disk[1])}</iyy><iyz>0</iyz><izz>{fmt(j_disk[2])}</izz></inertia></inertial>",
            f'      <visual name="disk"><pose>0 0 0.004 0 0 0</pose><geometry><cylinder><radius>{fmt(r)}</radius>'
            f"<length>0.006</length></cylinder></geometry>{_material(f'rotor_{sense}')}"
            f"<transparency>{fmt(1.0 - COLOURS[f'rotor_{sense}'][3])}</transparency><cast_shadows>false</cast_shadows></visual>",
            f'      <visual name="blade"><pose>0 0 0.006 0 0 0</pose><geometry><box><size>{fmt(2 * r)} 0.045 0.008</size></box>'
            f"</geometry>{_material('motor')}</visual>",
            "    </link>",
            f'    <joint name="rotor_{i}_joint" type="revolute"><parent>base_link</parent><child>rotor_{i}</child>'
            "<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit>"
            "<dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics></axis></joint>",
        ]
        rotor_cfg.append((i, s))
    for i, s in rotor_cfg:
        lines += [
            '    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">',
            f"      <robotNamespace>{namespace}</robotNamespace><jointName>rotor_{i}_joint</jointName><linkName>rotor_{i}</linkName>",
            f"      <turningDirection>{'ccw' if s > 0 else 'cw'}</turningDirection>",
            f"      <timeConstantUp>{fmt(ac.rotor.tau_motor)}</timeConstantUp><timeConstantDown>{fmt(ac.rotor.tau_motor)}</timeConstantDown>",
            f"      <maxRotVelocity>{fmt(ac.rotor.omega_max)}</maxRotVelocity><motorConstant>{fmt(ac.rotor.k_f)}</motorConstant>",
            f"      <momentConstant>{fmt(km_over_kf)}</momentConstant><commandSubTopic>gazebo/command/motor_speed</commandSubTopic>",
            f"      <actuator_number>{i - 1}</actuator_number><rotorDragCoefficient>0</rotorDragCoefficient>",
            f"      <rollingMomentCoefficient>0</rollingMomentCoefficient><motorSpeedPubTopic>motor_speed/{i - 1}</motorSpeedPubTopic>",
            f"      <rotorVelocitySlowdownSim>{fmt(fp['rotor_velocity_slowdown'])}</rotorVelocitySlowdownSim><motorType>velocity</motorType>",
            "    </plugin>",
        ]
    lines += [
        '    <plugin filename="gz-sim-multicopter-control-system" name="gz::sim::systems::MulticopterVelocityControl">',
        f"      <robotNamespace>{namespace}</robotNamespace><commandSubTopic>gazebo/command/twist</commandSubTopic>",
        "      <enableSubTopic>enable</enableSubTopic><comLinkName>base_link</comLinkName>",
        f"      <velocityGain>{fmt_vec(fp['velocity_gain'])}</velocityGain><attitudeGain>{fmt_vec(fp['attitude_gain'])}</attitudeGain>",
        f"      <angularRateGain>{fmt_vec(fp['angular_rate_gain'])}</angularRateGain>",
        f"      <maximumLinearAcceleration>{fmt_vec(fp['maximum_linear_acceleration'])}</maximumLinearAcceleration>",
        f"      <maximumLinearVelocity>{fmt_vec(fp['maximum_linear_velocity'])}</maximumLinearVelocity>",
        f"      <maximumAngularVelocity>{fmt_vec(fp['maximum_angular_velocity'])}</maximumAngularVelocity>",
        "      <linearVelocityNoiseMean>0 0 0</linearVelocityNoiseMean><linearVelocityNoiseStdDev>0 0 0</linearVelocityNoiseStdDev>",
        "      <angularVelocityNoiseMean>0 0 0</angularVelocityNoiseMean><angularVelocityNoiseStdDev>0 0 0</angularVelocityNoiseStdDev>",
        "      <rotorConfiguration>",
    ]
    for i, s in rotor_cfg:
        lines.append(f"        <rotor><jointName>rotor_{i}_joint</jointName><forceConstant>{fmt(ac.rotor.k_f)}</forceConstant>"
                     f"<momentConstant>{fmt(km_over_kf)}</momentConstant><direction>{1 if s > 0 else -1}</direction></rotor>")
    lines += ["      </rotorConfiguration>", "    </plugin>", "  </model>", "</sdf>", ""]
    return "\n".join(lines)


def write_flight_model(name: str, out_dir: Path = MODELS_DIR) -> list[Path]:
    geo = build_geometry(name)
    d = Path(out_dir) / (name + FLIGHT_SUFFIX)
    d.mkdir(parents=True, exist_ok=True)
    cfg = model_config(geo).replace(f"<name>{name}</name>", f"<name>{name + FLIGHT_SUFFIX}</name>").replace(
        "generated from", "Gazebo-physics flight variant (gz MulticopterMotorModel + MulticopterVelocityControl); generated from")
    paths = [d / "model.sdf", d / "model.config"]
    paths[0].write_text(flight_model_sdf(geo), encoding="utf-8")
    paths[1].write_text(cfg, encoding="utf-8")
    return paths
