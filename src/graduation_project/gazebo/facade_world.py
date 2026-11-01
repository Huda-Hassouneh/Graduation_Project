"""Parameterised facade-inspection worlds for Gazebo Harmonic.

The scenario facade is taken from :mod:`graduation_project.experiments.scenarios`: a vertical
plane through ``FACADE_ORIGIN`` with outward normal ``FACADE_NORMAL`` (wall face toward -y at
y = 8 m), spanning ``[0, FACADE_WIDTH_M]`` along ``FACADE_TANGENT``. All building geometry is
built in a facade frame F and transformed to the world (ENU):

* F origin = ``facade_origin`` (on the wall face, at ground level)
* x_F = ``facade_tangent`` (along the wall), y_F = ``-facade_normal`` (into the wall), z_F = up

so the scenario wall face is exactly ``y_F = 0, x_F in [0, width]``. Variants:

* ``flat``: one wall block.
* ``complex``: the same main block (identical scenario face) plus a recessed neighbour block
  (convex + concave corners at x_F = 0), a protruding wing beyond x_F = width (concave corner
  at x_F = width) and balconies with railings on the main face.

Keep-out regions are translucent red, visual-only boxes (no collision): every solid inflated by
``keepout_margin`` toward the flight side. They are markers for later planners; nothing
enforces them in the Python simulator. ``<world>.geometry.json`` lists every solid and keep-out
box in world coordinates for tests and later milestones.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from graduation_project.experiments.scenarios import FACADE_NORMAL, FACADE_ORIGIN, FACADE_TANGENT, FACADE_WIDTH_M
from graduation_project.gazebo import SDF_VERSION, WORLDS_DIR, fmt, fmt_vec
from graduation_project.gazebo.sdf_model import build_geometry

CONCRETE = (0.75, 0.73, 0.70, 1.0)
FACADE_FACE = (0.82, 0.80, 0.76, 1.0)
BALCONY = (0.60, 0.60, 0.62, 1.0)
RAILING = (0.30, 0.32, 0.35, 0.6)
KEEPOUT = (0.95, 0.10, 0.10, 0.18)
PAD = (0.20, 0.20, 0.22, 1.0)
PAD_MARK = (0.95, 0.85, 0.10, 1.0)
GROUND = (0.45, 0.50, 0.42, 1.0)


@dataclass(frozen=True)
class FacadeWorldSpec:
    name: str
    variant: str = "flat"  # "flat" | "complex"
    facade_origin: tuple[float, float, float] = FACADE_ORIGIN
    facade_tangent: tuple[float, float, float] = FACADE_TANGENT
    facade_normal: tuple[float, float, float] = FACADE_NORMAL
    facade_width: float = FACADE_WIDTH_M
    facade_height: float = 12.0  # assumed; above the 9 m scan ceiling
    building_depth: float = 10.0  # assumed
    keepout_margin: float = 1.0  # assumed safety buffer [m]
    # complex variant (all assumed, chosen to keep the baseline rasters outside every keep-out box)
    setback_width: float = 6.0
    setback_depth: float = 2.0  # recess of the neighbour face behind the scenario plane
    setback_height: float = 9.0
    wing_width: float = 4.0
    wing_projection: float = 1.5  # wing face protrudes this far in front of the scenario plane
    wing_height: float = 15.0
    balcony_levels: tuple[float, ...] = (4.0, 7.0)  # slab bottom heights [m]
    balcony_spans: tuple[tuple[float, float], ...] = ((1.5, 4.5), (7.5, 10.5))  # x_F ranges [m]
    balcony_depth: float = 1.2
    balcony_slab: float = 0.2
    railing_height: float = 1.0
    railing_thickness: float = 0.05
    # take-off pad at the world origin (docs/coordinate_frames.md: W origin = take-off point)
    pad_size: float = 2.0
    pad_thickness: float = 0.02
    ground_size: float = 120.0
    aircraft: str | None = "research_quad"
    aircraft_static: bool = True  # static = kinematic replay only, no Gazebo dynamics on the vehicle


@dataclass(frozen=True)
class WorldBox:
    """Box in the facade frame F: ``lo``/``hi`` corners [m]."""

    name: str
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]
    kind: str  # "solid" | "railing" | "keepout"
    colour: tuple[float, float, float, float] = CONCRETE


@dataclass
class FacadeLayout:
    spec: FacadeWorldSpec
    boxes: list[WorldBox] = field(default_factory=list)

    @property
    def rotation(self) -> np.ndarray:
        return facade_frame(self.spec)[1]

    def to_world(self, p_f) -> np.ndarray:
        o, R = facade_frame(self.spec)
        return o + R @ np.asarray(p_f, dtype=float)

    def world_box(self, b: WorldBox) -> dict:
        lo, hi = np.asarray(b.lo), np.asarray(b.hi)
        o, R = facade_frame(self.spec)
        return {"name": b.name, "kind": b.kind, "centre": (o + R @ (0.5 * (lo + hi))).tolist(),
                "size": (hi - lo).tolist(), "yaw": float(np.arctan2(R[1, 0], R[0, 0]))}


def facade_frame(spec: FacadeWorldSpec) -> tuple[np.ndarray, np.ndarray]:
    t = np.asarray(spec.facade_tangent, dtype=float)
    n = np.asarray(spec.facade_normal, dtype=float)
    t, n = t / np.linalg.norm(t), n / np.linalg.norm(n)
    if abs(t[2]) > 1e-9 or abs(n[2]) > 1e-9 or abs(t @ n) > 1e-9:
        raise ValueError("facade tangent and normal must be horizontal and orthogonal")
    y_f = -n
    if np.cross(t, y_f)[2] < 0.0:
        raise ValueError("facade frame must be right-handed: cross(tangent, -normal) must point up")
    return np.asarray(spec.facade_origin, dtype=float), np.column_stack([t, y_f, [0.0, 0.0, 1.0]])


def build_layout(spec: FacadeWorldSpec) -> FacadeLayout:
    if spec.variant not in ("flat", "complex"):
        raise ValueError(f"unknown facade variant {spec.variant!r}")
    facade_frame(spec)  # validate orientation before building anything
    W, H, D, m = spec.facade_width, spec.facade_height, spec.building_depth, spec.keepout_margin
    lay = FacadeLayout(spec)
    add = lay.boxes.append

    def solid(name, lo, hi, colour=CONCRETE, inflate=True):
        add(WorldBox(name, tuple(lo), tuple(hi), "solid", colour))
        if inflate and m > 0:
            add(WorldBox(f"keepout_{name}", (lo[0] - m, lo[1] - m, max(0.0, lo[2] - m)), (hi[0] + m, lo[1], hi[2] + m),
                         "keepout", KEEPOUT))

    # Main block: its -y_F face is exactly the scenario facade (y_F = 0, x_F in [0, W]).
    solid("main_block", (0.0, 0.0, 0.0), (W, D, H), FACADE_FACE)
    if spec.variant == "complex":
        sw, sd = spec.setback_width, spec.setback_depth
        solid("setback_block", (-sw, sd, 0.0), (0.0, D, spec.setback_height))
        ww, wp = spec.wing_width, spec.wing_projection
        solid("wing_block", (W, -wp, 0.0), (W + ww, D, spec.wing_height))
        for li, z in enumerate(spec.balcony_levels):
            for si, (x0, x1) in enumerate(spec.balcony_spans):
                if not 0.0 <= x0 < x1 <= W:
                    raise ValueError("balcony spans must lie on the main facade")
                tag = f"balcony_L{li + 1}_S{si + 1}"
                bd, slab, rh, rt = spec.balcony_depth, spec.balcony_slab, spec.railing_height, spec.railing_thickness
                solid(tag, (x0, -bd, z), (x1, 0.0, z + slab), BALCONY, inflate=False)
                add(WorldBox(f"{tag}_railing", (x0, -bd, z + slab), (x1, -bd + rt, z + slab + rh), "railing", RAILING))
                add(WorldBox(f"keepout_{tag}", (x0 - m, -bd - m, z - m), (x1 + m, 0.0, z + slab + rh + m),
                             "keepout", KEEPOUT))
    return lay


def _material(rgba) -> str:
    s = fmt_vec(rgba)
    return f"<material><ambient>{s}</ambient><diffuse>{s}</diffuse><specular>0.05 0.05 0.05 1</specular></material>"


def _box_link(lay: FacadeLayout, b: WorldBox, collide: bool) -> str:
    wb = lay.world_box(b)
    pose = f"<pose>{fmt_vec(wb['centre'])} 0 0 {fmt(wb['yaw'])}</pose>"
    geom = f"<geometry><box><size>{fmt_vec(wb['size'])}</size></box></geometry>"
    tr = 1.0 - b.colour[3]
    extra = (f"<transparency>{fmt(tr)}</transparency><cast_shadows>false</cast_shadows>" if tr > 0 else "")
    col = f'<collision name="collision">{geom}</collision>' if collide else ""
    return (f'      <link name="{b.name}">{pose}{col}<visual name="visual">{geom}{_material(b.colour)}{extra}</visual>'
            "</link>")


def world_sdf(spec: FacadeWorldSpec) -> str:
    lay = build_layout(spec)
    o, R = facade_frame(spec)
    end = o + R @ np.array([spec.facade_width, 0.0, 0.0])
    lines = [
        '<?xml version="1.0" ?>',
        "<!--",
        "  GENERATED FILE - do not edit. Regenerate with:",
        "    python3 simulation/worlds/generate_facade_world.py",
        f"  World '{spec.name}', variant '{spec.variant}'. Frame: ENU (x east, y north, z up), origin = take-off point.",
        f"  Scenario facade face (graduation_project.experiments.scenarios): plane through [{fmt_vec(o)}]",
        f"  with outward normal [{fmt_vec(spec.facade_normal)}], from [{fmt_vec(o)}] to [{fmt_vec(end)}].",
        "  Building dimensions beyond that face, balconies, wings and keep-out margins are assumptions.",
        "  Keep-out boxes are visual markers only (no collision, not enforced by any controller).",
    ]
    if spec.aircraft and spec.aircraft_static:
        lines.append("  The aircraft is STATIC: Gazebo does not simulate its flight. It is moved only by kinematic")
        lines.append("  replay of Python-simulator logs (simulation/gazebo/replay_run.py).")
    lines += [
        "-->",
        f'<sdf version="{SDF_VERSION}">',
        f'  <world name="{spec.name}">',
        '    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>',
        '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
        '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
        '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        "    <gravity>0 0 -9.80665</gravity>",
        "    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.70 0.80 0.95 1</background><shadows>true</shadows></scene>",
        '    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 30 0 0 0</pose>'
        "<diffuse>0.9 0.9 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular>"
        "<direction>-0.4 0.3 -0.85</direction></light>",
        '    <model name="ground_plane"><static>true</static><link name="link">'
        '<collision name="collision"><geometry><plane><normal>0 0 1</normal>'
        f"<size>{fmt(spec.ground_size)} {fmt(spec.ground_size)}</size></plane></geometry></collision>"
        '<visual name="visual"><geometry><plane><normal>0 0 1</normal>'
        f"<size>{fmt(spec.ground_size)} {fmt(spec.ground_size)}</size></plane></geometry>{_material(GROUND)}</visual>"
        "</link></model>",
    ]
    ps, pt = spec.pad_size, spec.pad_thickness
    mark = f"<geometry><box><size>{fmt(0.6 * ps)} {fmt(0.08 * ps)} {fmt(0.002)}</size></box></geometry>"
    lines.append(
        '    <model name="takeoff_pad"><static>true</static>'
        f'<pose>0 0 {fmt(0.5 * pt)} 0 0 0</pose><link name="link">'
        f'<collision name="collision"><geometry><box><size>{fmt(ps)} {fmt(ps)} {fmt(pt)}</size></box></geometry></collision>'
        f'<visual name="pad"><geometry><box><size>{fmt(ps)} {fmt(ps)} {fmt(pt)}</size></box></geometry>{_material(PAD)}</visual>'
        f'<visual name="mark_x"><pose>0 0 {fmt(0.5 * pt + 0.001)} 0 0 0</pose>{mark}{_material(PAD_MARK)}</visual>'
        f'<visual name="mark_y"><pose>0 0 {fmt(0.5 * pt + 0.001)} 0 0 1.57079633</pose>{mark}{_material(PAD_MARK)}</visual>'
        "</link></model>"
    )
    lines.append('    <model name="facade_building"><static>true</static>')
    lines += [_box_link(lay, b, collide=True) for b in lay.boxes if b.kind in ("solid", "railing")]
    lines.append("    </model>")
    lines.append('    <model name="keepout_zones"><static>true</static>')
    lines += [_box_link(lay, b, collide=False) for b in lay.boxes if b.kind == "keepout"]
    lines.append("    </model>")
    if spec.aircraft:
        geo = build_geometry(spec.aircraft.removesuffix("_flight"))
        z = pt - geo.gear_bottom_z + 0.001  # gear 1 mm above the pad
        lines.append(f"    <include><uri>model://{spec.aircraft}</uri><name>{spec.aircraft}</name>"
                     f"<pose>0 0 {fmt(z)} 0 0 0</pose><static>{'true' if spec.aircraft_static else 'false'}</static></include>")
    lines += ["  </world>", "</sdf>", ""]
    return "\n".join(lines)


def geometry_json(spec: FacadeWorldSpec) -> str:
    lay = build_layout(spec)
    data = {
        "generated_by": "simulation/worlds/generate_facade_world.py",
        "frame": "world ENU; box centre/size in metres; yaw about +z [rad]",
        "spec": asdict(spec),
        "boxes": [lay.world_box(b) for b in lay.boxes],
    }
    return json.dumps(data, indent=2, sort_keys=True) + "\n"


DEFAULT_WORLDS = (
    FacadeWorldSpec(name="facade_flat", variant="flat"),
    FacadeWorldSpec(name="facade_complex", variant="complex"),
)
# Company-platform worlds: static M400 model for kinematic replay of Python missions, and the
# Gazebo-physics flight model for the secondary flight test (simulation/gazebo/flight_test.py).
M400_WORLDS = (
    FacadeWorldSpec(name="facade_complex_m400", variant="complex", aircraft="dji_matrice_400"),
    FacadeWorldSpec(name="m400_flight_test", variant="flat", aircraft="dji_matrice_400_flight", aircraft_static=False),
)


def write_world(spec: FacadeWorldSpec, out_dir: Path = WORLDS_DIR) -> list[Path]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    sdf_path, json_path = out_dir / f"{spec.name}.sdf", out_dir / f"{spec.name}.geometry.json"
    sdf_path.write_text(world_sdf(spec), encoding="utf-8")
    json_path.write_text(geometry_json(spec), encoding="utf-8")
    return [sdf_path, json_path]
