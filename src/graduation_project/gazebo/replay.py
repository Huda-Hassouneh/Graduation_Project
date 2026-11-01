"""Kinematic replay of Python-simulator runs in Gazebo (visualisation only).

A run directory written by :func:`graduation_project.experiments.io.save_run` is read with
:func:`load_run_csv`. The logged truth pose (``p_*`` world ENU, ``q_*`` body-to-world scalar-first)
is resampled to a display rate and applied to a *static* Gazebo model through the
``/world/<world>/set_pose_vector`` service. Gazebo integrates no dynamics for the vehicle, so a
replay shows what the Python simulator computed; it does not validate flight dynamics.

Frames: both sides are ENU world / FLU body with the model origin at the CoM, so positions and
quaternions pass through unchanged. Only the component order differs: the log stores
``[w, x, y, z]``; ``gz.msgs.Quaternion`` has named fields.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from graduation_project.experiments.io import load_run_csv

REFERENCE_MARKER = "reference_marker"
REFERENCE_MARKER_SDF = f"""<?xml version="1.0" ?>
<sdf version="1.10"><model name="{REFERENCE_MARKER}"><static>true</static><link name="link">
<visual name="visual"><cast_shadows>false</cast_shadows><geometry><sphere><radius>0.08</radius></sphere></geometry>
<material><ambient>0.1 0.9 0.2 0.7</ambient><diffuse>0.1 0.9 0.2 0.7</diffuse></material>
<transparency>0.3</transparency></visual></link></model></sdf>"""


@dataclass(frozen=True)
class ReplayData:
    run_dir: Path
    aircraft: str
    scenario: str
    feedback_mode: str
    t: np.ndarray  # (N,) [s]
    position: np.ndarray  # (N, 3) world ENU [m]
    quat_wxyz: np.ndarray  # (N, 4) body-to-world
    ref_position: np.ndarray | None  # (N, 3) or None if not logged


def load_replay(run_dir: str | Path) -> ReplayData:
    run_dir = Path(run_dir)
    meta: dict[str, Any] = json.loads((run_dir / "metadata.json").read_text())
    cols, data = load_run_csv(run_dir / "timeseries.csv")
    col = lambda name: data[:, cols.index(name)]  # noqa: E731
    pos = np.column_stack([col(f"p_{c}") for c in "xyz"])
    q = np.column_stack([col(f"q_{c}") for c in "wxyz"])
    norms = np.linalg.norm(q, axis=1)
    if not np.allclose(norms, 1.0, atol=1e-6):
        raise ValueError(f"{run_dir}: logged quaternions are not unit length")
    ref = np.column_stack([col(f"ref_p_{c}") for c in "xyz"]) if "ref_p_x" in cols else None
    aircraft = meta["aircraft"]["name"] if isinstance(meta.get("aircraft"), dict) else str(meta.get("aircraft"))
    return ReplayData(run_dir, aircraft, str(meta.get("scenario", "?")), str(meta.get("feedback_mode", "?")),
                      col("t"), pos, q / norms[:, None], ref)


def frame_indices(t: np.ndarray, fps: float) -> np.ndarray:
    """Indices of the log rows nearest (not after) each display instant ``k / fps``."""
    if fps <= 0:
        raise ValueError("fps must be positive")
    stamps = np.arange(t[0], t[-1] + 1e-12, 1.0 / fps)
    idx = np.searchsorted(t, stamps + 1e-12, side="right") - 1
    return np.unique(np.clip(idx, 0, len(t) - 1))


def _import_gz_transport() -> None:
    """Import gz.transport13 before any gz.msgs10 module.

    With the C++ protobuf backend, creating a gz.msgs message before gz.transport13 is imported
    leaves the descriptor pool unable to nest it (``TypeError: No message class registered for
    'gz.msgs.Pose'`` when building a Pose_V). Verified with gz-transport13 13.6 / gz-msgs10.
    """
    import gz.transport13  # noqa: F401


def pose_msg(name: str, p, q_wxyz):
    _import_gz_transport()
    from gz.msgs10.pose_pb2 import Pose

    msg = Pose()
    msg.name = name
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in p)
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = (float(v) for v in q_wxyz)
    return msg


class GazeboReplayer:
    """Thin gz-transport client: query the scene, spawn static entities, set poses."""

    def __init__(self, world: str, timeout_ms: int = 3000) -> None:
        from gz.transport13 import Node

        self.world = world
        self.timeout_ms = timeout_ms
        self.node = Node()

    def model_names(self) -> list[str]:
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene

        ok, scene = self.node.request(f"/world/{self.world}/scene/info", Empty(), Empty, Scene, self.timeout_ms)
        if not ok:
            raise RuntimeError(f"no response from /world/{self.world}/scene/info (is `gz sim` running this world?)")
        return [m.name for m in scene.model]

    def spawn_sdf(self, sdf: str, name: str) -> None:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory

        req = EntityFactory()
        req.sdf = sdf
        req.name = name
        req.allow_renaming = False
        ok, rep = self.node.request(f"/world/{self.world}/create", req, EntityFactory, Boolean, self.timeout_ms)
        if not (ok and rep.data):
            raise RuntimeError(f"failed to spawn {name!r} in world {self.world!r}")

    def ensure_static_aircraft(self, aircraft: str, present: list[str]) -> None:
        if aircraft in present:
            return
        from graduation_project.gazebo.sdf_model import build_geometry, model_sdf

        self.spawn_sdf(model_sdf(build_geometry(aircraft), static=True), aircraft)

    def set_poses(self, poses: list) -> bool:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_v_pb2 import Pose_V

        req = Pose_V()
        for p in poses:
            req.pose.append(p)
        ok, rep = self.node.request(f"/world/{self.world}/set_pose_vector", req, Pose_V, Boolean, self.timeout_ms)
        return bool(ok and rep.data)


def replay(data: ReplayData, replayer: GazeboReplayer, fps: float = 30.0, speed: float = 1.0,
           show_reference: bool = True, wait_for_scene_s: float = 10.0) -> int:
    """Stream the run to Gazebo in (scaled) wall-clock time. Returns the number of frames sent."""
    deadline = time.monotonic() + wait_for_scene_s
    while True:
        try:
            present = replayer.model_names()
            break
        except RuntimeError:
            if time.monotonic() > deadline:
                raise
            time.sleep(0.5)
    replayer.ensure_static_aircraft(data.aircraft, present)
    show_reference = show_reference and data.ref_position is not None
    if show_reference and REFERENCE_MARKER not in present:
        replayer.spawn_sdf(REFERENCE_MARKER_SDF, REFERENCE_MARKER)
    idx = frame_indices(data.t, fps)
    identity = (1.0, 0.0, 0.0, 0.0)
    t0_wall, t0_log = time.monotonic(), data.t[idx[0]]
    for i in idx:
        delay = (data.t[i] - t0_log) / speed - (time.monotonic() - t0_wall)
        if delay > 0:
            time.sleep(delay)
        poses = [pose_msg(data.aircraft, data.position[i], data.quat_wxyz[i])]
        if show_reference:
            poses.append(pose_msg(REFERENCE_MARKER, data.ref_position[i], identity))
        if not replayer.set_poses(poses):
            raise RuntimeError(f"set_pose_vector rejected frame at t = {data.t[i]:.3f} s")
    return len(idx)
