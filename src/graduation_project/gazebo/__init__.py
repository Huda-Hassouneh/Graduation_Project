"""Gazebo Harmonic (Gazebo Sim 8) model/world generation and kinematic trajectory replay (Milestone 3).

Gazebo is a visualisation and scenario environment here. The Python simulator remains the only
dynamics and controller implementation: replay moves *static* models along logged poses and
does not simulate flight (docs/gazebo_milestone3.md).
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path

from graduation_project.config import REPO_ROOT

SIMULATION_DIR = REPO_ROOT / "simulation"
MODELS_DIR = SIMULATION_DIR / "models"
WORLDS_DIR = SIMULATION_DIR / "worlds"
VISUAL_CONFIG_DIR = REPO_ROOT / "config" / "visual"
SDF_VERSION = "1.10"  # supported by SDFormat 14 (Gazebo Harmonic)
# `gz sim` resolves model:// through GZ_SIM_RESOURCE_PATH, `gz sdf` through SDF_PATH (verified with
# Gazebo Sim 8.15 / SDFormat 14). simulation/gazebo/env.sh sets the same two variables for a shell.
GZ_PATH_VARS = ("GZ_SIM_RESOURCE_PATH", "SDF_PATH")


def gazebo_env(base: Mapping[str, str] | None = None) -> dict[str, str]:
    """Copy of ``base`` (default ``os.environ``) with MODELS_DIR prepended to both model search paths."""
    env = dict(os.environ if base is None else base)
    for var in GZ_PATH_VARS:
        entries = [p for p in env.get(var, "").split(os.pathsep) if p and Path(p) != MODELS_DIR]
        env[var] = os.pathsep.join([str(MODELS_DIR), *entries])
    return env


def fmt(x: float) -> str:
    """Deterministic, compact float formatting for generated SDF (no ``-0``)."""
    x = float(x)
    if abs(x) < 1e-12:
        x = 0.0
    return f"{x:.9g}"


def fmt_vec(v) -> str:
    return " ".join(fmt(c) for c in v)


def repo_relative(path: Path) -> str:
    try:
        return Path(path).resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return Path(path).name
