"""Run persistence: ``<run_dir>/timeseries.csv`` + ``<run_dir>/metadata.json``.

The CSV has one header row of column names (units: docs/experiment_protocol.md, "Log format").
Metadata records the experiment ID, git commit (and whether the tree was dirty), software
versions, full configuration snapshots, seed, timing, injected changes, metrics and status.
"""

from __future__ import annotations

import datetime as dt
import json
import platform
import subprocess
import uuid
from pathlib import Path
from typing import Any

import numpy as np

from graduation_project import __version__
from graduation_project.config import REPO_ROOT
from graduation_project.experiments.runner import RunResult

RUNS_DIR = REPO_ROOT / "results" / "runs"


def git_info() -> dict[str, Any]:
    def git(*args: str) -> str:
        return subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True).stdout.strip()

    try:
        return {"commit": git("rev-parse", "HEAD"), "dirty": bool(git("status", "--porcelain"))}
    except (subprocess.CalledProcessError, FileNotFoundError):
        return {"commit": None, "dirty": None}


def software_versions() -> dict[str, str]:
    import matplotlib
    import scipy
    import yaml

    return {
        "graduation_project": __version__,
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "matplotlib": matplotlib.__version__,
        "pyyaml": yaml.__version__,
    }


def new_experiment_id(scenario: str, aircraft: str, seed: int) -> str:
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{stamp}_{scenario}_{aircraft}_s{seed}_{uuid.uuid4().hex[:6]}"


def _jsonable(x: Any) -> Any:
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    return x


def save_run(run: RunResult, metadata: dict[str, Any], runs_dir: Path = RUNS_DIR) -> Path:
    exp_id = metadata["experiment_id"]
    run_dir = runs_dir / exp_id
    run_dir.mkdir(parents=True, exist_ok=False)
    np.savetxt(run_dir / "timeseries.csv", run.data, delimiter=",", header=",".join(run.columns), comments="", fmt="%.9g")
    meta = {
        **metadata,
        "status": run.status,
        "failure_message": run.message,
        "warnings": run.warnings,
        "git": git_info(),
        "software": software_versions(),
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    (run_dir / "metadata.json").write_text(json.dumps(_jsonable(meta), indent=2))
    return run_dir


def load_run_csv(path: Path) -> tuple[list[str], np.ndarray]:
    with open(path, encoding="utf-8") as f:
        columns = f.readline().strip().split(",")
    return columns, np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
