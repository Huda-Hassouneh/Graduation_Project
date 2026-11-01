"""Reproducible experiment campaigns (Phase I): conditions A-D, ablations, seeded repetitions, parallel runs.

Conditions (identical initial conditions, trajectory, disturbance sequence per seed, safety limits):

====  ============================  ===================
id    controller                    research framework
====  ============================  ===================
A     cascaded PID                  disabled (stock: low-battery failsafe only)
B     DOB-assisted PID              disabled
C     cascaded PID                  enabled
D     DOB-assisted PID              enabled
====  ============================  ===================

Ablations (PID + framework, one component changed; reference = C):
``fixed_model`` (no online adaptation), ``periodic`` (periodic instead of event-triggered decisions and
probes), ``passive`` (no active diagnosis), ``no_energy`` (energy awareness off), ``no_replanning``
(path-preserving recovery only).

Outputs under ``results/campaign/<name>/``: ``summary.csv`` (one row per run), ``campaign.json``
(configuration, seeds, software, git), and per run ``runs/<scenario>/<condition>/seed_<n>/`` with
``metrics.json``, ``events.csv`` and (for the first ``keep_timeseries`` seeds) ``timeseries.csv.gz``.
"""

from __future__ import annotations

import csv
import dataclasses
import gzip
import json
import os
import time
from dataclasses import dataclass, replace
from multiprocessing import Pool
from pathlib import Path
from typing import Any

import numpy as np

from graduation_project.config import REPO_ROOT
from graduation_project.experiments.io import git_info, software_versions
from graduation_project.experiments.mission_runner import run_mission
from graduation_project.experiments.mission_scenarios import load_scenario
from graduation_project.framework.core import FrameworkConfig
from graduation_project.metrics.mission import mission_metrics

CAMPAIGN_DIR = REPO_ROOT / "results" / "campaign"
FW_ON = FrameworkConfig()
FW_OFF = FrameworkConfig(enabled=False)
CONDITIONS: dict[str, tuple[str, FrameworkConfig]] = {
    "A": ("pid", FW_OFF), "B": ("dob", FW_OFF), "C": ("pid", FW_ON), "D": ("dob", FW_ON),
}
ABLATIONS: dict[str, tuple[str, FrameworkConfig]] = {
    "fixed_model": ("pid", replace(FW_ON, adaptation=False)),
    "periodic": ("pid", replace(FW_ON, diagnosis_mode="periodic")),
    "passive": ("pid", replace(FW_ON, active=False)),
    "no_energy": ("pid", replace(FW_ON, energy_aware=False)),
    "no_replanning": ("pid", replace(FW_ON, replanning=False)),
}
ALL_CONFIGS = {**CONDITIONS, **ABLATIONS}


@dataclass(frozen=True)
class Job:
    scenario: str
    condition: str
    seed: int
    out_dir: str
    keep_timeseries: bool


def run_job(job: Job) -> dict[str, Any]:
    import psutil

    proc = psutil.Process(os.getpid())
    cpu0 = sum(proc.cpu_times()[:2])
    ctrl, fw = ALL_CONFIGS[job.condition]
    scn = load_scenario(job.scenario, job.seed)
    t0 = time.perf_counter()
    try:
        run = run_mission(scn, ctrl, fw)
    except Exception as exc:  # recorded, never hidden
        return {"scenario": job.scenario, "condition": job.condition, "seed": job.seed, "status": "error",
                "message": f"{type(exc).__name__}: {exc}"}
    wall = time.perf_counter() - t0
    run.info["cpu_s"] = sum(proc.cpu_times()[:2]) - cpu0
    run.info["rss_mb"] = proc.memory_info().rss / 2**20
    metrics = mission_metrics(run)
    d = Path(job.out_dir) / "runs" / job.scenario / job.condition / f"seed_{job.seed}"
    d.mkdir(parents=True, exist_ok=True)
    meta = {"scenario": job.scenario, "condition": job.condition, "controller": ctrl, "framework": dataclasses.asdict(fw),
            "seed": job.seed, "status": run.status, "message": run.info.get("message", ""), "wall_s": wall,
            "fault_onset": scn.fault_onset, "fault_class": scn.fault_class,
            "truth_events": [dataclasses.asdict(e) for e in scn.truth_events],
            "sensor_faults": [dataclasses.asdict(f) for f in scn.sensor_faults],
            "metrics": metrics, "aircraft": {"name": "dji_matrice_400"}, "feedback_mode": "eskf_estimated"}
    (d / "metrics.json").write_text(json.dumps(meta, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    with open(d / "events.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["t", "source", "event", "detail", "value"])
        w.writeheader()
        w.writerows(run.events)
        for ev in run.info.get("mission_events", []):
            w.writerow({"t": ev["t"], "source": "mission", "event": ev["event"],
                        "detail": ev.get("reason", ev.get("kind", "")), "value": ev.get("value")})
    if job.keep_timeseries:
        with gzip.open(d / "timeseries.csv.gz", "wt", encoding="utf-8") as f:
            f.write(",".join(run.columns) + "\n")
            np.savetxt(f, run.data, delimiter=",", fmt="%.6g")
    return {"scenario": job.scenario, "condition": job.condition, "seed": job.seed, "status": run.status,
            "message": run.info.get("message", ""), "wall_s": round(wall, 2), **metrics}


def code_hash() -> str:
    """SHA-256 over every source and configuration file (identifies the code even in a dirty git tree)."""
    import hashlib

    h = hashlib.sha256()
    for root in ("src", "config"):
        for f in sorted((REPO_ROOT / root).rglob("*")):
            if f.is_file() and f.suffix in (".py", ".yaml", ".json") and "__pycache__" not in f.parts:
                h.update(str(f.relative_to(REPO_ROOT)).encode())
                h.update(f.read_bytes())
    return h.hexdigest()


def make_jobs(scenarios: list[str], conditions: list[str], seeds: list[int], out_dir: Path, keep_timeseries: int) -> list[Job]:
    """Seed-major order: every (scenario, condition) cell receives seed k before any cell receives seed k+1,
    so a campaign stopped early still has the same number of seeds in every cell."""
    return [Job(s, c, sd, str(out_dir), i < keep_timeseries)
            for i, sd in enumerate(seeds) for s in scenarios for c in conditions]


def _existing_row(job: Job) -> dict[str, Any] | None:
    f = Path(job.out_dir) / "runs" / job.scenario / job.condition / f"seed_{job.seed}" / "metrics.json"
    if not f.is_file():
        return None
    m = json.loads(f.read_text())
    return {"scenario": job.scenario, "condition": job.condition, "seed": job.seed, "status": m["status"],
            "message": m.get("message", ""), "wall_s": round(m.get("wall_s", float("nan")), 2), **m["metrics"]}


def run_campaign(name: str, scenarios: list[str], conditions: list[str], seeds: list[int], workers: int = 8,
                 keep_timeseries: int = 2, root: Path = CAMPAIGN_DIR) -> Path:
    out = root / name
    out.mkdir(parents=True, exist_ok=True)
    history = []
    if (out / "campaign.json").is_file():  # resumed campaign: keep the record of earlier code versions
        prev = json.loads((out / "campaign.json").read_text())
        history = prev.get("history", []) + [{"code_sha256": prev.get("code_sha256"), "git": prev.get("git"),
                                               "started_utc": prev.get("started_utc")}]
    (out / "campaign.json").write_text(json.dumps({
        "history": history,
        "name": name, "scenarios": scenarios, "conditions": {c: [ALL_CONFIGS[c][0], dataclasses.asdict(ALL_CONFIGS[c][1])]
                                                             for c in conditions},
        "seeds": seeds, "git": git_info(), "code_sha256": code_hash(), "software": software_versions(),
        "aircraft": "dji_matrice_400",
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }, indent=2))
    jobs = make_jobs(scenarios, conditions, seeds, out, keep_timeseries)
    rows: list[dict[str, Any]] = []
    todo = []
    for j in jobs:  # resume: runs with a metrics.json are not repeated
        r = _existing_row(j)
        if r is None:
            todo.append(j)
        else:
            rows.append(r)
    if rows:
        print(f"[{name}] resuming: {len(rows)} runs already done, {len(todo)} to go", flush=True)
    jobs = todo
    summary = out / "summary.csv"
    t0 = time.time()
    with Pool(workers) as pool:
        for i, row in enumerate(pool.imap_unordered(run_job, jobs, chunksize=1), start=1):
            rows.append(row)
            if i % 10 == 0 or i == len(jobs):
                print(f"[{name}] {i}/{len(jobs)} new runs ({len(rows)} total), {time.time() - t0:.0f} s elapsed", flush=True)
                _write_rows(summary, rows)
    _write_rows(summary, rows)
    return summary


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    keys: list[str] = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    rows = sorted(rows, key=lambda r: (r["scenario"], r["condition"], r["seed"]))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
