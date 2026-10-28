#!/usr/bin/env python3
"""Regenerate per-run overview figures from saved logs (results/runs/*/timeseries.csv).

Figures are always drawn from logged data, never from in-memory results, so every figure
is traceable to a run directory. For each (aircraft, scenario) the newest run is used.

    python3 scripts/plot_runs.py [--milestone 2]
"""

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.experiments.io import RUNS_DIR, load_run_csv  # noqa: E402
from graduation_project.experiments.runner import RunResult  # noqa: E402
from graduation_project.visualization.plots import run_overview  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--milestone", type=int, default=2)
args = ap.parse_args()

latest: dict[tuple[str, str], tuple[str, Path, dict]] = {}
for meta_path in sorted(RUNS_DIR.glob("*/metadata.json")):
    meta = json.loads(meta_path.read_text())
    if meta.get("milestone") != args.milestone or meta.get("status") != "completed":
        continue
    key = (meta["aircraft"]["name"], meta["scenario"])
    if key not in latest or meta["created_utc"] > latest[key][0]:
        latest[key] = (meta["created_utc"], meta_path.parent, meta)

for (acn, scen), (_, run_dir, meta) in sorted(latest.items()):
    cols, data = load_run_csv(run_dir / "timeseries.csv")
    run = RunResult(cols, data, meta["status"])
    out = REPO / "figures" / "baseline" / acn / f"{scen}.png"
    run_overview(run, f"{acn} | {scen}: {meta['description']}  [run {meta['experiment_id']}]", out)
    print(f"{out.relative_to(REPO)}  <-  {run_dir.name}")
