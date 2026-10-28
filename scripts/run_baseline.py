#!/usr/bin/env python3
"""Run the Milestone 2 baseline scenario suite (cascaded PID, ideal state feedback).

Usage (from the repository root):
    PYTHONPATH=src python3 scripts/run_baseline.py                       # all scenarios, both aircraft
    PYTHONPATH=src python3 scripts/run_baseline.py --aircraft research_quad --scenario circle

Outputs:
    results/runs/<experiment_id>/{timeseries.csv,metadata.json}   (one per run, git-ignored)
    results/baseline_summary.csv                                  (one row per run)
    figures/baseline/<aircraft>/<scenario>.png and figures/baseline/summary_*.png
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.config import load_aircraft  # noqa: E402
from graduation_project.controllers.pid import CascadedPIDController, load_pid_gains  # noqa: E402
from graduation_project.experiments.io import new_experiment_id, save_run  # noqa: E402
from graduation_project.experiments.runner import run_closed_loop  # noqa: E402
from graduation_project.experiments.scenarios import SCENARIOS  # noqa: E402
from graduation_project.metrics.tracking import tracking_metrics  # noqa: E402
from graduation_project.sensors.state_estimate import IDEAL_TRUTH  # noqa: E402
from graduation_project.visualization.plots import grouped_bars, run_overview  # noqa: E402

AIRCRAFT = ["research_quad", "m400_reference_quad"]
SEED = 0  # no stochastic elements yet; recorded for protocol consistency


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aircraft", choices=AIRCRAFT, action="append")
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), action="append")
    ap.add_argument("--no-figures", action="store_true")
    args = ap.parse_args()
    aircraft = args.aircraft or AIRCRAFT
    scenarios = args.scenario or list(SCENARIOS)

    rows = []
    for acn in aircraft:
        ac = load_aircraft(acn)
        gains = load_pid_gains(acn)
        for name in scenarios:
            sc = SCENARIOS[name]()
            t0 = time.perf_counter()
            run = run_closed_loop(ac, CascadedPIDController(ac, gains), sc.trajectory, sc.sim)
            wall = time.perf_counter() - t0
            metrics = tracking_metrics(run, sc.eval_start_s) if run.status == "completed" else {}
            exp_id = new_experiment_id(name, acn, SEED)
            meta = {
                "experiment_id": exp_id,
                "milestone": 2,
                "scenario": name,
                "description": sc.description,
                "seed": SEED,
                "feedback_mode": IDEAL_TRUTH,
                "aircraft": ac.raw,
                "controller": {"type": "CascadedPIDController", "gains": dataclasses.asdict(gains)},
                "sim": dataclasses.asdict(sc.sim),
                "initial_conditions": {"reference_t0": sc.trajectory.sample(0.0).__dict__,
                                       "initial_offset": sc.sim.initial_offset},
                "trajectory": {"type": type(sc.trajectory).__name__, "duration_s": sc.trajectory.duration},
                "wind": {"type": "none"},
                "injected_changes": [],
                "metrics": metrics,
                "wall_time_s": wall,
            }
            run_dir = save_run(run, meta)
            if not args.no_figures and run.status == "completed":
                run_overview(run, f"{acn} | {name}: {sc.description}", REPO / "figures" / "baseline" / acn / f"{name}.png")
            rows.append({"aircraft": acn, "scenario": name, "status": run.status, "experiment_id": exp_id,
                         **{k: metrics.get(k, float("nan")) for k in
                            ("pos_rmse_m", "pos_max_err_m", "att_err_max_deg", "tilt_max_deg", "energy_J",
                             "mean_power_W", "energy_per_m_J", "saturation_ratio")}})
            print(f"{acn:22s} {name:18s} {run.status:9s} rmse={metrics.get('pos_rmse_m', float('nan')):.4f} m "
                  f"max={metrics.get('pos_max_err_m', float('nan')):.4f} m  wall={wall:.1f}s  -> {run_dir.name}",
                  flush=True)

    out = REPO / "results" / "baseline_summary.csv"
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    if not args.no_figures and len(aircraft) > 1:
        cats = scenarios
        for key, label, fname, fmt in [("pos_rmse_m", "position RMSE [m]", "summary_rmse.png", "{:.3f}"),
                                       ("pos_max_err_m", "max position error [m]", "summary_max_err.png", "{:.2f}")]:
            groups = {acn: [r[key] for r in rows if r["aircraft"] == acn] for acn in aircraft}
            grouped_bars(cats, groups, label, f"Baseline PID, ideal state feedback: {label}",
                         REPO / "figures" / "baseline" / fname, fmt)
    print(f"summary -> {out}")
    return 0 if all(r["status"] == "completed" for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
