#!/usr/bin/env python3
"""Run a seeded experiment campaign on the DJI Matrice 400 reference model (Phase I).

Usage (from the repository root):
    python3 scripts/run_campaign.py --suite smoke                          # 1 seed, all scenarios, A-D (~5 min)
    python3 scripts/run_campaign.py --suite main --seeds 30                # A-D x 13 scenarios x 30 seeds
    python3 scripts/run_campaign.py --suite ablations --seeds 10
    python3 scripts/run_campaign.py --suite main --scenarios nominal mass_change --seeds 5 --name trial

Evaluation seeds start at 1000 (development used seed 0), so evaluation never reuses tuning runs.
Outputs: results/campaign/<name>/summary.csv, campaign.json and runs/... (see experiments/campaign.py).
Analyse with: python3 scripts/analyze_campaign.py results/campaign/<name>
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.experiments.campaign import ABLATIONS, CONDITIONS, run_campaign  # noqa: E402
from graduation_project.experiments.mission_scenarios import list_scenarios  # noqa: E402

ABLATION_SCENARIOS = ["mass_change", "drag_change", "response_delay", "sensor_delay", "energy_limited", "compound"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--suite", choices=["smoke", "main", "ablations"], required=True)
    ap.add_argument("--seeds", type=int, default=None, help="number of seeds (default: smoke 1, main 30, ablations 10)")
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--scenarios", nargs="*")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--keep-timeseries", type=int, default=2, help="seeds per cell whose time series are saved")
    ap.add_argument("--name")
    args = ap.parse_args()

    n = args.seeds or {"smoke": 1, "main": 30, "ablations": 10}[args.suite]
    seeds = list(range(args.seed0, args.seed0 + n))
    if args.suite == "ablations":
        scenarios = args.scenarios or ABLATION_SCENARIOS
        conditions = ["C", *ABLATIONS]
    else:
        scenarios = args.scenarios or list_scenarios()
        conditions = list(CONDITIONS)
    name = args.name or f"{args.suite}_{n}seeds"
    summary = run_campaign(name, scenarios, conditions, seeds, args.workers, args.keep_timeseries)
    print(f"summary -> {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
