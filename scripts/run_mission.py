#!/usr/bin/env python3
"""Run ONE DJI Matrice 400 facade-coverage mission in the Python simulator and save logs + an overview figure.

Usage (from the repository root):
    python3 scripts/run_mission.py --scenario nominal
    python3 scripts/run_mission.py --scenario mass_change --controller dob --seed 3
    python3 scripts/run_mission.py --scenario sensor_delay --framework off       # stock behaviour (A/B)
    python3 scripts/run_mission.py --scenario drag_change --ablation passive

Outputs results/missions/<id>/{timeseries.csv, events.csv, metadata.json, overview.png}. The run directory is
compatible with the Gazebo kinematic replay:
    simulation/gazebo/launch_world.sh facade_complex_m400 &
    python3 simulation/gazebo/replay_run.py --world facade_complex_m400 --run results/missions/<id> --speed 4
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.experiments.campaign import ABLATIONS, FW_OFF, FW_ON  # noqa: E402
from graduation_project.experiments.io import git_info, software_versions  # noqa: E402
from graduation_project.experiments.mission_runner import run_mission  # noqa: E402
from graduation_project.experiments.mission_scenarios import list_scenarios, load_scenario  # noqa: E402
from graduation_project.metrics.mission import mission_metrics  # noqa: E402
from graduation_project.planning.mission import PHASES  # noqa: E402


def overview(run, path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = run.col("t")
    fig = plt.figure(figsize=(14, 9))
    ax = fig.add_subplot(2, 3, 1, projection="3d")
    ax.plot(run.col("ref_p_x"), run.col("ref_p_y"), run.col("ref_p_z"), "k--", lw=0.8, label="reference")
    ax.plot(run.col("p_x"), run.col("p_y"), run.col("p_z"), lw=1.2, label="true")
    xx, zz = np.meshgrid([0, 12], [0, 12])
    ax.plot_surface(xx, np.full_like(xx, 8.0), zz, alpha=0.15, color="grey")
    ax.set_xlabel("x [m]"), ax.set_ylabel("y [m]"), ax.set_zlabel("z [m]"), ax.legend(fontsize=7)
    ax.set_title("trajectory (facade plane y = 8 m)", fontsize=9)
    panels = [
        (2, [("pos_err", "tracking error (true)"), ("est_err", "estimation error")], "error [m]"),
        (3, [("standoff", "stand-off (true)"), ("standoff_ref", "stand-off ref"), ("clearance", "envelope clearance")], "[m]"),
        (4, [("power_true", "electrical power")], "P [W]"),
        (5, [("thrust_gain_hat", "identified thrust gain"), ("speed_scale", "speed scale")], "[-]"),
        (6, [("anomaly", "anomaly"), ("determined", "determined"), ("probe", "probe active")], "flag"),
    ]
    for idx, series, ylabel in panels:
        a = fig.add_subplot(2, 3, idx)
        for col, label in series:
            a.plot(t, run.col(col), lw=1, label=label)
        onset = run.info.get("fault_onset")
        if onset is not None:
            a.axvline(onset, color="r", ls=":", lw=1, label="fault onset (truth)")
        a.set_xlabel("t [s]"), a.set_ylabel(ylabel), a.grid(alpha=0.3), a.legend(fontsize=7)
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=list_scenarios(), default="nominal")
    ap.add_argument("--controller", choices=["pid", "dob"], default="pid")
    ap.add_argument("--framework", choices=["on", "off"], default="on")
    ap.add_argument("--ablation", choices=sorted(ABLATIONS))
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    fw = ABLATIONS[args.ablation][1] if args.ablation else (FW_ON if args.framework == "on" else FW_OFF)
    scn = load_scenario(args.scenario, args.seed)
    run = run_mission(scn, args.controller, fw)
    metrics = mission_metrics(run)
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = REPO / "results" / "missions" / f"{stamp}_{args.scenario}_{args.controller}_{'fw' if fw.enabled else 'nofw'}_s{args.seed}"
    out.mkdir(parents=True)
    np.savetxt(out / "timeseries.csv", run.data, delimiter=",", header=",".join(run.columns), comments="", fmt="%.9g")
    with open(out / "events.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["t", "source", "event", "detail", "value"])
        w.writeheader()
        w.writerows(run.events)
    meta = {"scenario": args.scenario, "description": scn.description, "expected_behaviour": scn.expected_behaviour,
            "seed": args.seed, "controller": args.controller, "framework": dataclasses.asdict(fw), "status": run.status,
            "aircraft": {"name": "dji_matrice_400"}, "feedback_mode": "eskf_estimated", "metrics": metrics,
            "truth_events": [dataclasses.asdict(e) for e in scn.truth_events],
            "sensor_faults": [dataclasses.asdict(f) for f in scn.sensor_faults],
            "git": git_info(), "software": software_versions()}
    (out / "metadata.json").write_text(json.dumps(meta, indent=2, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    overview(run, out / "overview.png", f"{args.scenario} | {args.controller} | framework {'on' if fw.enabled else 'off'}"
             f"{' (' + args.ablation + ')' if args.ablation else ''} | seed {args.seed} | {run.status}")
    print(f"status: {run.status} {run.info.get('message', '')}")
    for k in ("pos_rmse_m", "pos_max_err_m", "detected", "detection_delay_s", "classified_correct", "coverage_fraction",
              "min_clearance_m", "energy_total_kj", "safe_outcome"):
        print(f"  {k:22s} {metrics.get(k, float('nan')):.4g}")
    for e in run.events[:20]:
        print(f"  {e['t']:7.2f} {e['source']:9s} {e['event']:22s} {e['detail'][:90]}")
    print(f"-> {out.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
