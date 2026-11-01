#!/usr/bin/env python3
"""Aggregate a campaign: per-cell statistics with bootstrap 95 % CIs, paired comparisons, figures.

Usage (from the repository root):
    python3 scripts/analyze_campaign.py results/campaign/main_30seeds
    python3 scripts/analyze_campaign.py results/campaign/ablations_10seeds --reference C

Writes into the campaign directory:
    stats_by_cell.csv        mean, CI, n for every (scenario, condition, metric)
    paired_comparisons.csv   per scenario: B-A, C-A, D-B, D-C (main) or ablation-C (ablations), matched seeds
    report.md                tables of the headline metrics (generated numbers only)
and figures under figures/campaign/<campaign name>/.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from graduation_project.metrics.stats import bootstrap_ci, paired_comparison  # noqa: E402

HEADLINE = [
    ("pos_rmse_m", "position RMSE [m]", "{:.3f}"),
    ("pos_max_err_m", "max position error [m]", "{:.2f}"),
    ("detected", "detection rate [-]", "{:.2f}"),
    ("detection_delay_s", "detection delay [s]", "{:.1f}"),
    ("classified_correct", "correct classification [-]", "{:.2f}"),
    ("false_alarm", "false-alarm rate [-]", "{:.2f}"),
    ("recovery_time_s", "recovery time [s]", "{:.1f}"),
    ("coverage_fraction", "coverage [-]", "{:.2f}"),
    ("safe_outcome", "safe outcome [-]", "{:.2f}"),
    ("min_clearance_m", "min clearance [m]", "{:.2f}"),
    ("energy_total_kj", "energy [kJ]", "{:.0f}"),
    ("energy_diagnostic_kj", "diagnostic energy [kJ]", "{:.1f}"),
    ("energy_remaining_kj", "remaining energy [kJ]", "{:.0f}"),
    ("cycle_ms_mean", "cycle time mean [ms]", "{:.2f}"),
]
MAIN_PAIRS = [("A", "B"), ("A", "C"), ("B", "D"), ("C", "D")]


def load(summary: Path) -> list[dict]:
    rows = list(csv.DictReader(open(summary, encoding="utf-8")))
    for r in rows:
        for k, v in r.items():
            if k not in ("scenario", "condition", "status", "message"):
                try:
                    r[k] = float(v)
                except (TypeError, ValueError):
                    r[k] = float("nan")
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("campaign", type=Path)
    ap.add_argument("--reference", default=None, help="reference condition for ablation comparisons (e.g. C)")
    args = ap.parse_args()
    rows = load(args.campaign / "summary.csv")
    errors = [r for r in rows if r["status"] == "error"]
    rows = [r for r in rows if r["status"] != "error"]
    scenarios = sorted({r["scenario"] for r in rows})
    conditions = sorted({r["condition"] for r in rows}, key=lambda c: (len(c) > 1, c))
    metrics = [k for k in rows[0] if isinstance(rows[0][k], float) and k not in ("seed",)]
    cell = defaultdict(list)
    for r in rows:
        cell[(r["scenario"], r["condition"])].append(r)

    with open(args.campaign / "stats_by_cell.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "condition", "metric", "mean", "ci_lo", "ci_hi", "n"])
        for (s, c), rs in sorted(cell.items()):
            for k in metrics:
                mean, lo, hi, n = bootstrap_ci([r[k] for r in rs])
                w.writerow([s, c, k, mean, lo, hi, n])

    pairs = MAIN_PAIRS if args.reference is None else [(args.reference, c) for c in conditions if c != args.reference]
    comp_rows = []
    for s in scenarios:
        for a, b in pairs:
            ra = {r["seed"]: r for r in cell.get((s, a), [])}
            rb = {r["seed"]: r for r in cell.get((s, b), [])}
            seeds = sorted(set(ra) & set(rb))
            if not seeds:
                continue
            for k, _, _ in HEADLINE:
                pc = paired_comparison([ra[sd][k] for sd in seeds], [rb[sd][k] for sd in seeds])
                comp_rows.append({"scenario": s, "a": a, "b": b, "metric": k, **pc})
    with open(args.campaign / "paired_comparisons.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["scenario", "a", "b", "metric", "mean_diff", "ci_lo", "ci_hi", "n_pairs", "p_wilcoxon"])
        w.writeheader()
        w.writerows(comp_rows)

    # markdown report (generated numbers only)
    lines = [f"# Campaign `{args.campaign.name}` - generated summary", "",
             f"Runs: {len(rows)} ({len(errors)} errors). Conditions: {', '.join(conditions)}. "
             f"Seeds per cell: {min(len(v) for v in cell.values())}-{max(len(v) for v in cell.values())}.", "",
             "Values: mean [95 % bootstrap CI]. Detection/classification metrics use the scenario's fault label "
             "(evaluation only). See docs/EXPERIMENTAL_METHODOLOGY.md for definitions.", ""]
    for k, label, fmt in HEADLINE:
        lines += [f"## {label} (`{k}`)", "", "| scenario | " + " | ".join(conditions) + " |",
                  "|---|" + "---|" * len(conditions)]
        for s in scenarios:
            cells = []
            for c in conditions:
                mean, lo, hi, n = bootstrap_ci([r[k] for r in cell.get((s, c), [])])
                cells.append("n/a" if n == 0 else f"{fmt.format(mean)} [{fmt.format(lo)}, {fmt.format(hi)}]")
            lines.append(f"| {s} | " + " | ".join(cells) + " |")
        lines.append("")
    lines += ["## Paired differences (b - a, matched seeds)", "",
              "| scenario | b - a | metric | mean diff [95 % CI] | Wilcoxon p | n |", "|---|---|---|---|---|---|"]
    for cr in comp_rows:
        if cr["metric"] in ("pos_rmse_m", "safe_outcome", "energy_total_kj", "coverage_fraction") and cr["n_pairs"] > 0:
            lines.append(f"| {cr['scenario']} | {cr['b']} - {cr['a']} | {cr['metric']} | {cr['mean_diff']:.3g} "
                         f"[{cr['ci_lo']:.3g}, {cr['ci_hi']:.3g}] | {cr['p_wilcoxon']:.3g} | {cr['n_pairs']} |")
    if errors:
        lines += ["", "## Errors", ""] + [f"- {e['scenario']} {e['condition']} seed {int(e['seed'])}: {e['message']}" for e in errors]
    (args.campaign / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # figures
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig_dir = REPO / "figures" / "campaign" / args.campaign.name
    fig_dir.mkdir(parents=True, exist_ok=True)
    colors = ["#4C72B0", "#DD8452", "#55A868", "#C44E52", "#8172B3", "#937860", "#DA8BC3", "#8C8C8C", "#CCB974"]
    for k, label, _ in HEADLINE:
        fig, ax = plt.subplots(figsize=(max(8, 0.9 * len(scenarios) * len(conditions) / 3), 4.2))
        width = 0.8 / len(conditions)
        for j, c in enumerate(conditions):
            means, los, his = [], [], []
            for s in scenarios:
                mean, lo, hi, _ = bootstrap_ci([r[k] for r in cell.get((s, c), [])])
                means.append(mean)
                los.append(mean - lo if np.isfinite(lo) else 0)
                his.append(hi - mean if np.isfinite(hi) else 0)
            x = np.arange(len(scenarios)) + (j - (len(conditions) - 1) / 2) * width
            ax.bar(x, means, width, yerr=[los, his], capsize=2, label=c, color=colors[j % len(colors)])
        ax.set_xticks(np.arange(len(scenarios)), scenarios, rotation=35, ha="right", fontsize=8)
        ax.set_ylabel(label)
        ax.set_title(f"{args.campaign.name}: {label} (mean, 95 % bootstrap CI)", fontsize=10)
        ax.legend(fontsize=8, ncol=min(len(conditions), 6))
        ax.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(fig_dir / f"{k}.png", dpi=130)
        plt.close(fig)
    print(f"wrote {args.campaign / 'stats_by_cell.csv'}, paired_comparisons.csv, report.md and {len(HEADLINE)} figures in "
          f"{fig_dir.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
