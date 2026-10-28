"""Publication figures for single runs and aggregate tables (static PNG, light background).

Colour roles follow a validated categorical palette (slot order fixed, never cycled):
measured/actual = slot 1, second series = slot 2; references are neutral dashed ink so the
data series carry the colour. Each quantity has its own axis (no dual y-axes).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from graduation_project.experiments.runner import RunResult  # noqa: E402

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]  # categorical slots 1-4
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": INK_SECONDARY,
        "axes.labelcolor": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "xtick.color": INK_SECONDARY,
        "ytick.color": INK_SECONDARY,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "font.size": 9,
        "axes.titlesize": 10,
        "legend.frameon": False,
        "lines.linewidth": 1.6,
    }
)


def _ref_style() -> dict:
    return {"color": INK_SECONDARY, "linestyle": "--", "linewidth": 1.1}


def run_overview(run: RunResult, title: str, path: Path) -> None:
    """3D path, per-axis position, error norm, attitude, and power/energy for one run."""
    t = run.col("t")
    p, pr = run.cols("p"), run.cols("ref_p")
    fig = plt.figure(figsize=(11, 7.2), constrained_layout=True)
    gs = fig.add_gridspec(4, 3)

    ax3 = fig.add_subplot(gs[0:2, 0], projection="3d")
    ax3.plot(*pr.T, label="reference", **_ref_style())
    ax3.plot(*p.T, color=SERIES[0], label="actual")
    ax3.set_xlabel("x east [m]")
    ax3.set_ylabel("y north [m]")
    ax3.set_zlabel("z up [m]")
    ax3.legend(loc="upper left", fontsize=8)
    ax3.set_title("Path (world ENU)")

    for i, lab in enumerate("xyz"):
        ax = fig.add_subplot(gs[i, 1])
        ax.plot(t, pr[:, i], **_ref_style(), label="reference")
        ax.plot(t, p[:, i], color=SERIES[0], label="actual")
        ax.set_ylabel(f"{lab} [m]")
        if i == 0:
            ax.set_title("Position")
            ax.legend(loc="best", fontsize=8)
    ax = fig.add_subplot(gs[3, 1])
    ax.plot(t, 100 * run.col("pos_err"), color=SERIES[0])
    ax.set_ylabel("|e_p| [cm]")
    ax.set_xlabel("time [s]")

    names = [("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw")]
    from graduation_project.dynamics.frames import euler_zyx_from_quat

    qd = run.cols("qdes", "wxyz")
    eul_des = np.rad2deg(np.array([euler_zyx_from_quat(q) for q in qd]))
    for i, (col, lab) in enumerate(names):
        ax = fig.add_subplot(gs[i, 2])
        ax.plot(t, eul_des[:, i], **_ref_style(), label="desired")
        ax.plot(t, np.rad2deg(run.col(col)), color=SERIES[0], label="actual")
        ax.set_ylabel(f"{lab} [deg]")
        if i == 0:
            ax.set_title("Attitude (Z-Y-X Euler, logging only)")
            ax.legend(loc="best", fontsize=8)
    ax = fig.add_subplot(gs[3, 2])
    ax.plot(t, run.col("power_el"), color=SERIES[0])
    ax.set_ylabel("P_el [W]")
    ax.set_xlabel("time [s]")
    ax.text(0.98, 0.9, f"E = {run.col('energy')[-1] / 1e3:.2f} kJ", transform=ax.transAxes, ha="right",
            va="top", color=INK, fontsize=8)

    ax = fig.add_subplot(gs[2:4, 0])
    omega = run.cols("omega", "1234") * 60 / (2 * np.pi)
    for i in range(omega.shape[1]):
        ax.plot(t, omega[:, i], color=SERIES[i], linewidth=1.1, label=f"rotor {i + 1}")
    ax.set_ylabel("rotor speed [rpm]")
    ax.set_xlabel("time [s]")
    ax.set_title("Actual rotor speeds")
    ax.legend(loc="best", fontsize=8, ncol=2)

    fig.suptitle(title, color=INK, fontsize=11)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def grouped_bars(
    categories: list[str], groups: dict[str, list[float]], ylabel: str, title: str, path: Path, fmt: str = "{:.3f}"
) -> None:
    """Grouped bars (one colour per group, fixed slot order) with value labels above bars."""
    n_g = len(groups)
    x = np.arange(len(categories))
    width = 0.8 / n_g
    fig, ax = plt.subplots(figsize=(max(6.0, 0.9 * len(categories) + 2), 3.6), constrained_layout=True)
    for k, (name, vals) in enumerate(groups.items()):
        pos = x - 0.4 + width * (k + 0.5)
        ax.bar(pos, vals, width=width * 0.92, color=SERIES[k], label=name, edgecolor=SURFACE, linewidth=1.0)
        for xi, v in zip(pos, vals):
            ax.text(xi, v, fmt.format(v), ha="center", va="bottom", fontsize=7, color=INK_SECONDARY)
    ax.set_xticks(x, categories, rotation=20, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="x", visible=False)
    ax.legend(loc="upper left")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
