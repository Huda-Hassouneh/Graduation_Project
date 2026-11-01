"""Statistics for repeated seeded experiments: bootstrap confidence intervals and paired comparisons.

* :func:`bootstrap_ci` percentile bootstrap of the mean (seeded, 5000 resamples) with NaNs dropped;
* :func:`paired_comparison` mean of per-seed differences ``b - a`` with a bootstrap CI and the Wilcoxon
  signed-rank p-value (scipy). Pairs are matched by seed, so both runs saw the same disturbance sequence.

With n = 30 the CI half-width of a proportion near 0.5 is about +-0.18: small effects are not
resolvable and are reported as such (docs/EXPERIMENTAL_METHODOLOGY.md).
"""

from __future__ import annotations

import numpy as np
from scipy import stats as sps


def bootstrap_ci(x, level: float = 0.95, n_boot: int = 5000, seed: int = 0) -> tuple[float, float, float, int]:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    n = len(x)
    if n == 0:
        return float("nan"), float("nan"), float("nan"), 0
    if n == 1 or np.all(x == x[0]):
        return float(x.mean()), float(x.mean()), float(x.mean()), n
    rng = np.random.default_rng(seed)
    means = x[rng.integers(0, n, size=(n_boot, n))].mean(axis=1)
    lo, hi = np.percentile(means, [100 * (1 - level) / 2, 100 * (1 + level) / 2])
    return float(x.mean()), float(lo), float(hi), n


def paired_comparison(a, b, seed: int = 0) -> dict[str, float]:
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ok = np.isfinite(a) & np.isfinite(b)
    d = b[ok] - a[ok]
    mean, lo, hi, n = bootstrap_ci(d, seed=seed)
    if n >= 6 and np.any(d != 0):
        p = float(sps.wilcoxon(d, zero_method="zsplit").pvalue)
    else:
        p = float("nan")
    return {"mean_diff": mean, "ci_lo": lo, "ci_hi": hi, "n_pairs": n, "p_wilcoxon": p}
