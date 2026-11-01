"""Recursive least squares with exponential forgetting and covariance bounding.

Model ``y_k = phi_k^T theta + e_k`` with ``e_k ~ N(0, sigma^2)``::

    K     = P phi / (lambda sigma^2 + phi^T P phi)
    theta = theta + K (y - phi^T theta)
    P     = (P - K phi^T P) / lambda,   then clipped to trace(P) <= p_max (anti-windup)

``P`` is kept in units of the parameter covariance (the noise variance is part of the gain), so
``sqrt(diag(P))`` is the parameter standard deviation and ``0.5 log det(P_prior / P_post)`` is the
information gained. With forgetting factor ``lambda`` the memory is ``~1 / (1 - lambda)`` samples.
Without excitation (``phi`` constant or zero) the covariance grows back toward ``p_max``: the estimator
reports that the parameter is not currently identifiable instead of pretending to know it.
"""

from __future__ import annotations

import numpy as np

from graduation_project.dynamics.frames import FloatArray


class RecursiveLeastSquares:
    def __init__(self, theta0: FloatArray, p0: FloatArray, noise_std: float, forgetting: float = 0.995,
                 p_max: FloatArray | None = None) -> None:
        self.theta = np.asarray(theta0, dtype=float).copy()
        n = len(self.theta)
        self.P = np.diag(np.asarray(p0, dtype=float)) if np.ndim(p0) == 1 else np.asarray(p0, dtype=float).copy()
        if self.P.shape != (n, n):
            raise ValueError("covariance shape mismatch")
        if not 0.0 < forgetting <= 1.0 or noise_std <= 0.0:
            raise ValueError("invalid forgetting factor or noise std")
        self.noise_var = noise_std**2
        self.lam = forgetting
        self.p_max = np.diag(self.P).copy() if p_max is None else np.asarray(p_max, dtype=float)
        self.n_updates = 0

    @property
    def std(self) -> FloatArray:
        return np.sqrt(np.maximum(np.diag(self.P), 0.0))

    def update(self, phi: FloatArray, y: float) -> float:
        """Assimilate one sample; returns the a-priori prediction error."""
        phi = np.asarray(phi, dtype=float)
        if not (np.all(np.isfinite(phi)) and np.isfinite(y)):
            return float("nan")
        err = float(y - phi @ self.theta)
        Pphi = self.P @ phi
        denom = self.lam * self.noise_var + float(phi @ Pphi)
        K = Pphi / denom
        self.theta = self.theta + K * err
        self.P = (self.P - np.outer(K, Pphi)) / self.lam
        self.P = 0.5 * (self.P + self.P.T)
        d = np.diag(self.P)
        over = d > self.p_max
        if np.any(over):  # bound the covariance per parameter (prevents blow-up without excitation)
            s = np.ones_like(d)
            s[over] = np.sqrt(self.p_max[over] / d[over])
            self.P = self.P * np.outer(s, s)
        self.n_updates += 1
        return err

    def information_gain(self, phis: FloatArray) -> float:
        """Expected information gain [nats] of a planned regressor sequence (rows of ``phis``), no forgetting."""
        info = np.linalg.inv(self.P) + (np.asarray(phis).T @ np.asarray(phis)) / self.noise_var
        sign, logdet_post_inv = np.linalg.slogdet(info)
        sign0, logdet_prior = np.linalg.slogdet(self.P)
        return 0.5 * float(logdet_prior + logdet_post_inv)
