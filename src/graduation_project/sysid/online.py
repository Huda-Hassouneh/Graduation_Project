"""Online identification of the parameters that change in the research scenarios.

All inputs are signals available on a real aircraft: bias-corrected IMU specific force and body
rate (estimator), the estimated velocity/attitude, and the controller's own allocated wrench.

=====================  =====================================================  ===============================
estimator              model                                                  identifiable when
=====================  =====================================================  ===============================
thrust gain theta      f_z = theta * T_model / m_nom  (RLS, scalar)           always in flight (T ~ m g)
effective lag tau      f_z = theta * LPF_tau(T_cmd) / m_nom over a grid of    thrust varies at ~0.3-3 Hz
                       tau (Bayesian output-error)                            (needs excitation; not at hover)
drag / wind (per       a_ext = -c (v - w)|v - w|  (grid over the wind w,      airspeed varies (passes, probes)
horizontal axis)       least-squares c per grid point)
GNSS latency delta     r_gnss = -v delta  (RLS on innovations)                ground speed > ~0.3 m/s
=====================  =====================================================  ===============================

Identifiability notes (docs/DIAGNOSIS_AND_RECOVERY.md):

* ``theta = eta_eff * m_nom / m``: a mass increase and a uniform thrust loss give the SAME theta. They are
  not separable from translational data; only an asymmetric thrust loss leaves a torque signature.
* The lag estimate captures motor lag plus any pure transport delay (``tau_eff``); the two are not
  separated with the excitation used here.
* Drag coefficient and wind are jointly identifiable only while the ground speed varies; at constant
  speed every (c, w) pair on the curve ``c (v - w)^2 = F`` fits equally well (flat grid posterior).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from graduation_project.dynamics.frames import FloatArray
from graduation_project.sysid.rls import RecursiveLeastSquares


@dataclass(frozen=True)
class SysIdConfig:
    rate: float = 50.0  # framework update rate [Hz]
    gain_noise_std: float = 0.06  # [m/s^2] specific-force noise + model error per update
    gain_time_constant: float = 4.0  # memory [s]
    gain_prior_std: float = 0.05
    lag_grid: tuple[float, ...] = (0.03, 0.045, 0.06, 0.08, 0.10, 0.13, 0.16, 0.20, 0.25, 0.32, 0.40)
    lag_noise_std: float = 0.06  # [m/s^2]
    lag_time_constant: float = 6.0  # [s]
    drag_noise_std: float = 0.06  # [m/s^2]
    drag_time_constant: float = 15.0  # [s]
    delay_noise_std: float = 0.03  # [m]
    delay_time_constant: float = 10.0  # [s]
    delay_prior_std: float = 0.15  # [s]


class LagPosterior:
    """Grid posterior over the effective first-order response time constant.

    Each grid point ``tau_j`` runs a filter ``y_j' = (u - y_j) / tau_j`` of the commanded collective
    acceleration ``u``. With exponentially weighted sums ``S_yy, S_yf, S_ff`` the best gain for that
    candidate is ``k_j = S_yf / S_yy`` and the residual ``SSE_j = S_ff - S_yf^2 / S_yy``; the log-likelihood
    is ``-SSE_j / (2 sigma^2)``. Fitting the gain per candidate makes the likelihood depend on the response
    SHAPE only, so a thrust-gain (mass) change is not mistaken for a lag change. Signals are taken
    relative to slowly tracked means so that steady hover carries no lag information (flat posterior).
    """

    def __init__(self, grid: tuple[float, ...], dt: float, noise_std: float, time_constant: float, tau_nominal: float):
        self.grid = np.asarray(grid, dtype=float)
        self.dt = dt
        self.alpha = 1.0 - np.exp(-dt / self.grid)
        self.sigma2 = noise_std**2
        self.forget = np.exp(-dt / time_constant)
        self.y = None
        self.syy = np.zeros_like(self.grid)
        self.syf = np.zeros_like(self.grid)
        self.sff = 0.0
        self.mu_u = None
        self.mu_f = None
        self.sse = np.zeros_like(self.grid)
        self._post = None  # cached posterior (invalidated by update)
        self.log_prior = -0.5 * ((np.log(self.grid) - np.log(tau_nominal)) / 1.0) ** 2  # broad log-normal prior

    def update(self, u_cmd: float, f_meas: float, gain: float) -> None:
        if self.y is None:
            self.y = np.full_like(self.grid, u_cmd)
            self.mu_u, self.mu_f = u_cmd, f_meas
        self.y = self.y + self.alpha * (u_cmd - self.y)
        b = 1.0 - self.forget
        self.mu_u += b * (u_cmd - self.mu_u)
        self.mu_f += b * (f_meas - self.mu_f)
        yc, fc = self.y - self.mu_u, f_meas - self.mu_f
        self.syy = self.forget * self.syy + yc * yc
        self.syf = self.forget * self.syf + yc * fc
        self.sff = self.forget * self.sff + fc * fc
        self.sse = np.maximum(self.sff - self.syf**2 / np.maximum(self.syy, 1e-9), 0.0)
        self._post = None

    @property
    def posterior(self) -> FloatArray:
        if self._post is None:
            logp = self.log_prior - self.sse / (2.0 * self.sigma2)
            logp -= logp.max()
            p = np.exp(logp)
            self._post = p / p.sum()
        return self._post

    @property
    def fit_ratio(self) -> float:
        """Best-candidate residual relative to noise, ``min_j SSE_j / (N_eff sigma^2)`` (~1 if the model fits)."""
        n_eff = 1.0 / (1.0 - self.forget)
        return float(self.sse.min() / (n_eff * self.sigma2))

    @property
    def excitation(self) -> float:
        """Input excitation relative to noise, ``max_j S_yy,j / (N_eff sigma^2)``."""
        n_eff = 1.0 / (1.0 - self.forget)
        return float(self.syy.max() / (n_eff * self.sigma2))

    def valid(self, max_fit: float = 2.0, min_excitation: float = 3.0) -> bool:
        """Lag evidence is usable only if a first-order lag explains the data AND the input was exciting."""
        return self.y is not None and self.fit_ratio <= max_fit and self.excitation >= min_excitation

    @property
    def mean(self) -> float:
        return float(self.posterior @ self.grid)

    @property
    def std(self) -> float:
        p = self.posterior
        m = p @ self.grid
        return float(np.sqrt(max(p @ (self.grid - m) ** 2, 0.0)))

    @property
    def entropy(self) -> float:
        p = self.posterior
        p = p[p > 1e-300]
        return float(-(p * np.log(p)).sum())

    def expected_information_gain(self, u_profile: FloatArray, gain: float) -> float:
        """Expected entropy reduction [nats] if the planned input ``u_profile`` (one value per step) is applied.

        For each hypothetical true ``tau_i`` (weighted by the current posterior) the noise-free outputs of
        every grid filter are compared; the posterior after the probe is ``p_j exp(-D_ij / 2 sigma^2)``.
        Ignoring noise realisations makes this a mild over-estimate (documented).
        """
        p = self.posterior
        y0 = self.y if self.y is not None else np.full_like(self.grid, u_profile[0])
        Y = np.empty((len(self.grid), len(u_profile)))
        y = y0.copy()
        for k, u in enumerate(u_profile):
            y = y + self.alpha * (u - y)
            Y[:, k] = gain * y
        D = ((Y[:, None, :] - Y[None, :, :]) ** 2).sum(axis=2)
        h_post = 0.0
        for i, pi in enumerate(p):
            if pi < 1e-6:
                continue
            logq = np.log(np.maximum(p, 1e-300)) - D[i] / (2.0 * self.sigma2)
            logq -= logq.max()
            q = np.exp(logq)
            q /= q.sum()
            q = q[q > 1e-300]
            h_post += pi * float(-(q * np.log(q)).sum())
        return max(self.entropy - h_post, 0.0)


class DragWindGrid:
    """Joint drag-coefficient / wind estimate for one horizontal world axis.

    Model ``a_ext = -c (v - w)|v - w|`` with ground velocity ``v`` (estimate) and unknown wind ``w``.
    For each grid value ``w_j`` the regressor ``phi_j = -(v - w_j)|v - w_j|`` is linear in ``c``; exponentially
    weighted sums give ``c_j = S_pa / S_pp`` (clipped at 0) and ``SSE_j = S_aa - S_pa^2 / S_pp``.

    Honest uncertainty (identifiability is weak for a heavy, low-drag aircraft at low speed):

    * the noise variance is estimated from the residuals, ``sigma_eff^2 = max(sigma^2, min_j SSE_j / N_eff)``,
      so model mismatch (wind shear, body-axis drag, velocity errors) widens the posterior;
    * a systematic floor ``bias_std / rms(phi)`` accounts for the residual accelerometer bias, which a
      constant-offset model cannot separate from wind;
    * a MAP wind on the grid edge means "not identifiable" (infinite standard deviation).
    """

    def __init__(self, c_nominal: float, noise_std: float, time_constant: float, dt: float, bias_std: float = 0.02,
                 w_grid: FloatArray = np.arange(-12.0, 12.01, 0.5)):
        self.w = np.asarray(w_grid, dtype=float)
        self.forget = float(np.exp(-dt / time_constant))
        self.n_eff = 1.0 / (1.0 - self.forget)
        self.sigma2 = noise_std**2
        self.bias_std = bias_std
        self.c_nom = c_nominal
        self.spp = np.full_like(self.w, 1e-6)
        self.spa = self.spp * c_nominal  # weak prior at the nominal coefficient
        self.saa = 0.0
        self.n = 0
        self._cache: dict | None = None

    def update(self, v: float, a: float) -> None:
        phi = -(v - self.w) * np.abs(v - self.w)
        f = self.forget
        self.spp = f * self.spp + phi * phi
        self.spa = f * self.spa + phi * a
        self.saa = f * self.saa + a * a
        self.n += 1
        self._cache = None

    def _c(self) -> dict:
        """Derived quantities, computed once per update (posterior, c_j, SSE, effective noise)."""
        if self._cache is None:
            cj = np.maximum(self.spa / self.spp, 0.0)
            sse = np.maximum(self.saa - 2.0 * cj * self.spa + cj * cj * self.spp, 0.0)
            n = min(self.n_eff, max(self.n, 1))
            s2 = max(self.sigma2, float(sse.min()) / n)
            logp = -sse / (2.0 * s2)
            logp -= logp.max()
            p = np.exp(logp)
            self._cache = {"cj": cj, "sse": sse, "s2": s2, "post": p / p.sum()}
        return self._cache

    @property
    def _cj(self) -> FloatArray:
        return self._c()["cj"]

    @property
    def _sse(self) -> FloatArray:
        return self._c()["sse"]

    @property
    def _sigma2_eff(self) -> float:
        return self._c()["s2"]

    @property
    def posterior(self) -> FloatArray:
        return self._c()["post"]

    @property
    def _j(self) -> int:
        return int(np.argmax(self.posterior))

    @property
    def c(self) -> float:
        return float(self._cj[self._j])

    @property
    def c_std(self) -> float:
        j = self._j
        if j in (0, len(self.w) - 1):
            return float("inf")
        p = self.posterior
        cj = self._cj
        m = p @ cj
        within = self._sigma2_eff / self.spp
        n = min(self.n_eff, max(self.n, 1))
        rms_phi = float(np.sqrt(self.spp[j] / n))
        floor = self.bias_std / max(rms_phi, 1e-9)
        return float(np.sqrt(max(p @ (within + (cj - m) ** 2), 0.0) + floor**2))

    @property
    def wind(self) -> float:
        return float(self.posterior @ self.w)

    def information_gain(self, v_profile: FloatArray) -> float:
        """Expected reduction of the c-uncertainty entropy [nats] for a planned ground-speed profile."""
        var0 = self.c_std**2
        if not np.isfinite(var0):
            var0 = (3.0 * self.c_nom) ** 2
        phi = -(v_profile - self.wind) * np.abs(v_profile - self.wind)
        add = float(phi @ phi) / self._sigma2_eff
        return 0.5 * float(np.log(1.0 + add * var0))


@dataclass(frozen=True)
class SysIdSnapshot:
    t: float
    thrust_gain: float
    thrust_gain_std: float
    lag: float
    lag_std: float
    lag_valid: bool
    drag_coeff: FloatArray  # (2,) c / m_nom per world horizontal axis [1/m]
    drag_coeff_std: FloatArray
    wind_accel: FloatArray  # (2,) MAP wind velocity per horizontal axis [m/s] (name kept for log compatibility)
    gnss_delay: float
    gnss_delay_std: float
    gnss_delay_r2: float


class OnlineIdentifier:
    def __init__(self, mass_nominal: float, tau_nominal: float, drag_nominal: float, cfg: SysIdConfig = SysIdConfig()):
        self.cfg = cfg
        dt = 1.0 / cfg.rate
        self.m = mass_nominal
        self.tau_nom = tau_nominal
        self.c_nom = drag_nominal / mass_nominal
        lam = lambda tc: float(np.exp(-dt / tc))  # noqa: E731
        self.gain = RecursiveLeastSquares([1.0], [cfg.gain_prior_std**2], cfg.gain_noise_std, lam(cfg.gain_time_constant),
                                          p_max=np.array([0.5**2]))
        self.lag = LagPosterior(cfg.lag_grid, dt, cfg.lag_noise_std, cfg.lag_time_constant, tau_nominal)
        self.drag = [DragWindGrid(self.c_nom, cfg.drag_noise_std, cfg.drag_time_constant, dt) for _ in range(2)]
        self.delay = RecursiveLeastSquares([0.0], [cfg.delay_prior_std**2], cfg.delay_noise_std, 0.99,
                                           p_max=np.array([0.5**2]))
        self._t_model: float | None = None  # lagged collective-thrust model (nominal tau) [N]
        self._last_gnss_t = -np.inf
        self._r2_num = 1e-9  # EMA of squared a-priori regression error
        self._r2_den = 1e-9  # EMA of squared innovation
        self.t = 0.0

    def update(self, t: float, f_body: FloatArray, R: FloatArray, v_est: FloatArray, thrust_cmd: float,
               airborne: bool) -> None:
        """One framework step: ``f_body`` mean bias-corrected specific force since the last call."""
        self.t = t
        dt = 1.0 / self.cfg.rate
        if self._t_model is None:
            self._t_model = thrust_cmd
        self._t_model += (1.0 - np.exp(-dt / self.tau_nom)) * (thrust_cmd - self._t_model)
        if not airborne:
            return  # ground contact forces violate every free-flight model below
        u_model = self._t_model / self.m
        self.gain.update(np.array([u_model]), float(f_body[2]))
        self.lag.update(thrust_cmd / self.m, float(f_body[2]), float(self.gain.theta[0]))
        a_ext = R @ np.array([f_body[0], f_body[1], 0.0])
        for k in range(2):
            self.drag[k].update(float(v_est[k]), float(a_ext[k]))

    def update_gnss_innovation(self, t_meas: float, innovation: FloatArray, v_est: FloatArray,
                               max_innovation: float = 1.5) -> None:
        """Latency regression ``r = -v delta``. Innovations larger than any plausible latency effect
        (``max_innovation``, > 0.5 s at 2 m/s) are corrupted samples, not latency, and are skipped."""
        if t_meas <= self._last_gnss_t or innovation is None:
            return
        self._last_gnss_t = t_meas
        if float(np.max(np.abs(innovation))) > max_innovation:
            return
        for k in range(3):
            err = self.delay.update(np.array([-float(v_est[k])]), float(innovation[k]))
            self._r2_num = 0.99 * self._r2_num + 0.01 * err * err
            self._r2_den = 0.99 * self._r2_den + 0.01 * float(innovation[k]) ** 2

    @property
    def gnss_delay_r2(self) -> float:
        """Fraction of GNSS innovation variance explained by the latency model (noise -> ~0, latency -> ~1)."""
        return float(max(0.0, 1.0 - self._r2_num / self._r2_den))

    def snapshot(self) -> SysIdSnapshot:
        return SysIdSnapshot(
            t=self.t, thrust_gain=float(self.gain.theta[0]), thrust_gain_std=float(self.gain.std[0]),
            lag=self.lag.mean, lag_std=self.lag.std, lag_valid=self.lag.valid(),
            drag_coeff=np.array([d.c for d in self.drag]), drag_coeff_std=np.array([d.c_std for d in self.drag]),
            wind_accel=np.array([d.wind for d in self.drag]),
            gnss_delay=float(self.delay.theta[0]), gnss_delay_std=float(self.delay.std[0]), gnss_delay_r2=self.gnss_delay_r2,
        )
