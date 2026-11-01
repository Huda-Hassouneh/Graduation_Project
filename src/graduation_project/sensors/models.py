"""Seeded sensor models with latency, noise, bias drift and injectable faults.

The suite turns TRUE simulated quantities into the measurements a real aircraft would provide.
Nothing downstream of :class:`SensorSuite` (estimator, controller, research framework) receives
truth. Sensors and rates (parameters: ``simulation_assumptions.sensors`` in the aircraft spec):

=============  ===========================================================  ======================
sensor         measurement                                                  model
=============  ===========================================================  ======================
imu            specific force f_B and body rate omega_B at the IMU rate     white noise + bias random walk
                                                                             + vibration noise
gnss_pos       world position, reported with its time of validity           Gaussian, latency
gnss_vel       world velocity                                               Gaussian, latency
baro           altitude above the take-off point (from ISA pressure)        Gaussian + bias random walk
heading        yaw (dual-antenna RTK heading)                               Gaussian, wrapped
range          perpendicular distance to the mapped facade plane            Gaussian; valid 0.4-21 m and
                                                                             only in front of the facade
battery        terminal voltage, current, SoC (quantised)                   Gaussian / quantisation
=============  ===========================================================  ======================

Faults (:class:`SensorFault`) act on one sensor in a time window:

* ``noise``    multiply the noise standard deviation by ``magnitude``;
* ``bias``     add ``magnitude`` (scalar or vector) to the measurement;
* ``delay``    UNREPORTED extra latency: content is ``magnitude`` s older than its timestamp;
* ``dropout``  no measurement is produced;
* ``invalid``  with probability ``probability`` a sample is NaN, otherwise an outlier of size
  ``magnitude`` (alternating), mimicking corrupted packets.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from graduation_project.dynamics.frames import FloatArray, euler_zyx_from_quat
from graduation_project.environment.atmosphere import Site, pressure_altitude

SENSORS = ("imu", "gnss_pos", "gnss_vel", "baro", "heading", "range", "battery")
FAULT_KINDS = ("noise", "bias", "delay", "dropout", "invalid")


@dataclass(frozen=True)
class SensorParams:
    imu_rate: float
    gyro_noise_density: float
    gyro_bias_rw: float
    accel_noise_density: float
    accel_bias_rw: float
    vibration_accel_std: float
    gnss_rate: float
    gnss_latency: float
    gnss_pos_std: FloatArray  # (3,) [m]
    gnss_vel_std: float
    baro_rate: float
    baro_std: float
    baro_bias_rw: float
    heading_rate: float
    heading_std: float
    range_rate: float
    range_std: float
    range_min: float
    range_max: float
    battery_rate: float
    battery_voltage_std: float
    battery_current_std: float
    battery_soc_quantum: float

    @classmethod
    def from_spec(cls, spec: dict[str, Any]) -> "SensorParams":
        from graduation_project.aircraft_spec import sval

        s = lambda k: float(sval(spec, f"simulation_assumptions.sensors.{k}"))  # noqa: E731
        h, v = s("gnss.position_std_horizontal"), s("gnss.position_std_vertical")
        return cls(
            imu_rate=s("imu.rate"), gyro_noise_density=s("imu.gyro_noise_density"), gyro_bias_rw=s("imu.gyro_bias_rw"),
            accel_noise_density=s("imu.accel_noise_density"), accel_bias_rw=s("imu.accel_bias_rw"),
            vibration_accel_std=s("imu.vibration_accel_std"),
            gnss_rate=s("gnss.rate"), gnss_latency=s("gnss.latency"), gnss_pos_std=np.array([h, h, v]),
            gnss_vel_std=s("gnss.velocity_std"),
            baro_rate=s("baro.rate"), baro_std=s("baro.altitude_std"), baro_bias_rw=s("baro.bias_rw"),
            heading_rate=s("heading.rate"), heading_std=s("heading.std"),
            range_rate=s("facade_range.rate"), range_std=s("facade_range.std"), range_min=s("facade_range.min_range"),
            range_max=s("facade_range.max_range"),
            battery_rate=s("battery_telemetry.rate"), battery_voltage_std=s("battery_telemetry.voltage_std"),
            battery_current_std=s("battery_telemetry.current_std"), battery_soc_quantum=s("battery_telemetry.soc_quantum"),
        )


@dataclass(frozen=True)
class SensorFault:
    sensor: str
    kind: str
    t_start: float
    t_end: float = np.inf
    magnitude: Any = 0.0
    probability: float = 1.0

    def __post_init__(self) -> None:
        if self.sensor not in SENSORS or self.kind not in FAULT_KINDS:
            raise ValueError(f"unknown sensor fault {self.sensor}/{self.kind}")

    def active(self, t: float) -> bool:
        return self.t_start <= t < self.t_end


@dataclass(frozen=True)
class ImuSample:
    t: float
    accel: FloatArray  # specific force, body frame [m/s^2]
    gyro: FloatArray  # body rate [rad/s]


@dataclass(frozen=True)
class Measurement:
    sensor: str
    t: float  # reported time of validity [s]
    value: FloatArray


@dataclass(frozen=True)
class FacadePlane:
    """Planar facade face: ``origin`` on the wall, ``normal`` outward (toward the flight side), and its
    horizontal extent along ``tangent`` (``[0, width]`` from origin) and height ``[0, height]``."""

    origin: FloatArray
    normal: FloatArray
    tangent: FloatArray
    width: float
    height: float

    def distance(self, p: FloatArray) -> float:
        return float((np.asarray(p) - self.origin) @ self.normal)

    def in_front(self, p: FloatArray) -> bool:
        s = float((np.asarray(p) - self.origin) @ self.tangent)
        return 0.0 <= s <= self.width and 0.0 <= p[2] <= self.height


@dataclass
class TruthSample:
    t: float
    position: FloatArray
    velocity: FloatArray
    quaternion: FloatArray


@dataclass
class _Schedule:
    period: float
    next_t: float = 0.0


@dataclass
class SensorSuite:
    params: SensorParams
    seed: int
    faults: tuple[SensorFault, ...] = ()
    facade: FacadePlane | None = None
    site: Site = field(default_factory=Site)
    history_s: float = 3.0
    _rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        p = self.params
        self._gyro_bias = 0.002 * self._rng.standard_normal(3)  # turn-on bias [rad/s]
        self._accel_bias = 0.02 * self._rng.standard_normal(3)  # turn-on bias [m/s^2]
        self._baro_bias = 0.0
        self._sched = {"gnss": _Schedule(1.0 / p.gnss_rate), "baro": _Schedule(1.0 / p.baro_rate),
                       "heading": _Schedule(1.0 / p.heading_rate), "range": _Schedule(1.0 / p.range_rate),
                       "battery": _Schedule(1.0 / p.battery_rate)}
        self._pending: list[tuple[float, Measurement]] = []
        self._hist: deque[TruthSample] = deque()
        self._last_imu_t: float | None = None
        self._invalid_toggle = 0

    # ------------------------------------------------------------------ fault helpers
    def _faults(self, sensor: str, t: float) -> list[SensorFault]:
        return [f for f in self.faults if f.sensor == sensor and f.active(t)]

    def _noise_scale(self, sensor: str, t: float) -> float:
        s = 1.0
        for f in self._faults(sensor, t):
            if f.kind == "noise":
                s *= float(f.magnitude)
        return s

    def _apply_value_faults(self, sensor: str, t: float, value: FloatArray) -> FloatArray | None:
        out = np.array(value, dtype=float)
        for f in self._faults(sensor, t):
            if f.kind == "dropout":
                return None
            if f.kind == "bias":
                out = out + np.asarray(f.magnitude, dtype=float)
            if f.kind == "invalid" and self._rng.random() < f.probability:
                self._invalid_toggle ^= 1
                out = np.full_like(out, np.nan) if self._invalid_toggle else out + float(f.magnitude)
        return out

    def _extra_delay(self, sensor: str, t: float) -> float:
        return sum(float(f.magnitude) for f in self._faults(sensor, t) if f.kind == "delay")

    def _truth_at(self, t: float) -> TruthSample:
        """Most recent stored truth sample at or before ``t`` (for delayed content)."""
        for s in reversed(self._hist):  # newest first: almost every query is for the latest sample
            if s.t <= t + 1e-12:
                return s
        return self._hist[0]

    # ------------------------------------------------------------------ IMU
    def imu(self, t: float, specific_force_b: FloatArray, omega_b: FloatArray) -> ImuSample:
        p = self.params
        dt = 1.0 / p.imu_rate if self._last_imu_t is None else max(t - self._last_imu_t, 1e-6)
        self._last_imu_t = t
        self._gyro_bias = self._gyro_bias + p.gyro_bias_rw * np.sqrt(dt) * self._rng.standard_normal(3)
        self._accel_bias = self._accel_bias + p.accel_bias_rw * np.sqrt(dt) * self._rng.standard_normal(3)
        k = self._noise_scale("imu", t)
        gyro = omega_b + self._gyro_bias + k * p.gyro_noise_density * np.sqrt(p.imu_rate) * self._rng.standard_normal(3)
        acc_std = np.hypot(p.accel_noise_density * np.sqrt(p.imu_rate), p.vibration_accel_std)
        accel = specific_force_b + self._accel_bias + k * acc_std * self._rng.standard_normal(3)
        for f in self._faults("imu", t):
            if f.kind == "bias":
                m = np.asarray(f.magnitude, dtype=float)
                accel = accel + (m[:3] if m.size >= 3 else m)
                gyro = gyro + (m[3:6] if m.size >= 6 else 0.0)
        return ImuSample(t, accel, gyro)

    # ------------------------------------------------------------------ aiding sensors
    def poll(self, t: float, truth: TruthSample, battery: tuple[float, float, float] | None = None) -> list[Measurement]:
        """Record truth, generate due measurements, return those that have ARRIVED by ``t``.

        ``battery`` is (terminal voltage [V], current [A], SoC [-]) of the simulated battery.
        """
        p = self.params
        self._hist.append(truth)
        while self._hist and self._hist[0].t < t - self.history_s:
            self._hist.popleft()
        rng = self._rng

        for name, sch in self._sched.items():
            while sch.next_t <= t + 1e-9:
                tk = sch.next_t
                sch.next_t += sch.period
                if name == "gnss":
                    src = self._truth_at(tk - self._extra_delay("gnss_pos", tk))
                    k = self._noise_scale("gnss_pos", tk)
                    pos = src.position + k * p.gnss_pos_std * rng.standard_normal(3)
                    pos = self._apply_value_faults("gnss_pos", tk, pos)
                    if pos is not None:
                        self._pending.append((tk + p.gnss_latency, Measurement("gnss_pos", tk, pos)))
                    srcv = self._truth_at(tk - self._extra_delay("gnss_vel", tk))
                    vel = srcv.velocity + self._noise_scale("gnss_vel", tk) * p.gnss_vel_std * rng.standard_normal(3)
                    vel = self._apply_value_faults("gnss_vel", tk, vel)
                    if vel is not None:
                        self._pending.append((tk + p.gnss_latency, Measurement("gnss_vel", tk, vel)))
                elif name == "baro":
                    src = self._truth_at(tk - self._extra_delay("baro", tk))
                    self._baro_bias += p.baro_bias_rw * np.sqrt(sch.period) * rng.standard_normal()
                    # pressure at the true height, converted back exactly as a barometer would
                    h = pressure_altitude(self.site.at(src.position[2]).pressure) - pressure_altitude(self.site.at(0.0).pressure)
                    alt = h + self._baro_bias + self._noise_scale("baro", tk) * p.baro_std * rng.standard_normal()
                    val = self._apply_value_faults("baro", tk, np.array([alt]))
                    if val is not None:
                        self._pending.append((tk, Measurement("baro", tk, val)))
                elif name == "heading":
                    src = self._truth_at(tk - self._extra_delay("heading", tk))
                    yaw = euler_zyx_from_quat(src.quaternion)[2]  # [roll, pitch, yaw]
                    yaw += self._noise_scale("heading", tk) * p.heading_std * rng.standard_normal()
                    val = self._apply_value_faults("heading", tk, np.array([np.arctan2(np.sin(yaw), np.cos(yaw))]))
                    if val is not None:
                        self._pending.append((tk + p.gnss_latency, Measurement("heading", tk, val)))
                elif name == "range":
                    if self.facade is None:
                        continue
                    src = self._truth_at(tk - self._extra_delay("range", tk))
                    d = self.facade.distance(src.position)
                    if not (self.facade.in_front(src.position) and p.range_min <= d <= p.range_max):
                        continue  # no return
                    d += self._noise_scale("range", tk) * p.range_std * rng.standard_normal()
                    val = self._apply_value_faults("range", tk, np.array([d]))
                    if val is not None:
                        self._pending.append((tk, Measurement("range", tk, val)))
                elif name == "battery" and battery is not None:
                    v, i, soc = battery
                    q = p.battery_soc_quantum
                    meas = np.array([v + p.battery_voltage_std * rng.standard_normal(),
                                     i + p.battery_current_std * rng.standard_normal(),
                                     np.floor(soc / q + 1e-9) * q])
                    val = self._apply_value_faults("battery", tk, meas)
                    if val is not None:
                        self._pending.append((tk, Measurement("battery", tk, val)))

        arrived = [m for (ta, m) in self._pending if ta <= t + 1e-9]
        self._pending = [(ta, m) for (ta, m) in self._pending if ta > t + 1e-9]
        return arrived
