"""3D facade-coverage mission: plan generation and the runtime mission manager.

Plan (:func:`build_coverage_mission`), all positions in world ENU:

1. ``ground``    rest on the take-off pad (disarmed until ``arm_time``);
2. ``takeoff``   vertical climb to ``takeoff_height`` above the pad;
3. ``transit``   to the first pass start at the stand-off distance;
4. ``coverage``  horizontal passes along the facade (boustrophedon) joined by vertical transitions,
   inset ``x_inset`` from the facade edges;
5. ``exit``      retreat to ``standoff + exit_extra`` from the facade, fly back above the pad;
6. ``landing``   descend, slow final descent, touchdown target below the resting height.

Every segment is a rest-to-rest quintic (C2). Its duration is the smallest that satisfies the speed
and acceleration limits per axis group (quintic peak speed ``1.875 d / T``, peak acceleration
``5.7735 d / T^2``), so the reference respects the limits BY CONSTRUCTION (hard constraint, not a
penalty). Yaw is constant, facing the facade.

The :class:`MissionManager` serves references and supports the recovery actions: re-timing
(speed scaling), stand-off shift, truncation of remaining passes, abort along the safe exit route,
landing in place, and insertion of bounded diagnostic probes (dwell segments). Any replanned
waypoint is projected to respect the minimum stand-off (``FacadePlane`` distance).
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from graduation_project.active_diagnosis.probes import Probe
from graduation_project.dynamics.frames import FloatArray
from graduation_project.planning.trajectories import QuinticSegment, RefPoint
from graduation_project.sensors.models import FacadePlane

PHASES = ("ground", "takeoff", "transit", "coverage", "probe", "exit", "landing", "landed", "hold")
QUINTIC_V = 1.875
QUINTIC_A = 10.0 * np.sqrt(3.0) / 3.0  # 5.7735


@dataclass(frozen=True)
class MotionLimits:
    cruise_speed: float = 1.5  # coverage passes [m/s]
    transit_speed: float = 2.0  # [m/s]
    climb_speed: float = 1.0  # [m/s]
    descent_speed: float = 0.8  # [m/s]
    final_descent_speed: float = 0.3  # [m/s]
    max_accel: float = 1.0  # [m/s^2]
    min_segment_time: float = 1.0  # [s]


@dataclass(frozen=True)
class CoverageConfig:
    standoff: float = 3.0  # [m]
    z_min: float = 3.0
    z_max: float = 9.0
    n_passes: int = 3
    x_inset: float = 1.0  # [m] keeps the vehicle envelope away from facade edges / neighbouring blocks
    takeoff_height: float = 2.5  # [m] above the pad
    pad_xy: tuple[float, float] = (0.0, 0.0)
    exit_extra: float = 1.0  # [m] extra stand-off on the exit route
    arm_time: float = 1.0  # [s]
    touchdown_overshoot: float = 0.15  # [m] reference below resting height for firm touchdown


@dataclass
class Waypoint:
    position: FloatArray
    phase: str
    tag: str = "nominal"  # energy/accounting tag: nominal | diagnostic | recovery
    coverage: bool = False  # the segment ARRIVING at this waypoint is a coverage pass
    speed: str = "transit"  # cruise | transit | vertical | final
    probe: Probe | None = None  # dwell with probe after arrival


@dataclass
class Segment:
    t0: float
    duration: float
    p0: FloatArray
    p1: FloatArray
    phase: str
    tag: str
    coverage: bool
    quintic: QuinticSegment | None = None  # None -> dwell at p0
    probe: Probe | None = None

    @property
    def t1(self) -> float:
        return self.t0 + self.duration


def facade_yaw(facade: FacadePlane) -> float:
    n = facade.normal
    return float(np.arctan2(-n[1], -n[0]))


def build_coverage_mission(facade: FacadePlane, cfg: CoverageConfig, z_rest: float) -> list[Waypoint]:
    o, n, tg = facade.origin, facade.normal, facade.tangent
    pad = np.array([cfg.pad_xy[0], cfg.pad_xy[1], z_rest])
    up = np.array([0.0, 0.0, 1.0])
    wps = [Waypoint(pad.copy(), "ground"), Waypoint(pad + cfg.takeoff_height * up, "takeoff", speed="vertical")]
    levels = np.linspace(cfg.z_min, cfg.z_max, cfg.n_passes)
    x0, x1 = cfg.x_inset, facade.width - cfg.x_inset
    base = o + cfg.standoff * n
    first = True
    for k, z in enumerate(levels):
        a, b = base + x0 * tg + z * up, base + x1 * tg + z * up
        if k % 2:
            a, b = b, a
        wps.append(Waypoint(a, "transit" if first else "coverage", speed="transit" if first else "vertical"))
        wps.append(Waypoint(b, "coverage", coverage=True, speed="cruise"))
        first = False
    wps += exit_route(wps[-1].position, facade, cfg, z_rest, tag="nominal")
    return wps


def exit_route(p: FloatArray, facade: FacadePlane, cfg: CoverageConfig, z_rest: float, tag: str) -> list[Waypoint]:
    """Safe exit from ``p``: retreat from the facade, fly above the pad, descend and land."""
    n = facade.normal
    d = facade.distance(p)
    retreat = p + max(0.0, cfg.standoff + cfg.exit_extra - d) * n
    z_cruise = max(float(retreat[2]), z_rest + cfg.takeoff_height)
    retreat = np.array([retreat[0], retreat[1], z_cruise])
    above_pad = np.array([cfg.pad_xy[0], cfg.pad_xy[1], z_cruise])
    approach = np.array([cfg.pad_xy[0], cfg.pad_xy[1], z_rest + 0.6])
    touch = np.array([cfg.pad_xy[0], cfg.pad_xy[1], z_rest - cfg.touchdown_overshoot])
    return [Waypoint(retreat, "exit", tag), Waypoint(above_pad, "exit", tag), Waypoint(approach, "landing", tag, speed="vertical"),
            Waypoint(touch, "landing", tag, speed="final")]


def segment_duration(p0: FloatArray, p1: FloatArray, speed_kind: str, lim: MotionLimits, scale: float = 1.0) -> float:
    d = np.asarray(p1) - np.asarray(p0)
    dh, dz = float(np.linalg.norm(d[:2])), float(d[2])
    v_h = {"cruise": lim.cruise_speed, "transit": lim.transit_speed}.get(speed_kind, lim.transit_speed) * scale
    v_z = (lim.climb_speed if dz > 0 else lim.descent_speed) * scale
    if speed_kind == "final":
        v_z = lim.final_descent_speed * scale
    a = lim.max_accel * scale * scale
    dist = float(np.linalg.norm(d))
    T = max(lim.min_segment_time, QUINTIC_V * dh / v_h, QUINTIC_V * abs(dz) / v_z, np.sqrt(QUINTIC_A * dist / a))
    return float(T)


@dataclass
class MissionManager:
    waypoints: list[Waypoint]
    facade: FacadePlane
    cfg: CoverageConfig
    limits: MotionLimits
    z_rest: float
    min_standoff: float  # vehicle-size-aware minimum distance of any replanned waypoint from the facade
    yaw: float = 0.0
    speed_scale: float = 1.0
    segments: list[Segment] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.yaw = facade_yaw(self.facade)
        start = self.waypoints[0].position
        self.segments = [Segment(0.0, self.cfg.arm_time, start, start, "ground", "nominal", False)]
        self.segments += self._build(self.cfg.arm_time, start, self.waypoints[1:])
        self.planned_coverage_length = sum(np.linalg.norm(s.p1 - s.p0) for s in self.segments if s.coverage)
        self.covered_length = 0.0
        self._last_ref: RefPoint | None = None
        self._last_t = 0.0
        self.aborted = False
        self.terminated = False
        self.landed = False
        self.disarmed = False
        self.pending_probe: Probe | None = None
        self.events: list[dict] = []

    # ------------------------------------------------------------------ construction
    def _build(self, t0: float, p0: FloatArray, wps: list[Waypoint], v0: FloatArray | None = None,
               a0: FloatArray | None = None) -> list[Segment]:
        segs: list[Segment] = []
        t, p = t0, np.asarray(p0, dtype=float)
        for i, w in enumerate(wps):
            T = segment_duration(p, w.position, w.speed, self.limits, self.speed_scale)
            if i == 0 and v0 is not None and np.linalg.norm(v0) > 1e-6:
                T = max(T, 2.5 * float(np.linalg.norm(v0)) / max(self.limits.max_accel, 1e-6))
                q = QuinticSegment(p, w.position, T, v0=v0, a0=a0)
            else:
                q = QuinticSegment(p, w.position, T)
            segs.append(Segment(t, T, p.copy(), w.position.copy(), w.phase, w.tag, w.coverage, q))
            t += T
            p = w.position
            if w.probe is not None:
                segs.append(Segment(t, w.probe.duration, p.copy(), p.copy(), "probe", "diagnostic", False, None, w.probe))
                t += w.probe.duration
        return segs

    def _project_standoff(self, p: FloatArray) -> FloatArray:
        d = self.facade.distance(p)
        if d < self.min_standoff and self.facade.in_front(p - d * self.facade.normal):
            return p + (self.min_standoff - d) * self.facade.normal
        return p

    # ------------------------------------------------------------------ queries
    def current_index(self, t: float) -> int:
        for i, s in enumerate(self.segments):
            if t < s.t1:
                return i
        return len(self.segments) - 1

    def current(self, t: float) -> Segment:
        return self.segments[self.current_index(t)]

    @property
    def end_time(self) -> float:
        return self.segments[-1].t1

    def phase(self, t: float) -> str:
        if self.landed:
            return "landed"
        if t >= self.end_time:
            return "hold" if not self.segments[-1].phase == "landing" else "landing"
        return self.current(t).phase

    def tag(self, t: float) -> str:
        if t >= self.end_time:
            return self.segments[-1].tag
        return self.current(t).tag

    def remaining_time(self, t: float) -> float:
        return max(0.0, self.end_time - t)

    def coverage_fraction(self) -> float:
        return min(1.0, self.covered_length / max(self.planned_coverage_length, 1e-9))

    def remaining_coverage_time(self, t: float) -> float:
        return sum(max(0.0, s.t1 - max(t, s.t0)) for s in self.segments if s.coverage)

    def exit_time_estimate(self, p: FloatArray) -> float:
        """Duration of the safe exit route from ``p`` (energy for safe termination)."""
        wps = exit_route(np.asarray(p, dtype=float), self.facade, self.cfg, self.z_rest, "recovery")
        T, q = 0.0, np.asarray(p, dtype=float)
        for w in wps:
            T += segment_duration(q, w.position, w.speed, self.limits, 1.0)
            q = w.position
        return T

    def reference(self, t: float) -> RefPoint:
        seg = self.current(t)
        if t >= self.end_time:
            seg = self.segments[-1]
            ref = RefPoint(seg.p1.copy(), np.zeros(3), np.zeros(3), self.yaw)
        elif seg.quintic is None:
            dp, dv, da = (seg.probe.offset(t - seg.t0) if seg.probe is not None else (np.zeros(3),) * 3)
            ref = RefPoint(seg.p0 + dp, dv, da, self.yaw)
        else:
            p, v, a = seg.quintic.sample_pva(t - seg.t0)
            ref = RefPoint(p, v, a, self.yaw)
        if self._last_ref is not None and seg.coverage and t > self._last_t:
            self.covered_length += float(np.linalg.norm(ref.position - self._last_ref.position))
        self._last_ref, self._last_t = ref, t
        return ref

    # ------------------------------------------------------------------ actions
    def _future_waypoints(self, i: int) -> list[Waypoint]:
        """Waypoints of segments after index ``i`` (dwell segments re-attached as probes)."""
        out: list[Waypoint] = []
        for s in self.segments[i + 1:]:
            if s.quintic is None:
                if out and s.probe is not None:
                    out[-1].probe = s.probe
                continue
            kind = "cruise" if s.coverage else ("final" if s.phase == "landing" and s.p1[2] < self.z_rest
                                                else ("vertical" if np.linalg.norm(s.p1[:2] - s.p0[:2]) < 1e-6 else "transit"))
            out.append(Waypoint(s.p1.copy(), s.phase, s.tag, s.coverage, kind))
        return out

    def _replace_future(self, t: float, wps: list[Waypoint], immediate: bool, tag_override: str | None = None) -> None:
        i = self.current_index(t)
        if tag_override:
            wps = [replace(w, tag=tag_override) for w in wps]
        if immediate:
            ref = self._last_ref if self._last_ref is not None else self.reference(t)
            new = self._build(t, ref.position, wps, v0=ref.velocity, a0=ref.acceleration)
            cur = self.segments[i]
            cur.duration = max(0.0, t - cur.t0)
            self.segments = self.segments[: i + 1] + new
        else:
            cur = self.segments[i]
            self.segments = self.segments[: i + 1] + self._build(cur.t1, cur.p1, wps)

    def set_speed_scale(self, t: float, scale: float) -> None:
        scale = float(np.clip(scale, 0.3, 1.0))
        if abs(scale - self.speed_scale) < 1e-9:
            return
        self.speed_scale = scale
        self._replace_future(t, self._future_waypoints(self.current_index(t)), immediate=False)
        self.events.append({"t": t, "event": "speed_scale", "value": scale})

    def shift_standoff(self, t: float, delta: float) -> None:
        i = self.current_index(t)
        wps = self._future_waypoints(i)
        for w in wps:
            if w.phase in ("coverage", "transit"):
                w.position = self._project_standoff(w.position + delta * self.facade.normal)
                w.tag = "recovery"
        self._replace_future(t, wps, immediate=False)
        self.events.append({"t": t, "event": "standoff_shift", "value": delta})

    def truncate_coverage(self, t: float, keep_coverage_time: float) -> int:
        """Drop remaining coverage passes beyond ``keep_coverage_time`` seconds of coverage; returns passes dropped."""
        i = self.current_index(t)
        future = self.segments[i + 1:]
        kept_t, cut_at = 0.0, None
        for j, s in enumerate(future):
            if s.coverage:
                if kept_t + s.duration > keep_coverage_time:
                    cut_at = j
                    break
                kept_t += s.duration
        if cut_at is None:
            return 0
        dropped = sum(1 for s in future[cut_at:] if s.coverage)
        last = future[cut_at - 1].p1 if cut_at > 0 else self.segments[i].p1
        keep_wps = self._future_waypoints(i)[:cut_at]
        keep_wps += exit_route(last, self.facade, self.cfg, self.z_rest, "recovery")
        self._replace_future(t, keep_wps, immediate=False)
        self.events.append({"t": t, "event": "truncate_coverage", "value": dropped})
        return dropped

    def abort(self, t: float, reason: str) -> None:
        if self.aborted or self.terminated:
            return
        p = (self._last_ref.position if self._last_ref is not None else self.segments[self.current_index(t)].p0)
        self._replace_future(t, exit_route(p, self.facade, self.cfg, self.z_rest, "recovery"), immediate=True)
        self.aborted = True
        self.events.append({"t": t, "event": "abort_to_exit", "reason": reason})

    def land_in_place(self, t: float, reason: str, retreat: bool = True) -> None:
        """Safe termination: (optionally) retreat to a safe stand-off, then descend vertically and land."""
        if self.terminated:
            return
        p = self._last_ref.position if self._last_ref is not None else self.segments[self.current_index(t)].p0
        d = self.facade.distance(p)
        safe = p + (max(0.0, self.min_standoff + 0.5 - d) if retreat else 0.0) * self.facade.normal
        wps = [Waypoint(safe, "exit", "recovery"),
               Waypoint(np.array([safe[0], safe[1], self.z_rest + 0.6]), "landing", "recovery", speed="vertical"),
               Waypoint(np.array([safe[0], safe[1], self.z_rest - self.cfg.touchdown_overshoot]), "landing", "recovery", speed="final")]
        self._replace_future(t, wps, immediate=True)
        self.terminated = True
        self.events.append({"t": t, "event": "land_in_place", "reason": reason})

    def insert_probe(self, t: float, probe: Probe) -> float:
        """Insert a dwell + probe at the end of the current segment; returns its start time."""
        i = self.current_index(t)
        cur = self.segments[i]
        if cur.quintic is None and cur.probe is None and cur.phase != "ground":
            t_start = t  # already holding: start now
            cur.duration = t - cur.t0
        else:
            t_start = cur.t1
        dwell = Segment(t_start, probe.duration, cur.p1.copy(), cur.p1.copy(), "probe", "diagnostic", False, None, probe)
        shift = probe.duration
        for s in self.segments[i + 1:]:
            s.t0 += shift
        self.segments = self.segments[: i + 1] + [dwell] + self.segments[i + 1:]
        self.events.append({"t": t, "event": "probe_inserted", "kind": probe.kind, "t_start": t_start})
        return t_start

    def probe_active(self, t: float) -> bool:
        return self.current(t).phase == "probe" and t < self.end_time

    def coverage_active_ahead(self, t: float) -> bool:
        return any(s.coverage for s in self.segments[self.current_index(t):])
