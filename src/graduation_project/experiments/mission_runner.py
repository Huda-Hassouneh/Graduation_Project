"""Closed-loop mission simulation: plant + battery + wind + sensors + ESKF + controller + framework + mission.

Information flow (per 4 ms control step)::

    truth plant --(specific force, rates)--> IMU --> ESKF.predict
    truth plant --(position, velocity, attitude)--> GNSS/baro/heading/range --> ESKF.update
    truth battery --(V, I, SoC)--> battery telemetry -----------------------------+
    ESKF estimate + mission reference --> controller --> rotor commands --> plant  |
    every 5th step: FrameworkInputs (estimate, IMU means, own commands, telemetry) <+
                    --> ResearchFramework --> mission/controller actions

Truth reaches only the plant, the sensors and the evaluation log (columns prefixed ``true_``-like
quantities: ``p_*``, ``pos_err``, ``standoff``, ``clearance``, ``energy_true``...). The framework never
receives the scenario's fault schedule or labels.

Take-off and landing use the ground-contact model; the run ends 2 s after a detected landing and
disarm (estimate-based detection), at ``duration_max``, or on a crash (hard ground impact, tilt above
70 deg, or the vehicle envelope intersecting a building solid).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from graduation_project.aircraft_spec import load_spec, sval
from graduation_project.config import CONFIG_DIR, AircraftParams, load_aircraft
from graduation_project.controllers.dob import DisturbanceObserverController, load_dob_params
from graduation_project.controllers.pid import CascadedPIDController, load_pid_gains
from graduation_project.dynamics.contact import GroundContactParams
from graduation_project.dynamics.frames import euler_zyx_from_quat, rotation_angle_between
from graduation_project.dynamics.mass_properties import MassProperties
from graduation_project.dynamics.multirotor import MultirotorPlant, StateValidationError
from graduation_project.actuators.rotor import RotorHealth
from graduation_project.energy.battery import Battery, BatteryParams
from graduation_project.energy.power import electrical_power, shaft_power
from graduation_project.estimation.eskf import ErrorStateEKF
from graduation_project.experiments.mission_scenarios import MissionScenario
from graduation_project.experiments.scenarios import FACADE_NORMAL, FACADE_ORIGIN, FACADE_TANGENT, FACADE_WIDTH_M
from graduation_project.framework.core import FrameworkConfig, FrameworkInputs, ResearchFramework
from graduation_project.gazebo.facade_world import FacadeWorldSpec, build_layout
from graduation_project.planning.mission import PHASES, CoverageConfig, MissionManager, MotionLimits, build_coverage_mission
from graduation_project.sensors.models import FacadePlane, SensorParams, SensorSuite, TruthSample

CONTROLLERS = ("pid", "dob")
TAGS = ("nominal", "diagnostic", "recovery", "termination")
CAUSE_CODES = {None: 0, "wind_gust": 1, "steady_wind": 2, "drag_change": 3, "mass_or_thrust_change": 4,
               "actuator_degradation_or_com_shift": 5, "response_delay": 6, "gnss_delay": 7, "sensor_dropout": 8,
               "invalid_measurements": 9, "sensor_inconsistency": 10, "unexplained": 11}


@dataclass(frozen=True)
class SafetyLimits:
    """Evaluation envelope (references are designed well inside it)."""

    max_horizontal_speed: float = 3.0  # [m/s]
    max_vertical_speed: float = 2.0  # [m/s]
    max_acceleration: float = 3.0  # [m/s^2]
    min_clearance: float = 0.5  # [m] vehicle envelope to any building solid
    geofence_min: tuple[float, float, float] = (-10.0, -10.0, -0.5)
    geofence_max: tuple[float, float, float] = (22.0, 7.8, 20.0)


@dataclass
class MissionRun:
    scenario: MissionScenario
    controller: str
    framework: FrameworkConfig
    columns: list[str]
    data: np.ndarray
    events: list[dict]
    status: str  # completed | landed_early | timeout | crash | failed
    info: dict[str, Any] = field(default_factory=dict)

    def col(self, name: str) -> np.ndarray:
        return self.data[:, self.columns.index(name)]

    def cols(self, prefix: str, names: str = "xyz") -> np.ndarray:
        return np.column_stack([self.col(f"{prefix}_{c}") for c in names])


def facade_plane() -> FacadePlane:
    return FacadePlane(np.asarray(FACADE_ORIGIN, float), np.asarray(FACADE_NORMAL, float), np.asarray(FACADE_TANGENT, float),
                       FACADE_WIDTH_M, 12.0)


def building_boxes(variant: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    lay = build_layout(FacadeWorldSpec(name="eval", variant=variant, aircraft=None))
    boxes = [lay.world_box(b) for b in lay.boxes if b.kind in ("solid", "railing")]
    return (np.array([b["centre"] for b in boxes]), 0.5 * np.array([b["size"] for b in boxes]), [b["name"] for b in boxes])


def clearance(p: np.ndarray, centres: np.ndarray, halves: np.ndarray, radius: float) -> float:
    d = np.linalg.norm(np.maximum(np.abs(p - centres) - halves, 0.0), axis=1)
    return float(d.min() - radius)


def contact_params(spec: dict, ac: AircraftParams) -> GroundContactParams:
    com_z = float(sval(spec, "geometry.rotor_height")) - float(ac.rotor.positions[0, 2])  # CoM height in frame G
    bz = float(sval(spec, "geometry.landing_gear.bottom_z")) - com_z
    tr, sl = float(sval(spec, "geometry.landing_gear.track")), float(sval(spec, "geometry.landing_gear.skid_length"))
    pts = np.array([[sx * 0.5 * sl, sy * 0.5 * tr, bz] for sx in (1, -1) for sy in (1, -1)])
    gc = "simulation_assumptions.ground_contact"
    return GroundContactParams.from_damping_ratio(pts, ac.mass, float(sval(spec, f"{gc}.stiffness")),
                                                 float(sval(spec, f"{gc}.damping_ratio")), float(sval(spec, f"{gc}.friction_coefficient")))


def make_controller(kind: str, ac: AircraftParams):
    gains = load_pid_gains(ac.name)
    if kind == "pid":
        return CascadedPIDController(ac, gains)
    if kind == "dob":
        return DisturbanceObserverController(ac, gains, load_dob_params(ac.name))
    raise ValueError(f"unknown controller {kind!r}")


def _apply_event(plant: MultirotorPlant, ev, ac: AircraftParams) -> None:
    tr = plant.truth
    if ev.type == "mass_scale":
        mp = ac.mass_properties
        tr.mass_properties = MassProperties(mp.mass * ev.value, mp.inertia * ev.value)
    elif ev.type == "drag_scale":
        tr.drag_scale = ev.value
    elif ev.type == "command_delay":
        tr.command_delay_s = ev.value
    elif ev.type == "rotor_effectiveness":
        eff = tr.health.effectiveness.copy()
        eff[int(ev.rotor) - 1] = ev.value
        tr.health = RotorHealth(eff, tr.health.lag_factor.copy())
    else:
        raise ValueError(f"unknown truth event {ev.type}")


LOG_COLUMNS = (["t", "p_x", "p_y", "p_z", "v_x", "v_y", "v_z", "roll", "pitch", "yaw", "q_w", "q_x", "q_y", "q_z",
                "est_p_x", "est_p_y", "est_p_z", "est_v_x", "est_v_y", "est_v_z", "est_yaw",
                "ref_p_x", "ref_p_y", "ref_p_z", "ref_v_x", "ref_v_y", "ref_v_z",
                "pos_err", "est_err", "vel_err", "att_err", "standoff", "standoff_ref", "clearance",
                "acc_x", "acc_y", "acc_z", "thrust_cmd", "power_true", "energy_true", "batt_v", "batt_soc",
                "phase", "tag", "speed_scale", "probe", "anomaly", "determined", "cause", "thrust_gain_hat",
                "lag_hat", "torque_d", "force_h", "wind_x", "wind_y", "wind_z", "mass_true", "cycle_ms"])


def run_mission(scn: MissionScenario, controller_kind: str = "pid", fw_cfg: FrameworkConfig = FrameworkConfig(),
                aircraft: str = "dji_matrice_400", dt: float = 0.004, framework_decimation: int = 5,
                log_decimation: int = 10, safety: SafetyLimits = SafetyLimits()) -> MissionRun:
    spec = load_spec(aircraft)
    ac = load_aircraft(aircraft)
    radius = float(sval(spec, "simulation_assumptions.vehicle_clearance_radius"))
    sp = SensorParams.from_spec(spec)
    bp = BatteryParams.from_spec(spec)
    facade = facade_plane()
    centres, halves, _ = building_boxes(scn.facade_variant)

    cp = contact_params(spec, ac)
    plant = MultirotorPlant(ac, contact=cp)
    plant.truth.air_density_ratio = scn.site.density_ratio
    cov = CoverageConfig()
    z_rest = cp.resting_height(ac.mass, ac.gravity)
    wps = build_coverage_mission(facade, cov, z_rest)
    mission = MissionManager(wps, facade, cov, MotionLimits(), z_rest, min_standoff=radius + 0.5)
    plant.state = plant.ground_state(np.array(cov.pad_xy), yaw=mission.yaw)
    plant.armed = False
    battery = Battery(bp, scn.soc0, scn.capacity_scale, scn.r0_scale)
    wind = scn.wind(dt)
    rng = np.random.default_rng(scn.seed + 104729)
    sensors = SensorSuite(sp, scn.seed, scn.sensor_faults, facade, scn.site)
    s0 = plant.state
    ekf = ErrorStateEKF(sp, ac.gravity, facade, p=s0.position + 0.02 * rng.standard_normal(3), q=s0.quaternion.copy())
    ctrl = make_controller(controller_kind, ac)
    # The operator knows the site altitude/temperature (barometer, thermometer): expected thrust gain.
    fw_cfg = replace(fw_cfg, expected_density_ratio=scn.site.density_ratio)
    fw = ResearchFramework(fw_cfg, ac, bp, radius, mission, ctrl, estimator=ekf)

    events = sorted(scn.truth_events, key=lambda e: e.t)
    rows: list[np.ndarray] = []
    n_steps = int(scn.duration_max / dt)
    f_acc, w_acc, n_acc = np.zeros(3), np.zeros(3), 0
    battery_meas = None
    energy_true = 0.0
    energy_tag = dict.fromkeys(TAGS, 0.0)
    cycle_times: list[float] = []
    landed_at: float | None = None
    still_since: float | None = None
    status, message = "timeout", ""
    out = None
    prev_v = s0.velocity.copy()
    acc_true = np.zeros(3)
    max_impact = 0.0
    t = 0.0
    try:
        for k in range(n_steps):
            t = k * dt
            while events and events[0].t <= t + 1e-12:
                _apply_event(plant, events.pop(0), ac)
            s = plant.state
            x = s.to_vector()
            w = wind.velocity(t, s.position)

            c0 = time.perf_counter()
            imu = sensors.imu(t, plant.specific_force_body(x, w), s.angular_velocity)
            if k > 0:
                ekf.predict(imu, dt)
            truth = TruthSample(t, s.position.copy(), s.velocity.copy(), s.quaternion.copy())
            for m in sensors.poll(t, truth, (battery.state.voltage, battery.state.current, battery.state.soc)):
                if m.sensor == "battery":
                    battery_meas = m.value
                else:
                    ekf.update(m, t)
            est = ekf.estimate()
            ref = mission.reference(t)
            phase = mission.phase(t)
            if phase == "ground":
                plant.armed = False
                ctrl.reset()
                omega_cmd = np.zeros(plant.n)
            elif mission.landed:
                plant.armed = False
                omega_cmd = np.zeros(plant.n)
            else:
                plant.armed = True
                out = ctrl.update(est, ref)
                omega_cmd = out.omega_cmd
            f_acc += imu.accel - ekf.ba
            w_acc += imu.gyro - ekf.bg
            n_acc += 1
            if k % framework_decimation == 0 and out is not None and phase != "ground":
                inp = FrameworkInputs(t, est, ekf.diagnostics(), f_acc / n_acc, w_acc / n_acc,
                                      out.allocation.achieved_wrench.copy(), battery_meas,
                                      float(np.linalg.norm(ref.position - est.position)), phase, mission.tag(t))
                fw.update(inp)
                f_acc[:], w_acc[:], n_acc = 0.0, 0.0, 0
            cycle = time.perf_counter() - c0
            cycle_times.append(cycle)

            # estimate-based landing detection (final landing segment finished or terminating)
            if not mission.landed and phase in ("landing", "hold") and t >= mission.end_time - 0.5:
                if est.position[2] < z_rest + 0.08 and np.linalg.norm(est.velocity) < 0.15:
                    still_since = t if still_since is None else still_since
                    if t - still_since > 1.0:
                        mission.landed = True
                        landed_at = t
                else:
                    still_since = None

            if k % log_decimation == 0:
                eul = euler_zyx_from_quat(s.quaternion)
                rep = fw.report
                snap = fw.sysid.snapshot() if fw_cfg.enabled else None
                rows.append(np.array([
                    t, *s.position, *s.velocity, eul[0], eul[1], eul[2], *s.quaternion,
                    *est.position, *est.velocity, euler_zyx_from_quat(est.quaternion)[2],
                    *ref.position, *ref.velocity,
                    np.linalg.norm(ref.position - s.position), np.linalg.norm(est.position - s.position),
                    np.linalg.norm(ref.velocity - s.velocity),
                    rotation_angle_between(s.quaternion, out.q_des) if out is not None else 0.0,
                    facade.distance(s.position), facade.distance(ref.position), clearance(s.position, centres, halves, radius),
                    *acc_true, out.wrench_cmd[0] if out is not None else 0.0,
                    battery.state.current * battery.state.voltage, energy_true, battery.state.voltage, battery.state.soc,
                    PHASES.index(phase), TAGS.index(mission.tag(t)) if mission.tag(t) in TAGS else 0, mission.speed_scale,
                    float(mission.probe_active(t)), float(rep.anomaly) if rep else 0.0, float(rep.determined) if rep else 0.0,
                    CAUSE_CODES.get(rep.cause, 0) if rep else 0,
                    snap.thrust_gain if snap else np.nan, snap.lag if snap else np.nan,
                    float(np.linalg.norm(fw.diagnoser.torque_disturbance[:2])) if fw_cfg.enabled else np.nan,
                    float(np.linalg.norm(fw.diagnoser.external_force_h)) if fw_cfg.enabled else np.nan,
                    *w, plant.truth.mass_properties.mass, cycle * 1e3,
                ]))

            # advance truth
            plant.step(dt, omega_cmd, w)
            p_el = electrical_power(shaft_power(plant.state.rotor_speeds, ac.rotor.k_m * plant.truth.air_density_ratio,
                                                plant.truth.health.effectiveness), ac.power) if plant.armed or np.any(plant.state.rotor_speeds > 1) else ac.power.avionics_power
            battery.step(p_el, dt)
            energy_true += p_el * dt
            tag = "termination" if (mission.aborted or mission.terminated) and mission.tag(t) == "recovery" else mission.tag(t)
            energy_tag[tag if tag in energy_tag else "nominal"] += p_el * dt
            plant.truth.omega_max_scale = min(1.0, battery.voltage_ratio)
            acc_true = (plant.state.velocity - prev_v) / dt
            prev_v = plant.state.velocity.copy()

            sn = plant.state
            if cp is not None and sn.position[2] < z_rest + 0.02:
                max_impact = max(max_impact, -float(s.velocity[2]))
            tilt = np.degrees(np.arccos(np.clip(1 - 2 * (sn.quaternion[1] ** 2 + sn.quaternion[2] ** 2), -1, 1)))
            if tilt > 70.0:
                status, message = "crash", f"tilt {tilt:.0f} deg"
                break
            if max_impact > 2.5:
                status, message = "crash", f"ground impact at {max_impact:.1f} m/s"
                break
            if clearance(sn.position, centres, halves, radius) < -0.05:
                status, message = "crash", "vehicle envelope intersects a building solid"
                break
            if battery.state.depleted:
                status, message = "crash", "battery depleted in flight" if sn.position[2] > z_rest + 0.2 else "battery depleted"
                break
            if landed_at is not None and t - landed_at > 2.0:
                status = "completed" if not (mission.aborted or mission.terminated) else "landed_early"
                break
    except StateValidationError as exc:
        status, message = "failed", str(exc)

    ct = np.asarray(cycle_times)
    info = {
        "message": message, "t_end": t, "landed_at": landed_at, "coverage_fraction": mission.coverage_fraction(),
        "energy_true_j": energy_true, "energy_tag_true_j": energy_tag,
        "energy_framework_j": dict(fw.energy.buckets), "soc_end": battery.state.soc,
        "remaining_energy_true_j": max(0.0, fw.energy._energy_between(fw.energy.soc_floor, max(battery.state.soc, fw.energy.soc_floor))),
        "remaining_energy_framework_j": fw.energy.remaining_j(),
        "mission_events": mission.events, "aborted": mission.aborted, "terminated": mission.terminated,
        "n_probes": fw.planner.n_probes, "fault_onset": scn.fault_onset, "fault_class": scn.fault_class,
        "cycle_ms_mean": float(ct.mean() * 1e3) if ct.size else float("nan"),
        "cycle_ms_p99": float(np.percentile(ct, 99) * 1e3) if ct.size else float("nan"),
        "cycle_ms_max": float(ct.max() * 1e3) if ct.size else float("nan"),
        "missed_deadlines": int((ct > dt).sum()), "n_cycles": int(ct.size), "control_period_ms": dt * 1e3,
        "max_ground_impact_mps": max_impact, "radius": radius, "z_rest": z_rest,
    }
    return MissionRun(scn, controller_kind, fw_cfg, list(LOG_COLUMNS), np.array(rows), fw.events, status, info)
