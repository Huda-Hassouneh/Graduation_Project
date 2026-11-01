"""Mission-level evaluation metrics (Phase J). EVALUATION ONLY: these functions read truth columns and
the scenario's fault labels, which the research framework never sees.

Windows and definitions (docs/EXPERIMENTAL_METHODOLOGY.md):

* airborne window: phases transit, coverage, probe, exit, hold (take-off and landing excluded from
  tracking metrics; they are evaluated by the landing metrics);
* detection window: ``[onset, onset + detect_window_s]``; an anomaly event before the onset is a false
  alarm (in fault-free scenarios every anomaly event is a false alarm);
* recovery time: from onset until ``pos_err < recovery_tol`` holds for ``recovery_hold_s``;
* safety violations: clearance below ``min_clearance``, horizontal/vertical speed or acceleration above
  the evaluation envelope (SafetyLimits), geofence exits; counted as violation TIME [s].
"""

from __future__ import annotations

import numpy as np

from graduation_project.experiments.mission_runner import MissionRun, SafetyLimits
from graduation_project.planning.mission import PHASES

AIRBORNE = [PHASES.index(p) for p in ("transit", "coverage", "probe", "exit", "hold")]
TAG_DIAG, TAG_REC, TAG_TERM = 1, 2, 3


def _violation_time(mask: np.ndarray, dt: float) -> float:
    return float(mask.sum() * dt)


def mission_metrics(run: MissionRun, safety: SafetyLimits = SafetyLimits(), detect_window_s: float = 30.0,
                    recovery_tol: float = 0.3, recovery_hold_s: float = 3.0) -> dict[str, float]:
    m: dict[str, float] = {}
    info = run.info
    if run.data.size == 0:
        return {"status_code": -1}
    t = run.col("t")
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.04
    phase = run.col("phase").astype(int)
    air = np.isin(phase, AIRBORNE)
    if not np.any(air):
        air = np.ones_like(t, dtype=bool)
    e = run.col("pos_err")
    cov = phase == PHASES.index("coverage")

    # trajectory --------------------------------------------------------------------------
    m["pos_rmse_m"] = float(np.sqrt(np.mean(e[air] ** 2)))
    m["pos_mae_m"] = float(np.mean(e[air]))
    m["pos_max_err_m"] = float(np.max(e[air]))
    m["vel_rmse_mps"] = float(np.sqrt(np.mean(run.col("vel_err")[air] ** 2)))
    att = np.degrees(run.col("att_err")[air])
    m["att_err_mean_deg"], m["att_err_max_deg"] = float(att.mean()), float(att.max())
    so = run.col("standoff") - run.col("standoff_ref")
    m["standoff_err_rmse_m"] = float(np.sqrt(np.mean(so[cov] ** 2))) if np.any(cov) else float("nan")
    m["est_err_rmse_m"] = float(np.sqrt(np.mean(run.col("est_err")[air] ** 2)))

    # diagnosis (truth labels used here only) ----------------------------------------------
    onset, fclass = info.get("fault_onset"), info.get("fault_class", "none")
    alarms = [ev for ev in run.events if ev["source"] == "diagnosis" and ev["event"].startswith("anomaly")]
    determined = [ev for ev in alarms if ev["event"] == "anomaly"]
    if onset is None or fclass == "none":
        m["false_alarm"] = float(len(alarms) > 0)
        m["n_false_alarms"] = float(len(alarms))
        m["detected"] = float("nan")
        m["detection_delay_s"] = float("nan")
        m["classified_correct"] = float("nan")
        m["first_class_correct"] = float("nan")
    else:
        before = [ev for ev in alarms if ev["t"] < onset]
        inwin = [ev for ev in alarms if onset <= ev["t"] <= onset + detect_window_s]
        m["false_alarm"] = float(len(before) > 0)
        m["n_false_alarms"] = float(len(before))
        m["detected"] = float(len(inwin) > 0 or any(onset - 0.5 <= ev["t"] < onset for ev in before))
        m["detection_delay_s"] = float(inwin[0]["t"] - onset) if inwin else float("nan")
        det_in = [ev for ev in determined if onset <= ev["t"] <= onset + detect_window_s]
        causes = [ev["detail"].split("diagnosed: ")[-1].split(" (")[0] for ev in det_in]
        m["classified_correct"] = float(fclass in causes)
        m["first_class_correct"] = float(bool(causes) and causes[0] == fclass)
    m["n_undetermined_reports"] = float(sum(ev["event"] == "anomaly_undetermined" for ev in alarms))

    # recovery ---------------------------------------------------------------------------------
    if onset is not None and fclass != "none":
        post = t >= onset
        m["max_deviation_after_onset_m"] = float(e[post].max()) if np.any(post) else float("nan")
        bad = post & (e > recovery_tol)
        if not np.any(bad):
            m["recovery_time_s"] = 0.0
        else:
            ok_run = 0
            rec = float("nan")
            for i in np.where(post)[0]:
                ok_run = ok_run + 1 if e[i] <= recovery_tol else 0
                if ok_run * dt >= recovery_hold_s and t[i] > t[np.where(bad)[0][0]]:
                    rec = float(t[i] - recovery_hold_s - onset)
                    break
            m["recovery_time_s"] = rec
    else:
        m["max_deviation_after_onset_m"] = float("nan")
        m["recovery_time_s"] = float("nan")

    # safety -----------------------------------------------------------------------------------
    clr = run.col("clearance")
    vh = np.hypot(run.col("v_x"), run.col("v_y"))
    vz = np.abs(run.col("v_z"))
    acc = np.linalg.norm(run.cols("acc"), axis=1)
    p = run.cols("p")
    gmin, gmax = np.asarray(safety.geofence_min), np.asarray(safety.geofence_max)
    m["min_clearance_m"] = float(clr.min())
    m["clearance_violation_s"] = _violation_time(clr < safety.min_clearance, dt)
    m["speed_violation_s"] = _violation_time(air & ((vh > safety.max_horizontal_speed) | (vz > safety.max_vertical_speed)), dt)
    m["accel_violation_s"] = _violation_time(air & (acc > safety.max_acceleration), dt)
    m["geofence_violation_s"] = _violation_time(np.any((p < gmin) | (p > gmax), axis=1), dt)
    m["crash"] = float(run.status in ("crash", "failed"))
    m["landed"] = float(info.get("landed_at") is not None)
    m["landing_impact_mps"] = float(info.get("max_ground_impact_mps", float("nan")))
    m["safety_violation_any"] = float(m["clearance_violation_s"] > 0 or m["speed_violation_s"] > 0 or
                                      m["accel_violation_s"] > 0 or m["geofence_violation_s"] > 0 or m["crash"] > 0)

    # mission / recovery outcome -------------------------------------------------------------------
    m["coverage_fraction"] = float(info.get("coverage_fraction", float("nan")))
    m["mission_completed"] = float(run.status == "completed")
    m["safe_outcome"] = float(run.status in ("completed", "landed_early") and m["safety_violation_any"] == 0.0)
    m["aborted_or_terminated"] = float(bool(info.get("aborted") or info.get("terminated")))
    m["n_probes"] = float(info.get("n_probes", 0))
    m["mission_time_s"] = float(info.get("t_end", float("nan")))

    # energy (truth bookkeeping; framework's own estimate reported separately) ----------------------
    et = info.get("energy_tag_true_j", {})
    m["energy_total_kj"] = float(info.get("energy_true_j", float("nan"))) / 1000.0
    m["energy_diagnostic_kj"] = float(et.get("diagnostic", 0.0)) / 1000.0
    m["energy_recovery_kj"] = float(et.get("recovery", 0.0) + et.get("termination", 0.0)) / 1000.0
    m["energy_remaining_kj"] = float(info.get("remaining_energy_true_j", float("nan"))) / 1000.0
    m["energy_remaining_estimate_kj"] = float(info.get("remaining_energy_framework_j", float("nan"))) / 1000.0
    m["soc_end"] = float(info.get("soc_end", float("nan")))
    m["energy_per_coverage_kj"] = m["energy_total_kj"] / max(m["coverage_fraction"], 1e-3)

    # computation ---------------------------------------------------------------------------------
    m["cycle_ms_mean"] = float(info.get("cycle_ms_mean", float("nan")))
    m["cycle_ms_p99"] = float(info.get("cycle_ms_p99", float("nan")))
    m["cycle_ms_max"] = float(info.get("cycle_ms_max", float("nan")))
    m["missed_deadline_fraction"] = float(info.get("missed_deadlines", 0)) / max(float(info.get("n_cycles", 1)), 1.0)
    m["cpu_s_per_sim_s"] = float(info.get("cpu_s", float("nan"))) / max(m["mission_time_s"], 1e-9)
    m["rss_mb"] = float(info.get("rss_mb", float("nan")))
    return m
