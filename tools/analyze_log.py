"""Analyse a simulator flight log (.BIN): true position (SIM) vs ArduPilot's position (POS), time-matched.

Writes results/<name>/log_data.csv, log_events.txt and summary.txt. The EKF source set comes from the
logged source-set events (EV 85/86/87 = set 1/2/3), and GPS on/off from the logged SIM_GPS1_ENABLE
changes. Logging starts at arming, so the first logged position is t = 0.

Also, when the log has them: bank angle (ATT), the EKF's velocity innovation test ratio (XKF4 SV, what
the script's IMU check watches), the switch-over script's own log (VSW: state, airspeed check, bank
limit), flight mode changes (MODE) and the injected VIO error (VFT, from tools/sitl_scripts/sim_vio_fault.lua).

Usage:  python analyze_log.py <name> [path/to/flight.BIN]   (default: results/<name>/flight.BIN)
"""
import csv
import math
import sys
from pathlib import Path

import numpy as np
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
SOURCE_SET_EVENTS = {85: 1, 86: 2, 87: 3}  # LogEvent ids for "EKF source set primary/secondary/tertiary"
SCRIPT_STATES = {0: "GPS", 1: "VIO", 2: "dead reckoning"}
MODE_NAMES = mavutil.mode_mapping_bynumber(mavutil.mavlink.MAV_TYPE_FIXED_WING)
MODE_RTL = 11
DEFAULTS = {"VSW_INNOV": 1.0, "VSW_VEL_ERR": 12.0, "VSW_REJ_T": 1.0}


def step_at(times, series, t, missing):
    """Value of a step-wise series (times, values) at each time in t; `missing` before its first sample."""
    if len(times) == 0:
        return np.full(len(t), missing, dtype=float)
    idx = np.searchsorted(times, t, side="right") - 1
    out = np.asarray(series, dtype=float)[np.clip(idx, 0, None)]
    out[idx < 0] = missing
    return out


def main(name, log_path=None):
    out = ROOT / "results" / name
    log_path = Path(log_path) if log_path else out / "flight.BIN"
    log = mavutil.mavlink_connection(str(log_path))
    sim, pos, events, t_arm = [], [], [], None
    att, xkf4, vsw, vft, modes = [], [], [], [], []
    gps_state = None
    params = {}   # last logged value of the test parameters, to spot changes
    first_params = {}
    while True:
        m = log.recv_match(type=["SIM", "POS", "MSG", "PARM", "EV", "ATT", "XKF4", "VSW", "VFT", "MODE"])
        if m is None:
            break
        kind, t = m.get_type(), m.TimeUS / 1e6
        if kind == "SIM":
            sim.append((t, m.Lat, m.Lng))
        elif kind == "POS":
            pos.append((t, m.Lat, m.Lng, m.RelHomeAlt))
            t_arm = t_arm if t_arm is not None else t
        elif kind == "ATT":
            att.append((t, m.Roll))
        elif kind == "XKF4" and m.C == m.PI:
            xkf4.append((t, m.SV))
        elif kind == "VSW":
            vsw.append((t, m.St, m.Src, m.IVR, m.VErr, m.RLim, getattr(m, "RRad", 0.0)))
        elif kind == "VFT":
            vft.append((t, math.hypot(m.VN, m.VE), math.hypot(m.PN, m.PE)))
        elif kind == "MODE":
            modes.append((t, m.ModeNum))
            events.append((t, f"flight mode -> {MODE_NAMES.get(m.ModeNum, m.ModeNum)}", None, None))
        elif kind == "EV" and m.Id in SOURCE_SET_EVENTS:
            events.append((t, f"EKF source set -> {SOURCE_SET_EVENTS[m.Id]}", SOURCE_SET_EVENTS[m.Id], None))
        elif kind == "MSG" and m.Message.startswith("VSW:"):
            events.append((t, f"vehicle: {m.Message}", None, None))
        elif kind == "PARM":
            first_params.setdefault(m.Name, m.Value)
            if m.Name == "SIM_GPS1_ENABLE":
                if gps_state is not None and m.Value != gps_state:
                    events.append((t, f"TEST: GPS switched {'ON' if m.Value else 'OFF'}", None, m.Value))
                gps_state = m.Value
            elif m.Name in ("SIM_WIND_DIR", "SIM_WIND_SPD", "SIM_VICON_FAIL", "SIM_VICON_VGLI_X", "VFT_START"):
                old = params.get(m.Name)
                params[m.Name] = m.Value
                if old is None or m.Value == old:
                    continue
                if m.Name == "SIM_WIND_DIR":
                    events.append((t, f"TEST: wind shift to {params.get('SIM_WIND_SPD', 0):g} m/s from {m.Value:.0f} deg", None, None))
                elif m.Name == "SIM_VICON_FAIL":
                    events.append((t, f"TEST: simulated VIO dropout {'starts' if m.Value else 'ends'}", None, None))
                elif m.Name == "SIM_VICON_VGLI_X":
                    events.append((t, "TEST: simulated VIO starts drifting", None, None))
                elif m.Name == "VFT_START" and m.Value >= 1:
                    if first_params.get("VFT_DRIFT", 0) > 0:
                        events.append((t, "TEST: simulated VIO starts drifting", None, None))
                    if first_params.get("VFT_RUN_ACC", 0) > 0:
                        events.append((t + first_params.get("VFT_RUN_T", 0), "TEST: simulated VIO starts running away",
                                       None, None))
    events.sort(key=lambda e: e[0])

    sim, pos = np.array(sim), np.array(pos)
    lat0, lon0 = pos[0, 1], pos[0, 2]
    k_n = math.pi / 180 * 6371000
    k_e = k_n * math.cos(math.radians(lat0))
    truth_lat = np.interp(pos[:, 0], sim[:, 0], sim[:, 1])   # true position at the exact POS timestamps
    truth_lon = np.interp(pos[:, 0], sim[:, 0], sim[:, 2])
    truth_n, truth_e = (truth_lat - lat0) * k_n, (truth_lon - lon0) * k_e
    est_n, est_e = (pos[:, 1] - lat0) * k_n, (pos[:, 2] - lon0) * k_e
    err = np.hypot(truth_n - est_n, truth_e - est_e)

    src_at, gps_at = np.ones(len(pos), int), np.ones(len(pos), int)
    for t, _, src, gps in events:
        if src is not None:
            src_at[pos[:, 0] >= t] = src
        if gps is not None:
            gps_at[pos[:, 0] >= t] = int(gps)

    tp = pos[:, 0]
    att, xkf4, vsw, vft = (np.array(a) if a else np.zeros((0, 7)) for a in (att, xkf4, vsw, vft))
    rrad = step_at(vsw[:, 0], vsw[:, 6], tp, 0) if len(vsw) else np.zeros(len(tp))
    loiter_radius = np.where(rrad != 0, np.abs(rrad), abs(first_params.get("WP_LOITER_RAD", 60)))
    roll = np.interp(tp, att[:, 0], att[:, 1]) if len(att) else np.zeros(len(tp))
    ivr = step_at(xkf4[:, 0], xkf4[:, 1], tp, -1) if len(xkf4) else np.full(len(tp), -1.0)
    state = step_at(vsw[:, 0], vsw[:, 1], tp, -1) if len(vsw) else np.full(len(tp), -1.0)
    verr = step_at(vsw[:, 0], vsw[:, 4], tp, -1) if len(vsw) else np.full(len(tp), -1.0)
    rlim = step_at(vsw[:, 0], vsw[:, 5], tp, first_params.get("ROLL_LIMIT_DEG", -1)) if len(vsw) \
        else np.full(len(tp), first_params.get("ROLL_LIMIT_DEG", -1))
    fault_v = step_at(vft[:, 0], vft[:, 1], tp, 0) if len(vft) else np.zeros(len(tp))
    modes = np.array(modes) if modes else np.zeros((0, 2))
    mode_at = step_at(modes[:, 0], modes[:, 1], tp, -1) if len(modes) else np.full(len(tp), -1.0)

    with open(out / "log_data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "truth_n", "truth_e", "est_n", "est_e", "alt_rel", "pos_err_m", "src_set", "gps_on",
                    "roll_deg", "roll_limit", "script_state", "imu_check", "airspeed_check", "vio_fault_vel",
                    "mode_num"])
        for i in range(0, len(pos), 2):
            w.writerow([round(pos[i, 0] - t_arm, 2), round(truth_n[i], 2), round(truth_e[i], 2), round(est_n[i], 2),
                        round(est_e[i], 2), round(pos[i, 3], 2), round(err[i], 3), src_at[i], gps_at[i],
                        round(roll[i], 1), round(rlim[i], 1), int(state[i]), round(ivr[i], 3), round(verr[i], 2),
                        round(fault_v[i], 2), int(mode_at[i])])
    with open(out / "log_events.txt", "w", encoding="utf-8") as f:
        for t, text, _, _ in events:
            f.write(f"{t - t_arm:7.1f} s  {text}\n")
    limits = {k: first_params.get(k, v) for k, v in DEFAULTS.items()}
    with open(out / "log_params.txt", "w", encoding="utf-8") as f:
        for k in ("VSW_INNOV", "VSW_VEL_ERR", "VSW_REJ_T", "VSW_ROLL", "ROLL_LIMIT_DEG", "VFT_DRIFT", "VFT_DRIFT_DIR",
                  "VFT_RUN_T", "VFT_RUN_ACC", "VFT_RUN_DIR", "VFT_RUN_MAX"):
            if k in first_params:
                f.write(f"{k} {first_params[k]:g}\n")

    lines = [f"run '{name}' - from the flight log, true vs estimated position matched in time"]
    t_rel = tp - t_arm
    t_off = next((t - t_arm for t, text, _, _ in events if text == "TEST: GPS switched OFF"), None)
    t_on = next((t - t_arm for t, text, _, _ in events if text == "TEST: GPS switched ON"), None)
    phases = [("before GPS loss", (t_rel > 60) & (t_rel < (t_off if t_off is not None else np.inf)))]
    if t_off is not None:
        phases += [("GPS off, VIO in use", (src_at == 2) & (gps_at == 0)),
                   ("GPS off, dead reckoning", (src_at == 1) & (gps_at == 0))]
    for label, sel in phases:
        if sel.any():
            e = err[sel]
            seconds = float(np.sum(np.diff(t_rel)[sel[:-1] & sel[1:]]))
            if seconds >= 5:
                lines.append(f"{label:23}: position error mean {e.mean():7.2f} m, 95% below {np.percentile(e, 95):7.2f} m, "
                             f"max {e.max():7.2f} m over {seconds:4.0f} s")
    if t_on is not None:
        after = t_rel > t_on
        bad = np.nonzero(after & (err > 5))[0]
        recovered = t_rel[bad[-1] + 1] - t_on if len(bad) and bad[-1] + 1 < len(t_rel) else 0.0
        lines.append(f"{'after GPS came back':23}: position error below 5 m again {recovered:.1f} s after GPS returned")
        lines.append(f"{'at GPS return':23}: position error {err[min(np.searchsorted(t_rel, t_on), len(err) - 1)]:.1f} m")

    if t_off is not None:
        outage = (gps_at == 0)
        before = (t_rel > 15) & (t_rel < t_off)
        text = f"{'bank angle':23}: max {np.abs(roll[before]).max():4.1f} deg before GPS loss"
        lowered = (rlim > 0) & (rlim < first_params.get("ROLL_LIMIT_DEG", np.inf))
        if lowered.any():
            # from 3 s after each lowering: the plane may be in a steeper turn at that moment
            starts = t_rel[1:][lowered[1:] & ~lowered[:-1]]
            since = t_rel - np.array([starts[starts <= t].max() if (starts <= t).any() else -np.inf for t in t_rel])
            sel = lowered & (since >= 3)
            seconds = float(np.sum(np.diff(t_rel)[lowered[:-1] & lowered[1:]]))
            text += (f", {np.abs(roll[sel]).max():4.1f} deg while the bank limit was {rlim[lowered].min():g} deg "
                     f"({seconds:.0f} s), {rlim[-1]:g} deg at the end")
        else:
            text += f", {np.abs(roll[outage]).max():4.1f} deg without GPS (bank limit unchanged)"
        lines.append(text)
        on_vio = (src_at == 2) & outage
        fresh = on_vio & (ivr >= 0)
        if fresh.any():
            text = f"{'VIO checks, on VIO':23}: IMU check max {ivr[fresh].max():.2f} (limit {limits['VSW_INNOV']:g})"
            if (verr[on_vio] >= 0).any():
                text += f", airspeed check max {verr[on_vio].max():.1f} m/s (limit {limits['VSW_VEL_ERR']:g})"
            lines.append(text)
    t_run = next((t - t_arm for t, text, _, _ in events if "running away" in text), None)
    t_rej = next((t - t_arm for t, text, _, _ in events if "VIO rejected" in text), None)
    if t_rej is not None:
        i = min(np.searchsorted(t_rel, t_rej), len(t_rel) - 1)
        lines.append(f"{'VIO rejected':23}: at {t_rej:.1f} s" + (f" ({t_rej - t_run:.1f} s after the runaway began)" if t_run else "")
                     + f", injected VIO velocity error then {fault_v[i]:.1f} m/s, position error then {err[i]:.1f} m")
    elif t_run is not None:
        lines.append(f"{'VIO rejected':23}: never (runaway began at {t_run:.1f} s)")
    rtl = (mode_at == MODE_RTL) & (gps_at == 0)
    if rtl.any() and t_off is not None:
        t_rtl = t_rel[np.argmax(rtl)]
        lines.append(f"{'RTL':23}: at {t_rtl:.1f} s ({t_rtl - t_off:.1f} s after GPS loss)")
        est_home = np.hypot(est_n, est_e)
        # circling where ArduPilot thinks home is: from the first time it reaches its RTL circle
        arrived = np.nonzero(rtl & (est_home < 1.3 * loiter_radius))[0]
        home = rtl & (np.arange(len(rtl)) >= arrived[0]) if len(arrived) else np.zeros(len(rtl), bool)
        if home.any():
            true_home = np.hypot(truth_n, truth_e)
            lines.append(f"{'circling home, no GPS':23}: RTL circle {loiter_radius[home].min():.0f}-"
                         f"{loiter_radius[home].max():.0f} m; really {true_home[home].min():.0f}-{true_home[home].max():.0f} m "
                         f"from home (mean {true_home[home].mean():.0f} m), ArduPilot thought "
                         f"{est_home[home].min():.0f}-{est_home[home].max():.0f} m")
    for t, text, _, _ in events:
        lines.append(f"{t - t_arm:7.1f} s  {text}")
    (out / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
