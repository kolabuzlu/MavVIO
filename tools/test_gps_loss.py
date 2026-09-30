"""Fly an AUTO mission in the plane simulator, switch the GPS off mid-flight, then back on.

Records the simulator's true position against ArduPilot's own estimate, the active EKF source set,
GPS state, cross-track error and the switch-over script's state into results/<name>/data.csv, plus
events and a summary.

Events during the GPS outage (seconds counted from the moment the GPS goes off):
  --wind-shift 60,9,310       at 60 s the wind changes to 9 m/s from 310 deg
  --vio-drift 0.4,45          the simulated VIO drifts at 0.4 m/s towards 45 deg (position error grows)
  --vio-dropout 100,8         the simulated VIO stops for 8 s at 100 s (tracking lost)
  --vio-runaway 40,3,120,30   from 40 s the VIO's velocity error grows by 3 m/s every second towards
                              120 deg, up to 30 m/s (a VIO that has diverged, like OpenVINS did)
Drift and runaway are made by the simulator-only script tools/sitl_scripts/sim_vio_fault.lua.

Examples:
  python test_gps_loss.py --name vio_switch                 # GPS lost, simulated VIO takes over
  python test_gps_loss.py --name no_vio --no-vio            # same, but without any VIO (for comparison)
  python test_gps_loss.py --name noisy --param SIM_VICON_P_SD=0.5
  python test_gps_loss.py --name old --script ..\\scripts\\archive\\vio_gps_switch_v1.lua
"""
import argparse
import csv
import math
import shutil
import time

from pymavlink import mavutil

import sitl

ML = mavutil.mavlink
PARAMS = sitl.ROOT / "params" / "vio_plane.parm"
NO_VIO_PARAMS = sitl.ROOT / "params" / "no_vio_override.parm"
SCRIPT = sitl.ROOT / "scripts" / "vio_gps_switch.lua"
FAULT_SCRIPT = sitl.ROOT / "tools" / "sitl_scripts" / "sim_vio_fault.lua"
SCRIPT_STATES = {0: "GPS", 1: "VIO", 2: "dead reckoning"}   # VSW_ST values
HOME_LAT, HOME_LON = (float(v) for v in sitl.HOME.split(",")[:2])
CORNERS_NE = [(300, -300), (300, 500), (-300, 500), (-300, -300)]  # mission rectangle, metres from home
CRUISE_ALT = 100


def offset(north_m, east_m):
    """Lat/lon of a point north_m/east_m metres from home."""
    lat = HOME_LAT + north_m / 6371000 * 180 / math.pi
    lon = HOME_LON + east_m / (6371000 * math.cos(math.radians(HOME_LAT))) * 180 / math.pi
    return lat, lon


def local_ne(lat, lon):
    """Metres north/east of home."""
    north = math.radians(lat - HOME_LAT) * 6371000
    east = math.radians(lon - HOME_LON) * 6371000 * math.cos(math.radians(HOME_LAT))
    return north, east


def upload_mission(m, corners=None):
    """Take-off, then the waypoints (metres north/east of home; default the rectangle) round and round."""
    corners = corners or CORNERS_NE
    items = [(ML.MAV_CMD_NAV_WAYPOINT, 0, 0, HOME_LAT, HOME_LON, 0),     # 0: home (replaced by vehicle)
             (ML.MAV_CMD_NAV_TAKEOFF, 15, 0, HOME_LAT, HOME_LON, 80)]    # 1: take off, 15 deg pitch, 80 m
    items += [(ML.MAV_CMD_NAV_WAYPOINT, 0, 0, *offset(n, e), CRUISE_ALT) for n, e in corners]    # 2 ...
    items.append((ML.MAV_CMD_DO_JUMP, 2, 50, 0, 0, 0))                     # back to 2, 50 times
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), ML.MAV_MISSION_TYPE_MISSION)
    while True:
        msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=10)
        if msg is None:
            raise RuntimeError("mission upload timed out")
        if msg.get_type() == "MISSION_ACK":
            if msg.type != ML.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"mission rejected ({msg.type})")
            return
        cmd, p1, p2, lat, lon, alt = items[msg.seq]
        frame = ML.MAV_FRAME_GLOBAL if msg.seq == 0 else ML.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        m.mav.mission_item_int_send(m.target_system, m.target_component, msg.seq, frame, cmd, 0, 1,
                                    p1, p2, 0, 0, int(lat * 1e7), int(lon * 1e7), alt, ML.MAV_MISSION_TYPE_MISSION)


def arm_and_auto(m, timeout=90):
    m.set_mode(m.mode_mapping()["AUTO"])
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.mav.command_long_send(m.target_system, m.target_component, ML.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        end = time.time() + 2
        while time.time() < end:
            msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT" and "PreArm" in msg.text:
                print("   ", msg.text)
            elif msg.get_type() == "HEARTBEAT" and msg.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED:
                return
    raise RuntimeError("could not arm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="vio_switch")
    ap.add_argument("--script", default=str(SCRIPT), help="switch-over script to fly with")
    ap.add_argument("--sitl", choices=["mp", "wsl"], default="mp",
                    help="simulator: Mission Planner's ArduPlane.exe (4.8.0-dev) or the Ubuntu build (4.7.1)")
    ap.add_argument("--no-vio", action="store_true", help="fly without any visual odometry")
    ap.add_argument("--gps-on-s", type=float, default=150, help="seconds after arming before the GPS is switched off")
    ap.add_argument("--gps-off-s", type=float, default=180, help="how long the GPS stays off")
    ap.add_argument("--after-s", type=float, default=90, help="seconds to keep flying after the GPS is back")
    ap.add_argument("--speedup", type=int, default=4)
    ap.add_argument("--param", action="append", default=[], help="extra NAME=VALUE set before flight")
    ap.add_argument("--wind-shift", help="T,SPEED,DIR: wind change T s after GPS loss")
    ap.add_argument("--vio-drift", help="RATE,DIR: simulated VIO drift in m/s towards DIR deg, from GPS loss on")
    ap.add_argument("--vio-dropout", help="T,DURATION: simulated VIO stops T s after GPS loss for DURATION s")
    ap.add_argument("--vio-runaway", help="T,RATE,DIR[,MAX]: from T s after GPS loss the simulated VIO's velocity "
                                          "error grows RATE m/s per second towards DIR deg, up to MAX m/s (default 30)")
    args = ap.parse_args()
    wind_shift = [float(v) for v in args.wind_shift.split(",")] if args.wind_shift else None
    dropout = [float(v) for v in args.vio_dropout.split(",")] if args.vio_dropout else None
    fault = {}   # sim_vio_fault.lua parameters
    if args.vio_drift:
        rate, direction = (float(v) for v in args.vio_drift.split(","))
        fault.update(VFT_DRIFT=rate, VFT_DRIFT_DIR=direction)
    if args.vio_runaway:
        values = [float(v) for v in args.vio_runaway.split(",")]
        fault.update(VFT_RUN_T=values[0], VFT_RUN_ACC=values[1], VFT_RUN_DIR=values[2],
                     VFT_RUN_MAX=values[3] if len(values) > 3 else 30)

    out = sitl.ROOT / "results" / args.name
    out.mkdir(parents=True, exist_ok=True)
    events = open(out / "events.txt", "w", encoding="utf-8")

    def event(t, text):
        line = f"{t:7.1f} s  {text}"
        print(line)
        events.write(line + "\n")
        events.flush()

    sitl.install_scripts(args.script, FAULT_SCRIPT, build=args.sitl)
    defaults = [PARAMS, NO_VIO_PARAMS] if args.no_vio else [PARAMS]
    proc = sitl.start(extra_defaults=defaults, speedup=args.speedup, vicon=not args.no_vio, build=args.sitl)
    completed = False
    try:
        m = sitl.connect(timeout=60 if args.sitl == "mp" else 180)   # Ubuntu can be slow to start while busy
        sitl.set_rates(m, {ML.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10, ML.MAVLINK_MSG_ID_SIMSTATE: 10,
                           ML.MAVLINK_MSG_ID_GPS_RAW_INT: 5, ML.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 5,
                           ML.MAVLINK_MSG_ID_VFR_HUD: 5, ML.MAVLINK_MSG_ID_MISSION_CURRENT: 2,
                           ML.MAVLINK_MSG_ID_SYS_STATUS: 1})
        print("waiting for pre-arm checks ...")
        deadline = time.time() + 120
        while time.time() < deadline:
            msg = m.recv_match(type="SYS_STATUS", blocking=True, timeout=1)
            if msg is not None and msg.onboard_control_sensors_health & ML.MAV_SYS_STATUS_PREARM_CHECK:
                break
        for item in args.param:
            name, value = item.split("=")
            sitl.set_param(m, name, float(value))
            print(f"set {name} = {value}")
        for name, value in fault.items():
            sitl.set_param(m, name, value)
        roll_limit_before = sitl.get_param(m, "ROLL_LIMIT_DEG")
        rtl_radius_before = sitl.get_param(m, "RTL_RADIUS")
        upload_mission(m)
        arm_and_auto(m)

        state = dict(t=0.0, truth=None, est=None, alt=None, fix=None, src=1, xtrack=0.0, airspeed=0.0, wp=0,
                     vsw=0, verr=-1.0, ivr=-1.0, mode="AUTO")
        t_arm = None
        phase = "gps"
        rows = []
        writer_file = open(out / "data.csv", "w", newline="")
        writer = csv.writer(writer_file)
        writer.writerow(["t_s", "phase", "truth_n", "truth_e", "est_n", "est_e", "alt_rel", "gps_fix",
                         "src_set", "pos_err_m", "xtrack_m", "airspeed", "wp", "script_state", "vio_innov_ratio", "vio_mismatch", "mode"])
        last_row_t = -1.0
        while True:
            msg = m.recv_match(blocking=True, timeout=5)
            if msg is None:
                raise RuntimeError("simulator stopped sending messages")
            kind = msg.get_type()
            if hasattr(msg, "time_boot_ms"):
                state["t"] = msg.time_boot_ms / 1000
                t_arm = t_arm if t_arm is not None else state["t"]
            t = state["t"] - (t_arm or state["t"])
            if kind == "SIMSTATE":
                state["truth"] = local_ne(msg.lat / 1e7, msg.lng / 1e7)
            elif kind == "GLOBAL_POSITION_INT":
                state["est"] = local_ne(msg.lat / 1e7, msg.lon / 1e7)
                state["alt"] = msg.relative_alt / 1000
            elif kind == "GPS_RAW_INT":
                state["fix"] = msg.fix_type
            elif kind == "NAMED_VALUE_FLOAT" and msg.name == "VSW_SRC":
                if int(msg.value) != state["src"]:
                    event(t, f"EKF source set {state['src']} -> {int(msg.value)}")
                state["src"] = int(msg.value)
            elif kind == "NAMED_VALUE_FLOAT" and msg.name == "VSW_ST":
                if int(msg.value) != state["vsw"]:
                    event(t, f"script state {SCRIPT_STATES.get(state['vsw'])} -> {SCRIPT_STATES.get(int(msg.value))}")
                state["vsw"] = int(msg.value)
            elif kind == "NAMED_VALUE_FLOAT" and msg.name == "VSW_VERR":
                state["verr"] = msg.value
            elif kind == "NAMED_VALUE_FLOAT" and msg.name == "VSW_IVR":
                state["ivr"] = msg.value
            elif kind == "NAV_CONTROLLER_OUTPUT":
                state["xtrack"] = msg.xtrack_error
            elif kind == "VFR_HUD":
                state["airspeed"] = msg.airspeed
            elif kind == "MISSION_CURRENT":
                state["wp"] = msg.seq
            elif kind == "STATUSTEXT":
                event(t, f"vehicle: {msg.text}")
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system and msg.type != ML.MAV_TYPE_GCS:
                mode = mavutil.mode_string_v10(msg)
                if mode != state["mode"]:
                    event(t, f"flight mode {state['mode']} -> {mode}")
                state["mode"] = mode

            since_off = t - args.gps_on_s
            if phase == "gps" and t >= args.gps_on_s:
                sitl.set_param(m, "SIM_GPS1_ENABLE", 0)
                phase = "gps_off"
                event(t, "TEST: GPS switched OFF")
                if fault:
                    sitl.set_param(m, "VFT_START", 1)
                    if "VFT_DRIFT" in fault:
                        event(t, f"TEST: simulated VIO starts drifting {fault['VFT_DRIFT']:g} m/s towards "
                                 f"{fault['VFT_DRIFT_DIR']:.0f} deg")
                    if "VFT_RUN_T" in fault:
                        event(t, f"TEST: simulated VIO will run away from +{fault['VFT_RUN_T']:g} s: velocity error "
                                 f"+{fault['VFT_RUN_ACC']:g} m/s per s towards {fault['VFT_RUN_DIR']:.0f} deg, "
                                 f"up to {fault['VFT_RUN_MAX']:g} m/s")
            elif phase == "gps_off" and wind_shift and since_off >= wind_shift[0]:
                sitl.set_param(m, "SIM_WIND_SPD", wind_shift[1])
                sitl.set_param(m, "SIM_WIND_DIR", wind_shift[2])
                event(t, f"TEST: wind shifts to {wind_shift[1]:g} m/s from {wind_shift[2]:.0f} deg")
                wind_shift = None
            elif phase == "gps_off" and dropout and len(dropout) == 2 and since_off >= dropout[0]:
                sitl.set_param(m, "SIM_VICON_FAIL", 1)
                event(t, "TEST: simulated VIO dropout starts")
                dropout.append("started")
            elif phase == "gps_off" and dropout and len(dropout) == 3 and since_off >= dropout[0] + dropout[1]:
                sitl.set_param(m, "SIM_VICON_FAIL", 0)
                event(t, "TEST: simulated VIO dropout ends")
                dropout = None
            elif phase == "gps_off" and t >= args.gps_on_s + args.gps_off_s:
                sitl.set_param(m, "SIM_GPS1_ENABLE", 1)
                phase = "gps_back"
                event(t, "TEST: GPS switched back ON")
            elif phase == "gps_back" and t >= args.gps_on_s + args.gps_off_s + args.after_s:
                break

            if state["truth"] and state["est"] and t - last_row_t >= 0.5:
                err = math.dist(state["truth"], state["est"])
                row = [round(t, 1), phase, *(round(v, 1) for v in state["truth"]), *(round(v, 1) for v in state["est"]),
                       round(state["alt"], 1), state["fix"], state["src"], round(err, 1), round(state["xtrack"], 1),
                       round(state["airspeed"], 1), state["wp"], state["vsw"], round(state["ivr"], 2), round(state["verr"], 1), state["mode"]]
                writer.writerow(row)
                rows.append(row)
                last_row_t = t
                if int(t) % 15 == 0 and t - int(t) < 0.5:
                    print(f"  t={t:5.0f} s  {phase:8} alt {state['alt']:5.1f} m  GPS fix {state['fix']}  "
                          f"source set {state['src']}  position error {err:6.1f} m  cross-track {state['xtrack']:6.1f} m"
                          + (f"  VIO checks: IMU {state['ivr']:4.2f}, airspeed {state['verr']:4.1f} m/s"
                             if state["verr"] >= 0 else ""))
        writer_file.close()
        roll_limit_after = sitl.get_param(m, "ROLL_LIMIT_DEG")
        rtl_radius_after = sitl.get_param(m, "RTL_RADIUS")

        summary = [f"run: {args.name}   simulator: {args.sitl}   VIO: {'no' if args.no_vio else 'simulated (sim:vicon)'}   "
                   f"extra params: {args.param or 'none'}",
                   f"script: {args.script}   VIO faults: {fault or 'none'}",
                   f"ROLL_LIMIT_DEG before the flight {roll_limit_before}, after it {roll_limit_after}; "
                   f"RTL_RADIUS before {rtl_radius_before}, after {rtl_radius_after}"]
        for ph, label in (("gps", "GPS on"), ("gps_off", "GPS OFF"), ("gps_back", "GPS back")):
            sel = [r for r in rows if r[1] == ph and r[0] > 60]  # skip take-off
            if sel:
                errs = [r[9] for r in sel]
                alts = [r[6] for r in sel]
                summary.append(f"{label:9}: position error mean {sum(errs) / len(errs):6.1f} m, max {max(errs):6.1f} m | "
                               f"altitude {min(alts):5.1f}-{max(alts):5.1f} m | waypoints reached: {sorted(set(r[12] for r in sel))}")
        text = "\n".join(summary)
        print("\n" + text)
        (out / "summary.txt").write_text(text + "\n", encoding="utf-8")
        completed = True
    finally:
        events.close()
        sitl.stop(proc, build=args.sitl)
        logs = sorted((sitl.workdir(args.sitl) / "logs").glob("*.BIN"), key=lambda p: p.stat().st_mtime)
        if completed and logs:
            shutil.copy(logs[-1], out / "flight.BIN")


if __name__ == "__main__":
    main()
