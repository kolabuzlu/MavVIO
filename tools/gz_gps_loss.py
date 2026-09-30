"""Step 3: GPS loss in the Gazebo simulation with the real VIO (OpenVINS) in the loop.

Needs setup/run_gazebo_vio.sh (simulation + ArduPilot) and setup/run_vio_live.sh (the VIO chain with
tools/vio_bridge.py) running inside Ubuntu. Takes off like gz_fly_zephyr.py, flies the rectangle in
AUTO, waits until the VIO reaches the flight controller ("VIO: started ..." from the companion), flies
--gps-s more on GPS, then switches the simulated GPS off. vio_gps_switch.lua should switch to the VIO and
(VSW_GPS_RTL) fly home. Every 5 s it prints where ArduPilot thinks the plane is and where it really is
(SIMSTATE = the Gazebo truth), both as distance from home. After --lost-s the GPS comes back.

Usage:  python gz_gps_loss.py [--gps-s 40] [--lost-s 180] [--wind SPEED,FROM_DEG] [--wind-shift S,SPEED,FROM_DEG]
                              [--far N,E [--far-radius 150]] [--no-vio S]
  --wind        wind once the plane is up (40 m), in Gazebo (setup/gz_set_wind.sh) and for ArduPilot's simulated
                airspeed sensor (SIM_WIND_SPD / SIM_WIND_DIR) alike. Not before: the Zephyr's vertical launch is
                fragile in a crosswind (6 m/s from 250 deg: it rolled 77 deg at 4 m and flew into the ground)
  --wind-shift  S seconds after the GPS loss the wind changes to this - what the flight controller cannot
                know without GPS, unless the VIO shows it how the plane moves over the ground
  --far         once the VIO is in, fly (GUIDED) to this point, metres north and east of home, and lose the
                GPS there (within --far-radius of it) instead of on the rectangle
  --camera-stall S,DUR  S seconds after the GPS loss the VIO camera's pictures stop for DUR seconds while its
                IMU goes on (the grey-picture converter is paused) - like a USB hiccup on the plane
  --kill-vio S  S seconds after the GPS loss OpenVINS is killed (its launcher starts it again) - a crash
  --mission weave  instead of the rectangle: a straight 2 km leg across the middle of the world, then back
                along the same line in a zig-zag (75 m either side, every 300 m), round and round - does
                weaving help the VIO? (straight flight at constant speed hides the scale from one camera)
  --restart-at-legs  kill OpenVINS at the start of every leg (the companion starts it again), so each leg
                is flown by a fresh VIO and the legs can be compared
  --param NAME=VALUE  set a flight controller parameter before take-off (repeatable) - for tuning tests
"""
import argparse
import math
import subprocess
import time

from pymavlink import mavutil

import sitl
from gz_fly_zephyr import arm, throttle_override, wait_prearm
from test_gps_loss import CRUISE_ALT, local_ne, offset, upload_mission

ML = mavutil.mavlink


def weave_mission(half=75.0, step=300.0):
    """Waypoints (north, east) for --mission weave, and which mission items start a leg."""
    a, b = (-700.0, 700.0), (700.0, -700.0)                 # a diagonal through the finely painted middle
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    un, ue = (a[0] - b[0]) / length, (a[1] - b[1]) / length  # along the way back, b -> a
    zig, d, side = [], step, 1
    while d < length - step / 2:
        zig.append((b[0] + un * d - ue * half * side, b[1] + ue * d + un * half * side))
        d, side = d + step, -side
    corners = [a, b] + zig
    # the flight controller reports the waypoint it heads for: 3 (b) = the straight leg from a, 4 = the zig-zag from b
    return corners, {3: "straight", 4: "zig-zag"}


def wsl(cmd):
    subprocess.run(["wsl", "-d", "Ubuntu-22.04", "--", "bash", "-c", cmd], capture_output=True)


def set_wind(m, speed, from_deg):
    """The same wind in Gazebo (the plane's aerodynamics) and in SITL (its airspeed sensor)."""
    subprocess.run(["wsl", "-d", "Ubuntu-22.04", "--", "bash", "/mnt/c/Users/funfo/vio/setup/gz_set_wind.sh",
                    f"{speed:g}", f"{from_deg:g}"], check=True, capture_output=True)
    sitl.set_param(m, "SIM_WIND_SPD", speed)
    sitl.set_param(m, "SIM_WIND_DIR", from_deg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--gps-s", type=float, default=40, help="seconds on GPS after the VIO is in, before GPS off")
    ap.add_argument("--lost-s", type=float, default=180, help="seconds without GPS")
    ap.add_argument("--vio-wait-s", type=float, default=150, help="give up if no VIO this long after AUTO")
    ap.add_argument("--no-vio", type=float, metavar="S",
                    help="flight without the VIO (setup/run_vio_live.sh not running): GPS off S s after AUTO")
    ap.add_argument("--wind", help="SPEED,FROM_DEG wind from the start (Gazebo and SITL)")
    ap.add_argument("--wind-shift", help="S,SPEED,FROM_DEG: S seconds after the GPS loss the wind changes to this")
    ap.add_argument("--far", help="N,E: fly there once the VIO is in and lose the GPS there")
    ap.add_argument("--far-radius", type=float, default=150, help="GPS off within this distance of --far (m)")
    ap.add_argument("--camera-stall", help="S,DUR: S s after the GPS loss the camera stops for DUR s (IMU goes on)")
    ap.add_argument("--kill-vio", type=float, help="S: S s after the GPS loss OpenVINS is killed (a crash)")
    ap.add_argument("--mission", default="rect", choices=["rect", "weave"], help="the AUTO mission")
    ap.add_argument("--restart-at-legs", action="store_true", help="fresh VIO at the start of every leg")
    ap.add_argument("--param", action="append", default=[], help="NAME=VALUE, set before take-off (repeatable)")
    args = ap.parse_args()
    shift = [float(v) for v in args.wind_shift.split(",")] if args.wind_shift else None
    far = [float(v) for v in args.far.split(",")] if args.far else None
    stall = [float(v) for v in args.camera_stall.split(",")] if args.camera_stall else None
    t_stall = None
    kill_at = args.kill_vio

    print(f"connecting to the simulator on tcp:127.0.0.1:{args.port} ...")
    m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{args.port}", source_system=255)
    m.wait_heartbeat(timeout=60)
    sitl.set_rates(m, {ML.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 4, ML.MAVLINK_MSG_ID_VFR_HUD: 2,
                       ML.MAVLINK_MSG_ID_SYS_STATUS: 1, ML.MAVLINK_MSG_ID_MISSION_CURRENT: 1,
                       ML.MAVLINK_MSG_ID_SIMSTATE: 4})
    start_wind = [float(v) for v in args.wind.split(",")] if args.wind else None
    for item in args.param:
        name, value = item.split("=")
        sitl.set_param(m, name, float(value))
        print(f"parameter {name} = {value}")
    print("waiting for pre-arm checks ...")
    wait_prearm(m)
    corners, leg_starts = weave_mission() if args.mission == "weave" else (None, {})
    upload_mission(m, corners)
    last_wp = None
    m.set_mode(m.mode_mapping()["FBWA"])
    arm(m)
    print("armed in FBWA - full throttle for take-off")

    st = dict(alt=0.0, gs=0.0, aspd=0.0, ne=(0.0, 0.0), true_ne=None, wp=0, vsw_st=None)
    t0, phase, last_print, last_rc = time.time(), "takeoff", 0.0, 0.0
    t_vio = t_off = t_on = None
    rows = []   # (time since GPS off, ArduPilot's distance from home, true distance, error between them)
    while True:
        now = time.time() - t0
        if phase == "takeoff" and now - last_rc > 0.5:
            throttle_override(m, 1800)
            last_rc = now
        if phase == "takeoff" and now > 45 and st["alt"] < 10:
            print(f"{now:5.0f} s  take-off failed ({st['alt']:.0f} m up) - start a fresh simulation and try again")
            break
        if phase == "takeoff" and (st["alt"] > 40 or now > 45):
            m.set_mode(m.mode_mapping()["CIRCLE"])
            phase, t_phase = "circle", now
            print(f"{now:5.0f} s  {st['alt']:.0f} m up - CIRCLE")
            if start_wind:
                set_wind(m, *start_wind)
                print(f"{now:5.0f} s  wind {start_wind[0]:.1f} m/s from {start_wind[1]:.0f} deg (Gazebo and SITL)")
        elif phase == "circle" and now - t_phase > 15:
            throttle_override(m, 0)
            m.mav.mission_set_current_send(m.target_system, m.target_component, 2)
            m.set_mode(m.mode_mapping()["AUTO"])
            phase, t_auto = "auto", now
            print(f"{now:5.0f} s  AUTO - flying the rectangle, waiting for the VIO")
        elif phase == "auto" and args.no_vio is not None and t_vio is None and now - t_auto > args.no_vio - args.gps_s:
            t_vio = now
            print(f"{now:5.0f} s  no VIO in this flight - GPS off in {args.gps_s:.0f} s")
        elif phase == "auto" and t_vio is None and now - t_auto > args.vio_wait_s:
            print(f"{now:5.0f} s  no VIO after {args.vio_wait_s:.0f} s in AUTO - see the logs of setup/run_vio_live.sh")
            break
        elif phase == "auto" and t_vio is not None and far:
            lat, lon = offset(*far)
            m.mav.command_int_send(m.target_system, m.target_component, ML.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                   ML.MAV_CMD_DO_REPOSITION, 0, 0, -1, ML.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                                   0, 0, int(lat * 1e7), int(lon * 1e7), CRUISE_ALT)
            phase, t_out = "outbound", now
            print(f"{now:5.0f} s  GUIDED - flying out to N {far[0]:.0f} E {far[1]:.0f} "
                  f"({math.hypot(*far):.0f} m from home) to lose the GPS there")
        elif ((phase == "auto" and t_vio is not None and now - t_vio > args.gps_s) or
              (phase == "outbound" and now - t_vio > args.gps_s and
               (math.hypot(st["ne"][0] - far[0], st["ne"][1] - far[1]) < args.far_radius or now - t_out > 240))):
            sitl.set_param(m, "SIM_GPS1_ENABLE", 0)
            phase, t_off = "lost", now
            print(f"{now:5.0f} s  ===== GPS OFF ===== ({math.hypot(*st['ne']):.0f} m from home)")
        elif phase == "lost" and stall and t_stall is None and now - t_off > stall[0]:
            wsl("pkill -STOP -f '[r]os_rgb_to_mono'")
            t_stall = now
            print(f"{now:5.0f} s  ===== CAMERA STOPPED (IMU goes on) =====")
        elif phase == "lost" and stall and t_stall is not None and now - t_stall > stall[1]:
            wsl("pkill -CONT -f '[r]os_rgb_to_mono'")
            stall = None
            print(f"{now:5.0f} s  ===== CAMERA BACK =====")
        elif phase == "lost" and kill_at is not None and now - t_off > kill_at:
            wsl("pkill -KILL -f 'lib/ov_msckf/[r]un_subscribe_msckf'")
            kill_at = None
            print(f"{now:5.0f} s  ===== OpenVINS KILLED =====")
        elif phase == "lost" and shift and now - t_off > shift[0]:
            set_wind(m, shift[1], shift[2])
            print(f"{now:5.0f} s  ===== WIND NOW {shift[1]:.1f} m/s FROM {shift[2]:.0f} deg =====")
            shift = None
        elif phase == "lost" and now - t_off > args.lost_s:
            sitl.set_param(m, "SIM_GPS1_ENABLE", 1)
            phase, t_on = "back", now
            print(f"{now:5.0f} s  ===== GPS ON =====")
        elif phase == "back" and now - t_on > 20:
            break

        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            st["alt"] = msg.relative_alt / 1000
            st["ne"] = local_ne(msg.lat / 1e7, msg.lon / 1e7)
        elif kind == "SIMSTATE":
            st["true_ne"] = local_ne(msg.lat / 1e7, msg.lng / 1e7)
        elif kind == "VFR_HUD":
            st["gs"], st["aspd"] = msg.groundspeed, msg.airspeed
        elif kind == "MISSION_CURRENT":
            st["wp"] = msg.seq
            if msg.seq != last_wp and msg.seq in leg_starts and phase != "takeoff":
                print(f"{now:5.0f} s  ===== LEG: {leg_starts[msg.seq]} =====")
                if args.restart_at_legs and t_vio is not None:
                    wsl("pkill -KILL -f 'lib/ov_msckf/[r]un_subscribe_msckf'")
            last_wp = msg.seq
        elif kind == "NAMED_VALUE_FLOAT" and msg.name == "VSW_ST":
            st["vsw_st"] = int(msg.value)
        elif kind == "STATUSTEXT":
            print(f"{now:5.0f} s  vehicle: {msg.text}")
        # the VIO is in when the companion says so, or when ArduPilot reports visual odometry healthy
        vio_in = ((kind == "STATUSTEXT" and msg.text.startswith(("VIO: started", "VisOdom"))) or
                  (kind == "SYS_STATUS" and msg.onboard_control_sensors_health & ML.MAV_SYS_STATUS_SENSOR_VISION_POSITION))
        if vio_in and t_vio is None and phase != "takeoff":
            t_vio = now
            print(f"{now:5.0f} s  VIO is reaching the flight controller - {args.gps_s:.0f} s more on GPS")
        if now - last_print >= 5 and phase != "takeoff":
            n, e = st["ne"]
            line = (f"{now:5.0f} s  {m.flightmode:7} alt {st['alt']:5.0f} m  ground {st['gs']:4.1f} m/s  "
                    f"air {st['aspd']:4.1f} m/s  ArduPilot: {math.hypot(n, e):5.0f} m from home")
            if st["true_ne"] is not None:
                tn, te = st["true_ne"]
                err = math.hypot(n - tn, e - te)
                line += f"  really: {math.hypot(tn, te):5.0f} m  (off by {err:4.0f} m)"
                if phase in ("lost", "back"):
                    rows.append((now - t_off, math.hypot(n, e), math.hypot(tn, te), err))
            src = {0: "GPS", 1: "VIO", 2: "dead reckoning"}.get(st["vsw_st"], "-")
            print(line + f"  nav: {src}")
            last_print = now

    lost = [r for r in rows if r[0] <= args.lost_s]
    last_half = [r for r in lost if r[0] > args.lost_s / 2]
    if lost and last_half:
        print(f"\nwithout GPS ({args.lost_s:.0f} s): ArduPilot's position off by up to {max(r[3] for r in lost):.0f} m "
              f"(mean {sum(r[3] for r in lost) / len(lost):.0f} m); in the second half the plane was really "
              f"{min(r[2] for r in last_half):.0f}-{max(r[2] for r in last_half):.0f} m from home")
    print("done - the plane keeps flying; stop the simulator with setup/stop_gazebo_vio.sh")


if __name__ == "__main__":
    main()
