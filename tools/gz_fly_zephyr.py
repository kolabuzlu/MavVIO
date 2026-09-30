"""Gazebo step 1: fly the Zephyr in Gazebo (SITL started by setup/run_gazebo_zephyr.sh inside Ubuntu).

Follows the ardupilot_gazebo README - FBWA, arm, throttle up (the Zephyr takes off vertically), CIRCLE -
then flies the same rectangle mission as test_gps_loss.py in AUTO and reports what the plane does.
The plane is left flying at the end; stop the simulator with Ctrl+C in Ubuntu.

Usage:  python gz_fly_zephyr.py [--port 5760] [--fly-s 180]
"""
import argparse
import math
import time

from pymavlink import mavutil

import sitl
from test_gps_loss import local_ne, upload_mission

ML = mavutil.mavlink
IGNORE = 65535   # RC override: leave this channel alone


def throttle_override(m, pwm):
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    IGNORE, IGNORE, pwm, IGNORE, IGNORE, IGNORE, IGNORE, IGNORE)


def wait_prearm(m, timeout=180):
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type=["SYS_STATUS", "STATUSTEXT"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print("   ", msg.text)
        elif msg.onboard_control_sensors_health & ML.MAV_SYS_STATUS_PREARM_CHECK:
            return
    raise RuntimeError("pre-arm checks did not pass")


def arm(m, timeout=60):
    deadline = time.time() + timeout
    while time.time() < deadline:
        m.mav.command_long_send(m.target_system, m.target_component, ML.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        end = time.time() + 2
        while time.time() < end:
            msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print("   ", msg.text)
            elif msg.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED:
                return
    raise RuntimeError("could not arm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--fly-s", type=float, default=180, help="seconds to watch the flight in AUTO")
    args = ap.parse_args()

    print(f"connecting to the simulator on tcp:127.0.0.1:{args.port} ...")
    m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{args.port}", source_system=255)
    m.wait_heartbeat(timeout=60)
    sitl.set_rates(m, {ML.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 4, ML.MAVLINK_MSG_ID_VFR_HUD: 2,
                       ML.MAVLINK_MSG_ID_SYS_STATUS: 1, ML.MAVLINK_MSG_ID_MISSION_CURRENT: 1})
    print("waiting for pre-arm checks ...")
    wait_prearm(m)
    upload_mission(m)

    m.set_mode(m.mode_mapping()["FBWA"])
    arm(m)
    print("armed in FBWA - full throttle for take-off")
    state = dict(alt=0.0, gs=0.0, airspeed=0.0, ne=(0.0, 0.0), wp=0)
    t0, phase, last_print, last_rc = time.time(), "takeoff", 0.0, 0.0
    while True:
        now = time.time() - t0
        if phase == "takeoff" and now - last_rc > 0.5:
            throttle_override(m, 1800)
            last_rc = now
        if phase == "takeoff" and (state["alt"] > 40 or now > 45):
            m.set_mode(m.mode_mapping()["CIRCLE"])
            phase, t_phase = "circle", now
            print(f"{now:5.0f} s  {state['alt']:.0f} m up - CIRCLE")
        elif phase == "circle" and now - t_phase > 15:
            throttle_override(m, 0)     # hand the throttle back
            m.mav.mission_set_current_send(m.target_system, m.target_component, 2)   # first rectangle corner
            m.set_mode(m.mode_mapping()["AUTO"])
            phase, t_phase = "auto", now
            print(f"{now:5.0f} s  AUTO - flying the rectangle")
        elif phase == "auto" and now - t_phase > args.fly_s:
            break

        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            state["alt"] = msg.relative_alt / 1000
            state["ne"] = local_ne(msg.lat / 1e7, msg.lon / 1e7)
        elif kind == "VFR_HUD":
            state["gs"], state["airspeed"] = msg.groundspeed, msg.airspeed
        elif kind == "MISSION_CURRENT":
            state["wp"] = msg.seq
        elif kind == "STATUSTEXT":
            print(f"{now:5.0f} s  vehicle: {msg.text}")
        if now - last_print >= 5:
            n, e = state["ne"]
            print(f"{now:5.0f} s  {m.flightmode:7} alt {state['alt']:6.1f} m  ground speed {state['gs']:5.1f} m/s  "
                  f"{math.hypot(n, e):6.0f} m from home (N {n:6.0f}, E {e:6.0f})  waypoint {state['wp']}")
            last_print = now
    print("done - the plane keeps flying the rectangle; stop the simulator with Ctrl+C in Ubuntu")


if __name__ == "__main__":
    main()
