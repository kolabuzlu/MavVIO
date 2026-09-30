"""Start a Gazebo flight for you to take over: take off, fly straight (the VIO starts), loiter over home.

Needs setup/run_gazebo_vio.sh and setup/run_vio_live.sh running inside Ubuntu. Takes off like
gz_fly_zephyr.py (FBWA, full throttle, CIRCLE at 40 m), uploads the test rectangle as the mission (for AUTO),
flies GUIDED straight towards home at --alt - the VIO companion only starts OpenVINS on a straight, level
leg - and switches to LOITER over home. Then it disconnects, so the simulator's MAVLink ports are free for
Mission Planner or MAVProxy.

Usage:  python gz_takeoff_loiter.py [--alt 100]
"""
import argparse
import math
import time

from pymavlink import mavutil

import sitl
from gz_fly_zephyr import arm, throttle_override, wait_prearm
from test_gps_loss import local_ne, offset, upload_mission

ML = mavutil.mavlink


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5760)
    ap.add_argument("--alt", type=float, default=100, help="loiter height over home (m)")
    args = ap.parse_args()

    print(f"connecting to the simulator on tcp:127.0.0.1:{args.port} ...")
    m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{args.port}", source_system=255)
    m.wait_heartbeat(timeout=60)
    sitl.set_rates(m, {ML.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 4, ML.MAVLINK_MSG_ID_SYS_STATUS: 1})
    print("waiting for pre-arm checks ...")
    wait_prearm(m)
    upload_mission(m)
    m.set_mode(m.mode_mapping()["FBWA"])
    arm(m)
    print("armed in FBWA - full throttle for take-off")

    alt, ne, vio = 0.0, (0.0, 0.0), False
    t0, phase, last_rc, last_print = time.time(), "takeoff", 0.0, 0.0
    while True:
        now = time.time() - t0
        if phase == "takeoff" and now - last_rc > 0.5:
            throttle_override(m, 1800)
            last_rc = now
        if phase == "takeoff" and (alt > 40 or now > 45):
            m.set_mode(m.mode_mapping()["CIRCLE"])
            phase, t_phase = "circle", now
            print(f"{now:5.0f} s  {alt:.0f} m up - CIRCLE")
        elif phase == "circle" and now - t_phase > 15:
            throttle_override(m, 0)                     # hand the throttle back
            lat, lon = offset(0, 0)
            m.mav.command_int_send(m.target_system, m.target_component, ML.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                   ML.MAV_CMD_DO_REPOSITION, 0, 0, -1, ML.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
                                   0, 0, int(lat * 1e7), int(lon * 1e7), args.alt)
            phase, t_phase = "guided", now
            print(f"{now:5.0f} s  GUIDED - straight towards home, climbing to {args.alt:.0f} m (the VIO starts on the way)")
        elif phase == "guided" and alt > args.alt - 10 and math.hypot(*ne) < 250 and (vio or now - t_phase > 150):
            m.set_mode(m.mode_mapping()["LOITER"])
            phase, t_phase = "loiter", now
            print(f"{now:5.0f} s  LOITER over home at {alt:.0f} m" + ("" if vio else " - the VIO has not started"))
        elif phase == "loiter" and now - t_phase > 8:
            break

        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            alt, ne = msg.relative_alt / 1000, local_ne(msg.lat / 1e7, msg.lon / 1e7)
        elif kind == "STATUSTEXT":
            print(f"{now:5.0f} s  vehicle: {msg.text}")
            vio = vio or msg.text.startswith("VIO: started")
        if now - last_print >= 5 and phase != "takeoff":
            print(f"{now:5.0f} s  {m.flightmode:7} alt {alt:5.0f} m  {math.hypot(*ne):5.0f} m from home")
            last_print = now
    m.close()
    print("done - the plane is loitering over home; this script has let go of the simulator")


if __name__ == "__main__":
    main()
