"""Sanity check: start the plane simulator, show its messages for 45 s, report version, GPS and EKF state."""
import time

from pymavlink import mavutil

import sitl

proc = sitl.start()
try:
    m = sitl.connect()
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                            0, mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    t0, version, gps, ekf = time.time(), None, None, None
    while time.time() - t0 < 45:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT":
            print(f"[{time.time() - t0:5.1f} s] {msg.text}")
        elif kind == "AUTOPILOT_VERSION":
            v = msg.flight_sw_version
            version = f"{v >> 24 & 0xFF}.{v >> 16 & 0xFF}.{v >> 8 & 0xFF}"
        elif kind == "GPS_RAW_INT":
            gps = (msg.fix_type, msg.satellites_visible)
        elif kind == "EKF_STATUS_REPORT":
            ekf = msg.flags
    print(f"\nfirmware {version} | GPS fix type {gps[0]} with {gps[1]} satellites | EKF flags {ekf:#06x}")
finally:
    sitl.stop(proc)
