"""Boot the plane simulator with params/vio_plane.parm and scripts/vio_gps_switch.lua (no flight).

Checks: the script loads, its VSW_* parameters exist, visual odometry is healthy, the EKF uses GPS
(source set 1), pre-arm checks pass, and which parameter switches the simulated GPS off.
"""
import time

from pymavlink import mavutil

import sitl

PARAMS = sitl.ROOT / "params" / "vio_plane.parm"
SCRIPT = sitl.ROOT / "scripts" / "vio_gps_switch.lua"
ML = mavutil.mavlink

sitl.install_scripts(SCRIPT)
proc = sitl.start(extra_defaults=[PARAMS])
try:
    m = sitl.connect()
    sitl.set_rates(m, {ML.MAVLINK_MSG_ID_SYS_STATUS: 1, ML.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 1,
                       ML.MAVLINK_MSG_ID_GPS_RAW_INT: 1})
    t0, src, prearm_ok, ekf = time.time(), None, None, None
    while time.time() - t0 < 60:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "STATUSTEXT" and any(k in msg.text for k in ("VSW", "Lua", "script", "Script", "VisOdom", "external nav", "PreArm", "EKF3 IMU0")):
            print(f"[{time.time() - t0:5.1f} s] {msg.text}")
        elif kind == "NAMED_VALUE_FLOAT" and msg.name == "VSW_SRC":
            src = msg.value
        elif kind == "SYS_STATUS":
            prearm_ok = bool(msg.onboard_control_sensors_health & ML.MAV_SYS_STATUS_PREARM_CHECK)
        elif kind == "EKF_STATUS_REPORT":
            ekf = msg.flags

    print("\n--- parameters")
    for name in ("VSW_ENABLE", "VSW_SATS", "VSW_SACC", "VSW_BAD_MS", "VSW_GOOD_S", "VISO_TYPE", "SERIAL5_PROTOCOL", "GPS1_TYPE",
                 "EK3_SRC2_POSXY", "EK3_SRC_OPTIONS", "SCR_ENABLE", "SIM_GPS1_ENABLE", "SIM_GPS_DISABLE", "SIM_VICON_FAIL"):
        value = sitl.get_param(m, name)
        print(f"{name:18} {'(does not exist)' if value is None else round(value, 3)}")
    print(f"\nactive source set reported by the script: {src}")
    print(f"pre-arm checks passing: {prearm_ok} | EKF flags {ekf:#06x}")
finally:
    sitl.stop(proc)
