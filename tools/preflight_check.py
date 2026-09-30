#!/usr/bin/env python3
"""Pre-flight check of the VIO setup: the flight controller's settings, the Jetson's free space.

Connects to the flight controller, reads the parameters the VIO switch-over depends on and says what is off,
in a table and in one line to the ground station (STATUSTEXT "VIO check: ..."). Nothing is changed. Run by
setup/run_vio_d455.sh before the companion starts (it only warns - the pilot decides), or by hand:
  python3 preflight_check.py [--mavlink /dev/ttyTHS0,921600] [--record]
In the simulator: --mavlink tcp:127.0.0.1:5763 (while nothing else uses that port).
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

from pymavlink import mavutil

WANT = [   # parameter, wanted value (None = only shown), why
    ("AHRS_EKF_TYPE", 3, "EKF3 (the source sets are EKF3's)"),
    ("VISO_TYPE", 1, "MAVLink visual odometry, taken as it comes (2 turns the VIO frame at the start)"),
    ("EK3_SRC1_POSXY", 3, "set 1 = GPS"),
    ("EK3_SRC1_VELXY", 3, "set 1 = GPS"),
    ("EK3_SRC2_POSXY", 6, "set 2 = the VIO (ExternalNav)"),
    ("EK3_SRC2_VELXY", 6, "set 2 = the VIO (ExternalNav)"),
    ("EK3_SRC2_POSZ", 1, "set 2 height from the barometer"),
    ("EK3_SRC2_YAW", 1, "set 2 heading from the compass"),
    ("SCR_ENABLE", 1, "Lua scripting on (for vio_gps_switch.lua)"),
    ("VSW_ENABLE", None, "the switch-over script (1 = switches on GPS loss, 0 = shadow flight)"),
    ("VSW_GPS_RTL", None, "1 = RTL when switching to the VIO"),
    ("ARMING_SKIPCHK", None, "should include 262144 (skip only the visual odometry check - it starts in the air)"),
    ("VISO_DELAY_MS", None, "VIO delay (the companion's age_ms + the link)"),
    ("VISO_POS_M_NSE", None, "VIO position noise"),
    ("VISO_VEL_M_NSE", None, "VIO speed noise"),
]


def get_params(m, names, timeout=3.0):
    got = {}
    for name in names:
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        end = time.time() + timeout
        while time.time() < end:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None:
                pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
                got[pid.rstrip("\x00")] = msg.param_value
                if name in got:
                    break
    return got


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", default=os.environ.get("MAVLINK", "/dev/ttyTHS0,921600"))
    ap.add_argument("--record", action="store_true", help="recording is on: more free space needed")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/vio_logs"))
    a = ap.parse_args()

    print(f"connecting to the flight controller on {a.mavlink} ...")
    m = mavutil.mavlink_connection(a.mavlink, source_system=1, source_component=197)
    if m.wait_heartbeat(timeout=10) is None:
        print("no heartbeat from the flight controller - check the cable, the port and SERIALx_PROTOCOL/BAUD")
        sys.exit(1)
    m.target_component = 1
    serials = [f"SERIAL{i}_{k}" for i in range(1, 9) for k in ("PROTOCOL", "BAUD")]
    got = get_params(m, [n for n, _, _ in WANT] + serials)

    problems, rows = [], []
    for name, want, why in WANT:
        have = got.get(name)
        if have is None:
            state = "missing" if name.startswith("VSW_") or want is not None else "-"
            if name.startswith("VSW_") or want is not None:
                problems.append(f"{name} missing")
        elif want is not None and int(round(have)) != want:
            state = f"{have:g} (want {want})"
            problems.append(f"{name} {have:g} not {want}")
        elif name == "ARMING_SKIPCHK" and not (int(round(have)) & 262144):
            state = f"{have:g} (no 262144)"
            problems.append("ARMING_SKIPCHK lacks 262144")
        else:
            state = f"{have:g}"
        rows.append((name, state, why))
    links = [i for i in range(1, 9) if got.get(f"SERIAL{i}_PROTOCOL") == 2 and got.get(f"SERIAL{i}_BAUD") == 921]
    rows.append(("MAVLink port at 921600", ", ".join(f"SERIAL{i}" for i in links) or "none",
                 "the Jetson's port (SERIALx_PROTOCOL 2, BAUD 921)"))
    if not links and "," in a.mavlink and a.mavlink.startswith("/dev/tty"):
        problems.append("no SERIALx at 921600 for MAVLink")

    os.makedirs(a.log_dir, exist_ok=True)
    free_gb = shutil.disk_usage(a.log_dir).free / 1e9
    need = 20 if a.record else 2
    rows.append(("free space for logs", f"{free_gb:.0f} GB", f"at least {need} GB" + (" (recording)" if a.record else "")))
    if free_gb < need:
        problems.append(f"only {free_gb:.0f} GB free")
    try:
        mode = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, timeout=5).stdout.strip().splitlines()
        rows.append(("Jetson power mode", mode[0] if mode else "?", "the fastest (MAXN SUPER) for OpenVINS"))
    except (OSError, subprocess.TimeoutExpired):
        pass

    width = max(len(r[0]) for r in rows)
    for name, state, why in rows:
        print(f"  {name:{width}}  {state:18}  {why}")
    summary = "VIO check: OK" if not problems else "VIO check: " + "; ".join(problems)
    print(summary)
    m.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO if not problems else mavutil.mavlink.MAV_SEVERITY_WARNING,
                          summary.encode()[:50])
    time.sleep(0.5)


if __name__ == "__main__":
    main()
