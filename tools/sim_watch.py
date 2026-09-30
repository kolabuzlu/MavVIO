#!/usr/bin/env python3
"""Watches the Linux VM's free memory during a Gazebo session and warns the pilot before it runs out.

Under WSL the Gazebo server leaks graphics memory (~0.2 MB for every camera picture it draws), and when the
Linux VM runs out of memory it stops - with the simulated plane in it. This estimates the minutes left from
the last two minutes, sends warnings to the ground station through the simulator (STATUSTEXT on TCP 5760 -
ArduPilot passes them on to Mission Planner / MAVProxy on 5762) at 10, 5 and 2 minutes left, and stops the
simulation cleanly (setup/stop_gazebo_vio.sh) when less than --stop-mb is left. Also logs to stdout.
It watches the Windows C: drive too: Ubuntu's disk and the logs live there, and when C: was full the Linux
filesystem went read-only in the middle of a flight (2026-09-29) - warns below 1.5 GB, stops below --stop-c-gb.

Runs inside Ubuntu (pymavlink from ~/venv-ardupilot):
  PYTHONPATH=~/venv-ardupilot/lib/python3.10/site-packages python3 sim_watch.py [--stop-mb 500] [--stop-c-gb 0.8]
"""
import argparse
import os
import subprocess
import time

from pymavlink import mavutil


def available_mb():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024
    return 0.0


def c_drive_free_gb():
    s = os.statvfs("/mnt/c")
    return s.f_bavail * s.f_frsize / 1e9


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5760")
    ap.add_argument("--stop-mb", type=float, default=500)
    ap.add_argument("--stop-c-gb", type=float, default=0.8)
    a = ap.parse_args()

    state = {"mav": None}

    def link():
        if state["mav"] is None:
            try:
                state["mav"] = mavutil.mavlink_connection(a.mavlink, source_system=254, source_component=250)
            except OSError:
                pass
        return state["mav"]

    def tell(text, severity=mavutil.mavlink.MAV_SEVERITY_WARNING):
        print(text, flush=True)
        try:
            if link() is not None:
                link().mav.statustext_send(severity, text.encode())
        except OSError:
            state["mav"] = None

    history, warned, started = [], set(), time.time()
    while True:
        now, avail = time.time(), available_mb()
        history = [(t, v) for t, v in history if now - t < 120] + [(now, avail)]
        # MB per minute over the last 2 minutes: the slope of a straight line through all samples - not just the
        # oldest and newest, which a process freeing or taking a few hundred MB at once (a VIO restart) throws off
        if len(history) >= 3:
            ts, vs = [t - history[0][0] for t, _ in history], [v for _, v in history]
            tm, vm = sum(ts) / len(ts), sum(vs) / len(vs)
            rate = -sum((t - tm) * (v - vm) for t, v in zip(ts, vs)) / max(sum((t - tm) ** 2 for t in ts), 1e-9) * 60
        else:
            rate = 0.0
        left = (avail - a.stop_mb) / rate if rate > 1 else float("inf")
        c_free = c_drive_free_gb()
        print(f"{time.strftime('%H:%M:%S')}  {avail:5.0f} MB free, using {rate:4.0f} MB/min, about {left:5.1f} min left;"
              f" C: {c_free:4.1f} GB free", flush=True)
        for mark in (10, 5, 2):
            if left < mark and mark not in warned and now - started > 180:     # not the start-up ramp
                warned.add(mark)
                tell(f"SIM: memory for about {mark} min more - then it stops")
        if c_free < 1.5 and "disk" not in warned:
            warned.add("disk")
            tell(f"SIM: C: drive almost full ({c_free:.1f} GB) - stops below {a.stop_c_gb:.1f} GB")
        if avail < a.stop_mb or c_free < a.stop_c_gb:
            why = "out of memory" if avail < a.stop_mb else "C: drive full"
            tell(f"SIM: {why} - stopping now", mavutil.mavlink.MAV_SEVERITY_CRITICAL)
            subprocess.run("bash /mnt/c/Users/funfo/vio/setup/stop_gazebo_vio.sh", shell=True)
            return
        try:
            while link() is not None and link().recv_match(blocking=False) is not None:
                pass                                      # keep the link drained
        except OSError:
            state["mav"] = None
        time.sleep(10)


if __name__ == "__main__":
    main()
