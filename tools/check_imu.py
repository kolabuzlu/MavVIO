"""Sanity-check the IMU (and camera timing) in a ROS 2 (sqlite3) recording.

Over a period where the vehicle stands still: the accelerometer should read about 9.8 m/s^2 (gravity
reaction) and the gyroscope about 0 rad/s. Also checks the IMU rate/regularity and how camera and
IMU timestamps relate.

Usage:  python check_imu.py BAG_DIR [STILL_FROM_S STILL_TO_S] [--imu /imu0] [--cam /cam0/image_raw]
"""
import argparse
import sqlite3
from pathlib import Path

import numpy as np
from rosbags.typesys import Stores, get_typestore


def open_db(bag_dir):
    path = next(Path(bag_dir).glob("*.db3")).as_posix()
    uri = f"file://{path}" if path.startswith("//") else f"file:/{path}"
    return sqlite3.connect(f"{uri}?mode=ro&immutable=1", uri=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("still_from", nargs="?", type=float, default=0.0)
    ap.add_argument("still_to", nargs="?", type=float, default=10.0)
    ap.add_argument("--imu", default="/imu0")
    ap.add_argument("--cam", default="/cam0/image_raw")
    args = ap.parse_args()
    con = open_db(args.bag)
    store = get_typestore(Stores.ROS2_HUMBLE)
    topics = {name: (tid, typ) for tid, name, typ in con.execute("SELECT id, name, type FROM topics")}
    start = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

    tid, typ = topics[args.imu]
    rows = con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (tid,)).fetchall()
    stamps, rec, acc, gyr = [], [], [], []
    for t_rec, data in rows:
        m = store.deserialize_cdr(data, typ)
        stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        rec.append(t_rec * 1e-9)
        acc.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
        gyr.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    stamps, rec, acc, gyr = map(np.array, (stamps, rec, acc, gyr))
    t_rel = rec - start * 1e-9
    still = (t_rel >= args.still_from) & (t_rel <= args.still_to)
    dt = np.diff(stamps)
    print(f"IMU {args.imu}: {len(stamps)} samples, rate {1 / np.median(dt):.1f} Hz, "
          f"gaps > 2x median: {int(np.sum(dt > 2 * np.median(dt)))}, non-increasing stamps: {int(np.sum(dt <= 0))}")
    a, g = acc[still], gyr[still]
    print(f"standing still ({args.still_from:.0f}-{args.still_to:.0f} s, {len(a)} samples):")
    print(f"  accelerometer mean  x {a[:, 0].mean():7.3f}  y {a[:, 1].mean():7.3f}  z {a[:, 2].mean():7.3f}   "
          f"|a| = {np.linalg.norm(a.mean(axis=0)):.3f} m/s^2  (should be ~9.8)")
    print(f"  accelerometer std   x {a[:, 0].std():7.3f}  y {a[:, 1].std():7.3f}  z {a[:, 2].std():7.3f}   (vibration)")
    print(f"  gyroscope mean      x {g[:, 0].mean():7.4f}  y {g[:, 1].mean():7.4f}  z {g[:, 2].mean():7.4f}   (should be ~0)")
    print(f"  gyroscope std       x {g[:, 0].std():7.4f}  y {g[:, 1].std():7.4f}  z {g[:, 2].std():7.4f}")
    moving = t_rel > args.still_to + 20
    print(f"  gyroscope max while flying: {np.abs(gyr[moving]).max():.2f}  (rad/s would be ~0.5-3; deg/s would be ~30-170)")

    tid, typ = topics[args.cam]
    cam = []
    for t_rec, data in con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp LIMIT 400", (tid,)):
        m = store.deserialize_cdr(data, typ)
        cam.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, t_rec * 1e-9))
    cam = np.array(cam)
    print(f"camera {args.cam}: rate {1 / np.median(np.diff(cam[:, 0])):.1f} Hz")
    print(f"  recorder delay (arrival - stamp): camera median {1000 * np.median(cam[:, 1] - cam[:, 0]):.1f} ms, "
          f"IMU median {1000 * np.median(rec - stamps):.1f} ms")


if __name__ == "__main__":
    main()
