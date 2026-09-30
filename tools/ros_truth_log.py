#!/usr/bin/env python3
"""Logs the simulator's true pose (/vio/truth, the plane's pose from Gazebo, 100 per second) with its own time
stamps into OUT_DIR/truth.csv, and - with --odom - the live estimates of OpenVINS variants (/ov_v_NAME/odomimu)
into OUT_DIR/odom_NAME.csv, for scoring VIO runs (tools/compare_variants.py).

The estimates are logged here rather than taken from OpenVINS's own files: those are written through a buffer,
and an OpenVINS stopped with SIGKILL (the companion's restart) loses the last half minute of them. Each VIO
start is a new "run" (a gap of more than 2 s between estimates); velocities are turned into the global frame.

With --truth-topic /fc/odom the reference is instead the flight controller's GPS-aided estimate
(tools/vio_bridge.py publishes it) - for replaying a real flight's recording.

Runs inside Ubuntu with ROS 2 loaded:  python3 ros_truth_log.py OUT_DIR [--odom NAME ...] [--truth-topic T]
"""
import argparse
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

KEEP_EVERY_S = 0.05          # the odometry comes at the IMU rate (400 per second) - 20 per second is plenty


def rotate(q, v):
    """v turned by the unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 q_vec x v ; v' = v + w t + q_vec x t
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


class Log(Node):
    def __init__(self, out, names, truth_topic="/vio/truth"):
        super().__init__("truth_log")
        out.mkdir(parents=True, exist_ok=True)
        self.truth = open(out / "truth.csv", "w")
        self.truth.write("t,x,y,z,qx,qy,qz,qw\n")
        self.n = 0
        if truth_topic == "/vio/truth":
            self.create_subscription(TransformStamped, truth_topic, self.on_truth, 100)
        else:
            self.create_subscription(Odometry, truth_topic, self.on_truth_odom, 100)
        self.odom = {}
        for name in names:
            f = open(out / f"odom_{name}.csv", "w")
            f.write("t,run,x,y,z,vx,vy,vz\n")
            self.odom[name] = dict(f=f, last=None, run=0, n=0)
            self.create_subscription(Odometry, f"/ov_v_{name}/odomimu", lambda m, name=name: self.on_odom(name, m), 100)

    def on_truth(self, m):
        p, q = m.transform.translation, m.transform.rotation
        self.truth.write(f"{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.4f},{p.x:.3f},{p.y:.3f},{p.z:.3f},"
                         f"{q.x:.5f},{q.y:.5f},{q.z:.5f},{q.w:.5f}\n")
        self.n += 1
        if self.n % 500 == 0:
            self.truth.flush()

    def on_truth_odom(self, m):
        p = m.pose.pose.position
        self.truth.write(f"{m.header.stamp.sec + m.header.stamp.nanosec * 1e-9:.4f},{p.x:.3f},{p.y:.3f},{p.z:.3f},0,0,0,1\n")
        self.n += 1
        if self.n % 100 == 0:
            self.truth.flush()

    def on_odom(self, name, m):
        s = self.odom[name]
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if s["last"] is not None and t - s["last"] < KEEP_EVERY_S and t >= s["last"]:
            return
        if s["last"] is None or t - s["last"] > 2 or t < s["last"]:
            s["run"] += 1                                          # a new VIO start
        s["last"] = t
        p, o, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        vg = rotate((o.x, o.y, o.z, o.w), (v.x, v.y, v.z))         # OpenVINS gives the velocity in its IMU frame
        s["f"].write(f"{t:.4f},{s['run']},{p.x:.3f},{p.y:.3f},{p.z:.3f},{vg[0]:.3f},{vg[1]:.3f},{vg[2]:.3f}\n")
        s["n"] += 1
        if s["n"] % 100 == 0:
            s["f"].flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--odom", nargs="*", default=[], help="variant names whose /ov_v_NAME/odomimu to log")
    ap.add_argument("--truth-topic", default="/vio/truth", help="/vio/truth (simulator) or /fc/odom (a recording)")
    a = ap.parse_args()
    rclpy.init()
    node = Log(Path(a.out_dir), a.odom, a.truth_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.truth.close()
        for s in node.odom.values():
            s["f"].close()


if __name__ == "__main__":
    main()
