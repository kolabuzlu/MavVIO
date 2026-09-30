"""Extract the ground-truth trajectory (e.g. Vicon motion capture) from a ROS 1 or ROS 2 recording.

Writes one line per measurement:  timestamp_s x y z qx qy qz qw   (the common "TUM" text format).
Handles geometry_msgs TransformStamped, PoseStamped and PointStamped topics, and GNSS fixes
(NavSatFix-style latitude/longitude/altitude, e.g. an RTK receiver), which are converted to metres
east/north/up from the first fix (orientation then left as identity).

Usage:  python bag_groundtruth.py RECORDING TOPIC OUTPUT.txt
  e.g.  python bag_groundtruth.py V1_01_easy.bag /vicon/firefly_sbx/firefly_sbx V1_01_easy_gt.txt
Without TOPIC/OUTPUT it lists the recording's topics.
"""
import math
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader

WGS84_A, WGS84_E2 = 6378137.0, 6.69437999014e-3


def ecef(lat_deg, lon_deg, alt):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    return ((n + alt) * math.cos(lat) * math.cos(lon), (n + alt) * math.cos(lat) * math.sin(lon),
            (n * (1 - WGS84_E2) + alt) * math.sin(lat))


def enu(origin, lat_deg, lon_deg, alt):
    """Metres east/north/up of (lat, lon, alt) relative to origin = (lat0, lon0, alt0)."""
    x0, y0, z0 = ecef(*origin)
    x, y, z = ecef(lat_deg, lon_deg, alt)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0, lon0 = math.radians(origin[0]), math.radians(origin[1])
    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = -math.sin(lat0) * math.cos(lon0) * dx - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz
    up = math.cos(lat0) * math.cos(lon0) * dx + math.cos(lat0) * math.sin(lon0) * dy + math.sin(lat0) * dz
    return east, north, up


def main(bag, topic=None, out=None):
    with AnyReader([Path(bag)]) as reader:
        if topic is None:
            for c in reader.connections:
                print(f"{c.topic:45} {c.msgtype:40} {c.msgcount:7d} messages")
            return
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            sys.exit(f"topic {topic} not in the recording")
        lines, origin = [], None
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if hasattr(msg, "latitude"):
                if getattr(getattr(msg, "status", None), "status", 0) < 0:
                    continue  # no fix
                origin = origin or (msg.latitude, msg.longitude, msg.altitude)
                e, n, u = enu(origin, msg.latitude, msg.longitude, msg.altitude)
                lines.append(f"{stamp:.9f} {e:.4f} {n:.4f} {u:.4f} 0 0 0 1")
                continue
            if hasattr(msg, "transform"):
                p, q = msg.transform.translation, msg.transform.rotation
            elif hasattr(msg, "pose"):
                p, q = msg.pose.position, msg.pose.orientation
            else:
                p, q = msg.point, None
            quat = (q.x, q.y, q.z, q.w) if q is not None else (0.0, 0.0, 0.0, 1.0)
            lines.append(f"{stamp:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} " + " ".join(f"{v:.6f}" for v in quat))
    Path(out).write_text("# timestamp_s x y z qx qy qz qw\n" + "\n".join(lines) + "\n")
    print(f"{len(lines)} ground-truth poses from {topic} -> {out}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
