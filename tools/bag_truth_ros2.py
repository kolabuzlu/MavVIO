#!/usr/bin/env python3
"""Write the true trajectory from a Gazebo recording (setup/record_gazebo_flight.sh) as a text file.

Reads the /vio/truth poses (geometry_msgs TransformStamped, simulation time) with ROS 2's own bag reader,
which also handles the compressed recordings, and writes  timestamp_s x y z qx qy qz qw  per line.
Runs inside Ubuntu with ROS 2 loaded:
  source /opt/ros/humble/setup.bash
  python3 /mnt/c/Users/funfo/vio/tools/bag_truth_ros2.py ~/datasets/gazebo/NAME OUTPUT.txt [--topic /vio/truth]
"""
import argparse

import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--topic", default="/vio/truth")
    args = ap.parse_args()
    try:
        reader = rosbag2_py.SequentialCompressionReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("", ""))
    except Exception:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    n = 0
    with open(args.out, "w") as f:
        f.write("# timestamp_s x y z qx qy qz qw  (Gazebo world: x east, y north, z up)\n")
        while reader.has_next():
            _, data, _ = reader.read_next()
            m = deserialize_message(data, TransformStamped)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, q = m.transform.translation, m.transform.rotation
            f.write(f"{t:.6f} {p.x:.4f} {p.y:.4f} {p.z:.4f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
            n += 1
    print(f"{n} poses written to {args.out}")


if __name__ == "__main__":
    main()
