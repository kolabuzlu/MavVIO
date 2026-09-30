#!/usr/bin/env python3
"""True state of the VIO IMU from a Gazebo recording, for starting OpenVINS in the air and for scoring.

The recording's /vio/truth is the pose of the plane (wing frame). The VIO IMU sits in the camera module
(camera_link in tools/make_scenery.py: 0 -0.10 -0.03, turned 0 90 -90 deg from the wing), so this works
out that module's position, orientation and velocity and writes them in the EuRoC ground-truth format
OpenVINS reads (-p path_gt:=FILE -p init_from_gt:=true, see setup/openvins_init_from_state.py).
Gyro bias = its average while the plane stood still before take-off (a real VIO box can do the same);
accelerometer bias unknown (0).

Checks the frame maths against the recorded VIO IMU: at rest it must measure gravity in the direction
the true orientation predicts, and in flight its gyro must match the turn rates of the true orientation.

Also picks the moment to start the VIO: the first time the plane is up (> --min-height) and flying level
(nose within 20 deg of the horizon), plus --after-s. Prints the matching `ros2 bag play --start-offset`.

Runs inside Ubuntu with ROS 2 loaded:
  python3 /mnt/c/Users/funfo/vio/tools/bag_gt_init.py ~/datasets/gazebo/NAME OUT.csv [--after-s 5]
"""
import argparse

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

LEVER = np.array([0.0, -0.10, -0.03])       # camera module position in the wing frame
G = 9.80


def rpy_to_R(r, p, y):
    """SDF roll-pitch-yaw (fixed axes x, y, z) to a rotation matrix."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


R_WING_IMU = rpy_to_R(0.0, np.pi / 2, -np.pi / 2)


def quat_to_R(q):
    """(x, y, z, w) Hamilton quaternions, N x 4 -> N x 3 x 3 rotation matrices."""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], -2)


def R_to_quat(R):
    """N x 3 x 3 rotation matrices -> (w, x, y, z), w >= 0."""
    out = []
    for m in R:
        tr = np.trace(m)
        if tr > 0:
            s = 2 * np.sqrt(tr + 1)
            q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        else:
            i = int(np.argmax(np.diag(m)))
            j, k = (i + 1) % 3, (i + 2) % 3
            s = 2 * np.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
            q = [0.0] * 4
            q[0] = (m[k, j] - m[j, k]) / s
            q[1 + i] = 0.25 * s
            q[1 + j] = (m[j, i] + m[i, j]) / s
            q[1 + k] = (m[k, i] + m[i, k]) / s
        q = np.array(q)
        out.append(q if q[0] >= 0 else -q)
    return np.array(out)


def read_bag(path):
    try:
        reader = rosbag2_py.SequentialCompressionReader()
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    except Exception:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/vio/truth", "/vio/imu"]))
    truth, imu, first_bag_ns = [], [], None
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        first_bag_ns = bag_ns if first_bag_ns is None else first_bag_ns
        if topic == "/vio/truth":
            m = deserialize_message(data, TransformStamped)
            p, q = m.transform.translation, m.transform.rotation
            truth.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                          (bag_ns - first_bag_ns) * 1e-9])
        else:
            m = deserialize_message(data, Imu)
            imu.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    truth, imu = np.array(truth), np.array(imu)
    _, keep = np.unique(truth[:, 0], return_index=True)      # the pose publisher repeats stamps
    return truth[keep], imu


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--min-height", type=float, default=35.0)
    ap.add_argument("--after-s", type=float, default=5.0)
    a = ap.parse_args()

    truth, imu = read_bag(a.bag)
    t = truth[:, 0]
    R_wing = quat_to_R(truth[:, 4:8])
    R_imu = R_wing @ R_WING_IMU
    p_imu = truth[:, 1:4] + np.einsum("nij,j->ni", R_wing, LEVER)
    v_imu = np.gradient(p_imu, t, axis=0)
    k = 5                                                        # smooth the velocity over ~0.05 s
    v_imu = np.column_stack([np.convolve(v_imu[:, i], np.ones(k) / k, mode="same") for i in range(3)])

    # check 1: at rest the accelerometer measures gravity (pointing up) in the IMU frame
    moved = np.linalg.norm(truth[:, 1:4] - truth[0, 1:4], axis=1) > 0.05
    t_move = t[np.argmax(moved)]
    rest = (imu[:, 0] > t[0] + 1) & (imu[:, 0] < t_move - 1)
    acc_rest, gyro_bias = imu[rest, 4:7].mean(axis=0), imu[rest, 1:4].mean(axis=0)
    expected = R_imu[np.searchsorted(t, imu[rest, 0].mean())].T @ np.array([0, 0, G])
    ang = np.degrees(np.arccos(np.dot(acc_rest, expected) / np.linalg.norm(acc_rest) / np.linalg.norm(expected)))
    print(f"at rest ({rest.sum()} IMU samples): measured {np.round(acc_rest, 3)}, expected {np.round(expected, 3)}"
          f" -> {ang:.2f} deg apart (should be well under 1)")
    print(f"gyro bias from the rest: {np.round(gyro_bias, 5)} rad/s")

    # check 2: in flight the gyro measures the turn rate of the true orientation (in the IMU frame)
    fly = np.where(t > t_move + 5)[0][:-1]
    dR = np.einsum("nji,njk->nik", R_imu[fly], R_imu[fly + 1])   # R(t)^T R(t+dt)
    w_true = np.column_stack([dR[:, 2, 1] - dR[:, 1, 2], dR[:, 0, 2] - dR[:, 2, 0], dR[:, 1, 0] - dR[:, 0, 1]]) / 2
    w_true /= np.diff(t)[fly][:, None]
    tm = (t[fly] + t[fly + 1]) / 2
    w_meas = np.column_stack([np.interp(tm, imu[:, 0], imu[:, i]) for i in (1, 2, 3)]) - gyro_bias
    busy = np.linalg.norm(w_true, axis=1) > 0.05
    err = np.linalg.norm(w_meas[busy] - w_true[busy], axis=1)
    print(f"turning ({busy.sum()} samples): gyro vs true turn rate differ by {np.median(err):.4f} rad/s median "
          f"(turn rates up to {np.linalg.norm(w_true, axis=1).max():.2f} rad/s) - should be close to 0")

    # start time: up, level, then a few seconds
    fwd = -R_wing[:, :, 1]                                       # nose = -y of the wing frame
    pitch = np.degrees(np.arcsin(np.clip(fwd[:, 2], -1, 1)))
    ok = (truth[:, 3] - truth[0, 3] > a.min_height) & (np.abs(pitch) < 20) & (t > t_move)
    t_start = t[np.argmax(ok)] + a.after_s
    i0 = np.searchsorted(t, t_start)
    print(f"take-off at {t_move:.1f} s; level at {a.min_height:.0f} m+ at {t_start - a.after_s:.1f} s -> "
          f"start the VIO at {t_start:.1f} s (simulation time)")
    print(f"START_OFFSET {truth[i0, 8]:.2f}   (ros2 bag play --start-offset, seconds into the recording)")
    print(f"START_TIME {t_start:.3f}")

    q = R_to_quat(R_imu)
    with open(a.out, "w") as f:
        f.write("#timestamp [ns],p_x [m],p_y [m],p_z [m],q_w,q_x,q_y,q_z,v_x [m/s],v_y [m/s],v_z [m/s],"
                "bw_x [rad/s],bw_y [rad/s],bw_z [rad/s],ba_x [m/s^2],ba_y [m/s^2],ba_z [m/s^2]\n")
        for i in range(len(t)):
            row = [f"{int(round(t[i] * 1e9))}"] + [f"{v:.5f}" for v in p_imu[i]] + [f"{v:.7f}" for v in q[i]] + \
                  [f"{v:.4f}" for v in v_imu[i]] + [f"{v:.6f}" for v in gyro_bias] + ["0", "0", "0"]
            f.write(",".join(row) + "\n")
    print(f"{len(t)} states written to {a.out}")


if __name__ == "__main__":
    main()
