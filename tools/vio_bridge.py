#!/usr/bin/env python3
"""The VIO companion program ("the Jetson program"): OpenVINS <-> flight controller over MAVLink.

It does what the program on the plane's Jetson will do:
  1. Start the VIO in the flight controller's frame. Once the plane flies steadily (straight, or in a
     steady turn such as a loiter) above --init-alt with a good GPS, the flight controller's own state
     (EKF attitude, position, velocity) is handed to OpenVINS as its starting state (/ov_msckf/init_state;
     OpenVINS change: setup/openvins_init_from_state.py). From then on the VIO and the EKF share one frame,
     so there is no jump when ArduPilot switches over, and OpenVINS needs no take-off jolt to start.
     The state is taken at the exact moment of a camera IMU sample: the flight controller's samples carry
     its own clock (time_boot_ms), the offset to the camera's clock is learnt from their arrival, and its
     attitude is turned on at its measured rates to that moment. Without this, in a turn the few
     hundredths of a second between the two cost the VIO a degree of heading.
  2. Send the VIO to the flight controller: OpenVINS's estimate (/ov_msckf/odomimu) goes out as
     VISION_POSITION_ESTIMATE + VISION_SPEED_ESTIMATE (north-east-down, --rate per second);
     ArduPilot reads them with VISO_TYPE 1, which takes them as they are (VISO_TYPE 2 would turn the
     whole VIO frame to match its heading at the first message - not wanted when it is already right).
  3. Keep the VIO fresh. A VIO drifts over a long flight (in a 20 minute loiter its speed error grew to
     3-4 m/s, and the switch-over script then rightly rejected it at the GPS loss). While the plane surely
     flies on GPS (the switch-over script reports VSW_ST = 0 and the GPS has been good for 10 s; without
     the script: GPS good for 30 s), OpenVINS is restarted and started again from the flight controller's
     state when
       - its speed differs from the flight controller's (GPS) speed at the same moment by more than
         --refresh-err for --refresh-s seconds of steady flight, or
       - it is older than --max-age-min (at a steady moment, so it can start again at once).
     The restart command (--restart-cmd) stops OpenVINS; its launcher (setup/run_vio_live.sh,
     setup/run_vio_d455.sh) starts it again. A VIO that stops by itself is noticed too (no estimate for
     3 s) and started again the same way. The pilot is told each time (STATUSTEXT "VIO: ..."), and
     ArduPilot through the messages' reset counter.
  4. Hold the VIO back when its camera stops. OpenVINS publishes its estimate on every IMU sample, so
     when the pictures stop (a USB hiccup) and the IMU goes on, it keeps coming - from the IMU alone,
     drifting quietly, and ArduPilot's IMU check cannot see it (it is the IMU agreeing with itself). Its
     /ov_msckf/poseimu comes only after a camera update: with none for --stale-s, nothing is sent (the
     switch-over script then sees the VIO as missing; ArduPilot flies on airspeed and its wind estimate).
     After a stall of --stall-restart-s or more, OpenVINS is not trusted to carry on: in the simulator it
     came back from a 40 s stall with a jump (20 m) or not at all. So once pictures flow again (the camera's
     own per-frame --camera-topic) it is restarted - and if the GPS is gone, started from ArduPilot's current
     estimate without GPS ("fresh start"; also after an OpenVINS crash during a GPS outage).

Frames: ArduPilot uses north-east-down and the plane's body frame (forward-right-down); OpenVINS uses
east-north-up at the EKF origin, and its IMU is the camera's IMU, fixed to the body as --mount says:
  gazebo          the Gazebo Zephyr's camera module: IMU axes down-left-forward, 10 cm forward, 3 cm down
  realsense-down  a RealSense (D455, D435i, D435if) looking straight down, top of the picture towards the
                  nose. realsense-ros gives the IMU in the camera's axes (x right, y down the picture, z out
                  of the lens). Where it sits relative to the flight controller: --lever FORWARD,RIGHT,DOWN (m)
While the GPS is good, the flight controller's own (GPS-aided) estimate also goes out as /fc/odom
(nav_msgs/Odometry, east-north-up like OpenVINS, stamped on the camera's clock), so a recording of a
flight (setup/run_vio_d455.sh RECORD=1) carries its own reference for replaying OpenVINS settings later.
Everything sent is logged to --log (CSV) with the flight controller's estimate at the same moment and, in
the simulator, the true position (/vio/truth), for scoring; column "run" counts the VIO starts, "sent" is 0 while
held back, "pos_std" is OpenVINS's own horizontal position uncertainty (m), "age_ms" is how old each
estimate is when it leaves. OpenVINS carries its estimate forward to the latest IMU sample, so this is
only the delivery delay (0-2 ms in the simulator) - the picture processing only makes its corrections come
a little later. ArduPilot's time-stamp handling cannot see a constant delay: VISO_DELAY_MS = age_ms + the link.

Runs inside Ubuntu (or on the Jetson) with ROS 2 loaded and pymavlink on the path:
  python3 vio_bridge.py [--mavlink tcp:127.0.0.1:5763] [--mount gazebo] [--imu-topic /vio/imu] [--log FILE]
"""
import argparse
import math
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu

R_ENU_NED = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])     # swaps north/east and flips up/down (its own inverse)
MOUNTS = {   # camera IMU axes in the body frame (columns: its x, y, z), and its position in the body frame (m)
    "gazebo": (np.array([[0.0, 0, 1], [0, -1, 0], [1, 0, 0]]), np.array([0.10, 0.0, 0.03])),
    "realsense-down": (np.array([[0.0, -1, 0], [1, 0, 0], [0, 0, 1]]), np.zeros(3)),
}
GPS_GOOD_S = 10                  # GPS good this long (and the switch-over script on GPS) before judging the VIO
GPS_GOOD_NO_SCRIPT_S = 30        # ... without word from the script (it switches back after VSW_GOOD_S = 10 s)
START_GRACE_S = 20               # a new VIO settles this long before it is judged
LOST_S = 3                       # no estimate for this long: OpenVINS has stopped
# SIGKILL: after SIGINT OpenVINS's ROS node crashes on the way out, and a crash dump each time (130-180 MB
# under WSL, /var/crash on the Jetson) fills the disk; it has nothing to save anyway
RESTART_CMD = ("pkill -KILL -f 'lib/ov_msckf/[r]un_subscribe_msckf'; for i in 1 2 3 4 5 6; do sleep 0.5; "
               "pgrep -f 'lib/ov_msckf/[r]un_subscribe_msckf' > /dev/null || exit 0; done")


def quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """Rotation matrix -> (w, x, y, z)."""
    tr = np.trace(R)
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        return 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0] * 4
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def euler_ned(R):
    """Roll, pitch, yaw (radians) of a body-to-NED rotation."""
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def turned(w, dt):
    """The rotation after turning at body rates w (rad/s) for dt seconds (Rodrigues)."""
    rate = np.linalg.norm(w)
    if rate * abs(dt) < 1e-9:
        return np.eye(3)
    k = w / rate
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    th = rate * dt
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Bridge(Node):
    def __init__(self, a):
        super().__init__("vio_bridge")
        self.a = a
        self.R_body_imu, self.lever = MOUNTS[a.mount]
        if a.lever:
            self.lever = np.array([float(v) for v in a.lever.split(",")])
        self.lock = threading.Lock()
        self.fc = dict(att_t=0.0, pos_t=0.0, fix=0, sats=0, armed=False, vsw_st=None, vsw_t=0.0)
        self.att_hist = deque(maxlen=200)   # flight controller attitude: (its time s, R body->NED, body rates, euler)
        self.pv_hist = deque(maxlen=200)    # ... position and velocity: (its time s, NED position, NED velocity)
        self.offsets = deque(maxlen=100)    # camera (ROS) time minus flight controller time, at each arrival
        self.imu_clock = None               # (latest camera IMU stamp s, wall time it arrived, its stamp message)
        self.truth = None
        self.init_sent_at = None
        self.vio_started = False
        self.vio_started_at = 0.0
        self.last_vio_at = 0.0
        self.runs = 0                       # VIO starts so far - also the reset counter ArduPilot gets
        self.restarting = False
        self.restarted_at = 0.0
        self.gps_good_since = None
        self.errs, self.bad_s, self.err = [], 0, 0.0
        self.last_update_at = 0.0        # when OpenVINS last updated from a picture (/ov_msckf/poseimu)
        self.pos_std = math.nan
        self.stalled = False
        self.stall_since = None          # when OpenVINS's last camera update before a stall came
        self.camera_at = 0.0             # when the camera's last picture came (its per-frame --camera-topic)
        self.recovery = False            # the next start may go without GPS, from ArduPilot's estimate
        self.last_sent = 0.0
        self.sent = 0
        self.warned = set()
        self.log = open(a.log, "w") if a.log else None
        if self.log:
            self.log.write("t,vio_n,vio_e,vio_d,vio_vn,vio_ve,vio_vd,fc_n,fc_e,fc_d,fc_vn,fc_ve,fc_vd,true_n,true_e,true_d,"
                           "gps_fix,run,age_ms,sent,pos_std\n")

        self.mav = mavutil.mavlink_connection(a.mavlink, source_system=1, source_component=197, autoreconnect=True)
        self.get_logger().info(f"waiting for the flight controller on {a.mavlink} ...")
        self.mav.wait_heartbeat()
        self.target = (self.mav.target_system, 1)
        # ArduPlane caps message rates by its main loop rate: 50 Hz requests are refused, 20 Hz is fine
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)):
            self.mav.mav.command_long_send(*self.target, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                           msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
        threading.Thread(target=self.mavlink_loop, daemon=True).start()

        self.init_pub = self.create_publisher(Odometry, "/ov_msckf/init_state", 10)
        self.fc_pub = self.create_publisher(Odometry, "/fc/odom", 50)
        self.create_subscription(Imu, a.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ov_msckf/odomimu", self.on_vio, 50)
        self.create_subscription(PoseWithCovarianceStamped, "/ov_msckf/poseimu", self.on_update, 10)
        self.create_subscription(CameraInfo, a.camera_topic, self.on_camera, qos_profile_sensor_data)
        self.create_subscription(TransformStamped, "/vio/truth", self.on_truth, 10)
        self.create_timer(0.1, self.check_start)
        self.create_timer(1.0, self.heartbeat)
        self.create_timer(1.0, self.check_fresh)
        self.create_timer(0.2, self.check_stall)
        self.get_logger().info(f"connected ({a.mount} mount, IMU {a.imu_topic}); starting the VIO once flying "
                               f"steadily above {a.init_alt:.0f} m with a good GPS")

    def tell(self, text):
        """To the log and to the pilot (the flight controller passes it on to the ground station)."""
        self.get_logger().info(text)
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text.encode()[:50])

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warning(text)

    # ---- flight controller side
    def mavlink_loop(self):
        while rclpy.ok():
            m = self.mav.recv_match(blocking=True, timeout=1)
            if m is None:
                continue
            t, now = m.get_type(), time.time()
            with self.lock:
                if t in ("ATTITUDE_QUATERNION", "LOCAL_POSITION_NED"):
                    fc_t = m.time_boot_ms / 1000
                    if self.att_hist and fc_t < self.att_hist[-1][0] - 1:       # it restarted
                        self.att_hist.clear()
                        self.pv_hist.clear()
                        self.offsets.clear()
                    if self.imu_clock:              # the camera clock now, from the last IMU sample
                        stamp, arrived, _ = self.imu_clock
                        self.offsets.append(stamp + (now - arrived) - fc_t)
                if t == "ATTITUDE_QUATERNION":
                    R = quat_to_R(m.q1, m.q2, m.q3, m.q4)
                    self.att_hist.append((fc_t, R, np.array([m.rollspeed, m.pitchspeed, m.yawspeed]), euler_ned(R)))
                    self.fc["att_t"] = now
                elif t == "LOCAL_POSITION_NED":
                    self.pv_hist.append((fc_t, np.array([m.x, m.y, m.z]), np.array([m.vx, m.vy, m.vz])))
                    self.fc["pos_t"] = now
                    if self.offsets and self.fc["fix"] >= 3 and self.fc["sats"] >= 6:
                        self.publish_fc(fc_t + float(np.median(self.offsets)), m)
                elif t == "GPS_RAW_INT":
                    self.fc["fix"], self.fc["sats"] = m.fix_type, m.satellites_visible
                elif t == "HEARTBEAT" and m.get_srcComponent() == 1:
                    self.fc["armed"] = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif t == "NAMED_VALUE_FLOAT" and m.name == "VSW_ST":        # switch-over script: 0 = on GPS
                    self.fc["vsw_st"], self.fc["vsw_t"] = int(m.value), now
            if t == "COMMAND_ACK" and m.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and m.result != 0:
                self.get_logger().warning(f"the flight controller refused a message rate request (result {m.result})")

    def publish_fc(self, ros_t, m):
        """The flight controller's GPS-aided position and velocity, east-north-up, at camera time ros_t."""
        msg = Odometry()
        msg.header.stamp.sec = int(ros_t)
        msg.header.stamp.nanosec = int((ros_t - int(ros_t)) * 1e9)
        msg.header.frame_id = "global"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = m.y, m.x, -m.z
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = m.vy, m.vx, -m.vz
        self.fc_pub.publish(msg)

    def heartbeat(self):
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

    def fc_at(self, ros_t):
        """The flight controller's attitude (body->NED), position and velocity at camera (ROS) time ros_t: from
        its samples around that moment, turned / moved on at their rates. None without fresh samples."""
        with self.lock:
            att, pv = self.att_hist, self.pv_hist
            if len(att) < 2 or len(pv) < 2 or not self.offsets:
                return None
            t = ros_t - float(np.median(self.offsets))      # on its clock
            if not att[0][0] <= t < att[-1][0] + 0.3 or not pv[0][0] <= t < pv[-1][0] + 0.3:
                return None
            a = next(h for h in reversed(att) if h[0] <= t)
            R = a[1] @ turned(a[2], t - a[0])
            k = next((i for i in range(len(pv) - 2, -1, -1) if pv[i][0] <= t), 0)
            f = (t - pv[k][0]) / max(pv[k + 1][0] - pv[k][0], 1e-3)
            return R, pv[k][1] + f * (pv[k + 1][1] - pv[k][1]), pv[k][2] + f * (pv[k + 1][2] - pv[k][2])

    def steady(self, secs):
        """Straight, or in a steady turn, for the last secs seconds: bank and pitch moderate and hardly
        changing, turning at under 20 deg/s (a loiter circle turns at about 12)."""
        with self.lock:
            if not self.att_hist or time.time() - self.fc["att_t"] > 0.3:
                return False
            last = self.att_hist[-1][0]
            hist = [(h[0], h[3]) for h in self.att_hist if last - h[0] <= secs]
        if len(hist) < secs * 10 or hist[-1][0] - hist[0][0] < secs * 0.8:
            return False
        roll, pitch = [e[0] for _, e in hist], [e[1] for _, e in hist]
        turn = abs(wrap(hist[-1][1][2] - hist[0][1][2])) / (hist[-1][0] - hist[0][0])
        return (max(map(abs, roll)) < math.radians(35) and max(map(abs, pitch)) < math.radians(20)
                and max(roll) - min(roll) < math.radians(4) and max(pitch) - min(pitch) < math.radians(4)
                and turn < math.radians(20))

    # ---- camera / simulator side
    def on_imu(self, msg):
        self.imu_clock = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, time.time(), msg.header.stamp)

    def on_update(self, msg):
        """OpenVINS has used a camera picture (it publishes poseimu only then)."""
        self.last_update_at = time.time()
        c = msg.pose.covariance
        self.pos_std = math.sqrt(max(c[0] + c[7], 0.0))

    def on_camera(self, msg):
        self.camera_at = time.time()

    def on_truth(self, msg):
        p = msg.transform.translation
        self.truth = R_ENU_NED @ np.array([p.x, p.y, p.z])

    # ---- 1. start OpenVINS from the flight controller's state
    def check_start(self):
        now = time.time()
        if self.vio_started or self.restarting or self.imu_clock is None:
            return
        with self.lock:
            fc = dict(self.fc)
        stamp_s, _, stamp = self.imu_clock
        state = self.fc_at(stamp_s)
        if state is None:
            if fc["armed"]:
                self.warn_once("stale", "no fresh attitude / position from the flight controller - cannot start")
            return
        R_att, pos, vel = state
        gps_ok = fc["fix"] >= 3 and fc["sats"] >= 6
        fresh_start = not gps_ok and self.recovery and self.runs >= 1 and self.a.recovery
        ready = fc["armed"] and (gps_ok or fresh_start) and -pos[2] > self.a.init_alt and self.steady(2)
        if not ready or (self.init_sent_at and now - self.init_sent_at < 5):
            return                                  # (waits for OpenVINS to answer before sending again)
        if self.init_pub.get_subscription_count() == 0:
            self.warn_once("listener", "OpenVINS is not listening for its start state (yet)")
            return
        R_enu_imu = R_ENU_NED @ R_att @ self.R_body_imu
        p_enu = R_ENU_NED @ (pos + R_att @ self.lever)
        v_enu = R_ENU_NED @ vel
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "global"
        msg.child_frame_id = "imu"
        w, x, y, z = R_to_quat(R_enu_imu)
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x = w, x
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z = y, z
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p_enu
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = v_enu
        self.init_pub.publish(msg)
        self.init_sent_at = now
        roll = math.degrees(euler_ned(R_att)[0])
        self.get_logger().info(f"start state sent to OpenVINS: {-pos[2]:.0f} m up, N {pos[0]:.0f} E {pos[1]:.0f}, "
                               f"{np.linalg.norm(vel[:2]):.1f} m/s, bank {roll:.0f} deg"
                               + (" - no GPS: from ArduPilot's estimate" if fresh_start else ""))

    # ---- 2. VIO -> flight controller
    def on_vio(self, msg):
        now = time.time()
        if self.restarting or now - self.restarted_at < 1.0:
            return                                  # the old OpenVINS, still finishing
        self.last_vio_at = now
        if not self.vio_started:
            self.vio_started, self.vio_started_at, self.bad_s, self.errs = True, now, 0, []
            self.last_update_at = max(self.last_update_at, now)    # not the old OpenVINS's last update
            self.runs += 1
            with self.lock:
                gps_ok = self.fc["fix"] >= 3 and self.fc["sats"] >= 6
            self.tell("VIO: started from the flight controller's state" if gps_ok
                      else "VIO: fresh start without GPS")
            self.recovery, self.stall_since = False, None
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last_sent < 1.0 / self.a.rate:
            return
        self.last_sent = t
        stale = self.a.stale_s > 0 and now - self.last_update_at > self.a.stale_s
        if stale != self.stalled:
            self.stalled = stale
            self.tell("VIO: no camera updates - held back" if stale else "VIO: camera updates again")
        o, p, v = msg.pose.pose.orientation, msg.pose.pose.position, msg.twist.twist.linear
        R_enu_imu = quat_to_R(o.w, o.x, o.y, o.z)
        R_ned_body = R_ENU_NED @ R_enu_imu @ self.R_body_imu.T
        pos = R_ENU_NED @ np.array([p.x, p.y, p.z]) - R_ned_body @ self.lever
        vel = R_ENU_NED @ (R_enu_imu @ np.array([v.x, v.y, v.z]))      # OpenVINS gives it in its IMU frame
        roll, pitch, yaw = euler_ned(R_ned_body)
        usec, reset = int(t * 1e6), self.runs % 256
        if not self.stalled:
            self.mav.mav.vision_position_estimate_send(usec, *pos, roll, pitch, yaw, reset_counter=reset)
            self.mav.mav.vision_speed_estimate_send(usec, *vel, reset_counter=reset)
        self.sent += 1
        stamp_s, arrived, _ = self.imu_clock or (t, now, None)
        age_ms = (stamp_s + (now - arrived) - t) * 1000          # camera clock now minus the picture's time
        state = self.fc_at(t)                       # the flight controller at the moment of the picture
        if state is not None and not self.stalled:
            self.errs.append(float(np.linalg.norm(vel[:2] - state[2][:2])))
        if self.log:
            fcp, fcv = (state[1], state[2]) if state is not None else ([math.nan] * 3, [math.nan] * 3)
            tr = self.truth if self.truth is not None else [math.nan] * 3
            self.log.write(",".join(f"{x:.3f}" for x in [t, *pos, *vel, *fcp, *fcv, *tr])
                           + f",{self.fc['fix']},{self.runs},{age_ms:.0f},{0 if self.stalled else 1},{self.pos_std:.2f}\n")
            if self.sent % 100 == 0:
                self.log.flush()

    # ---- 3. keep the VIO fresh while the plane flies on GPS
    def check_fresh(self):
        now = time.time()
        with self.lock:
            fc = dict(self.fc)
        errs, self.errs = self.errs, []
        gps_good = fc["fix"] >= 3 and fc["sats"] >= 6
        self.gps_good_since = (self.gps_good_since or now) if gps_good else None
        if not self.vio_started or self.restarting:
            return
        if now - self.last_vio_at > LOST_S:
            self.recovery = not gps_good and self.a.recovery
            self.tell("VIO: stopped - fresh start without GPS" if self.recovery
                      else "VIO: stopped - starts again with GPS")
            self.vio_started, self.init_sent_at = False, None
            return
        script_says = fc["vsw_st"] if now - fc["vsw_t"] < 3 else None
        good_s = now - self.gps_good_since if self.gps_good_since else 0
        on_gps = good_s >= GPS_GOOD_S and script_says == 0 or script_says is None and good_s >= GPS_GOOD_NO_SCRIPT_S
        if not on_gps or now - self.vio_started_at < START_GRACE_S:
            self.bad_s = 0
            return                                  # the plane may be flying on the VIO, or it is new
        steady = self.steady(2)
        if errs and steady:                         # (while bank or pitch change, the two can disagree briefly)
            self.err = float(np.median(errs))
            self.bad_s = self.bad_s + 1 if self.err > self.a.refresh_err else 0
        age_min = (now - self.vio_started_at) / 60
        if self.bad_s >= self.a.refresh_s:
            self.restart(f"VIO: restart, speed {self.err:.1f} m/s off GPS")
        elif 0 < self.a.max_age_min < age_min and steady:
            self.restart(f"VIO: restart, {age_min:.0f} min old")

    def check_stall(self):
        """A long camera stall: once pictures flow again (or OpenVINS carries on by itself), start it afresh."""
        now = time.time()
        if not self.vio_started or self.restarting or self.a.stale_s <= 0:
            return
        stale = now - self.last_update_at > self.a.stale_s
        if stale and self.stall_since is None:
            self.stall_since = self.last_update_at
        long_stall = self.stall_since is not None and now - self.stall_since >= self.a.stall_restart_s
        frames = now - self.camera_at < 0.5
        if long_stall and (frames or not stale):
            with self.lock:
                gps_ok = self.fc["fix"] >= 3 and self.fc["sats"] >= 6
            self.recovery = not gps_ok and self.a.recovery
            self.restart(f"VIO: {now - self.stall_since:.0f} s without camera updates - restart")
            self.stall_since = None
        elif not stale:
            self.stall_since = None
        if self.vio_started and now - self.vio_started_at > 5 and self.camera_at == 0.0:
            self.warn_once("camera", f"no pictures seen on {self.a.camera_topic} - camera stalls are only noticed "
                                     "when OpenVINS carries on by itself")

    def restart(self, text):
        self.tell(text)
        self.vio_started, self.init_sent_at, self.bad_s, self.restarting = False, None, 0, True
        threading.Thread(target=self.stop_openvins, daemon=True).start()

    def stop_openvins(self):
        subprocess.run(self.a.restart_cmd, shell=True)       # returns once the old OpenVINS is gone
        self.restarted_at, self.restarting = time.time(), False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink", default="tcp:127.0.0.1:5763", help="flight controller connection")
    ap.add_argument("--mount", default="gazebo", choices=sorted(MOUNTS), help="how the camera sits on the plane")
    ap.add_argument("--lever", help="FORWARD,RIGHT,DOWN: camera IMU position from the flight controller (m)")
    ap.add_argument("--imu-topic", default="/vio/imu", help="the camera IMU's ROS topic (D455: /d455/imu)")
    ap.add_argument("--init-alt", type=float, default=40.0, help="start the VIO above this height (m)")
    ap.add_argument("--rate", type=float, default=20.0, help="VIO messages per second to the flight controller")
    ap.add_argument("--refresh-err", type=float, default=0.7, help="restart when the VIO speed is this far off (m/s)")
    ap.add_argument("--refresh-s", type=float, default=5.0, help="... for this many seconds of steady flight")
    ap.add_argument("--max-age-min", type=float, default=5.0, help="restart when older than this (min; 0 = never)")
    ap.add_argument("--restart-cmd", default=RESTART_CMD, help="stops OpenVINS; its launcher starts it again")
    ap.add_argument("--stale-s", type=float, default=1.0,
                    help="hold the VIO back after this long without a camera update (s; 0 = never)")
    ap.add_argument("--stall-restart-s", type=float, default=3.0,
                    help="after a stall this long, restart OpenVINS once pictures flow again (s)")
    ap.add_argument("--camera-topic", default="/vio/camera_info",
                    help="the camera's per-frame CameraInfo (D455: /d455/color/camera_info)")
    ap.add_argument("--no-recovery", dest="recovery", action="store_false",
                    help="never start the VIO without GPS (after a stall or crash during a GPS outage)")
    ap.add_argument("--log", help="CSV log of everything sent")
    a = ap.parse_args()
    rclpy.init()
    node = Bridge(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.log:
            node.log.close()


if __name__ == "__main__":
    main()
