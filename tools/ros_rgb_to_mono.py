#!/usr/bin/env python3
"""Turns the simulator's colour camera pictures into grey ones, as a real mono camera gives them.

Gazebo's own grey format (L8) comes out several times too dark (it skips the gamma step a real camera
applies), so the VIO cameras draw in colour and this node converts them: it subscribes to /vio/CAM_rgb
(sensor_msgs/Image rgb8, bridged by setup/record_gazebo_flight.sh) and publishes /vio/CAM as mono8 with
the same time stamps, using luminance = 0.299 R + 0.587 G + 0.114 B - with OpenCV itself (cv2.cvtColor, as
cv_bridge does; 0.7 ms for a 1280 x 800 picture, 15x faster than the numpy way, which stays as a fallback).

With --sd WxH it also publishes /vio/CAM_sd: the picture cut to that shape around the centre and shrunk to
W x H (area averaging) - what the D455 gives in its smaller modes: its 848 x 480 is the middle 1280 x 720 of
the 1280 x 800 sensor, scaled down. So one 1280 x 800 camera can feed OpenVINS at both resolutions at once.

For testing how much a real camera's pictures may be spoiled (the simulator's are perfect), extra copies:
  --blur-deg D   /vio/CAM_blur: motion blur D degrees long - what the exposure time times the turn rate
                 smears; the direction follows the camera IMU's gyro (/vio/imu), as a real smear would
  --noise S      /vio/CAM_noise: sensor noise, standard deviation S grey levels (dim light, high gain)
With --sd each also comes as ..._sd, shrunk from the spoiled full picture (so the shrinking averages the
noise away as the camera's own scaler does).

Each picture also produces a tiny /vio/CAM_info (sensor_msgs/CameraInfo, its time stamp and size) - as the
RealSense driver's camera_info - so tools/vio_bridge.py can tell that pictures flow without receiving them.

Runs inside Ubuntu with ROS 2 loaded:
  python3 ros_rgb_to_mono.py [CAM ...] [--sd 848x480] [--blur-deg 0.6] [--noise 8]   (default: 3 VIO cameras)
"""
import argparse
import array
import math

import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

WEIGHTS = np.array([0.299, 0.587, 0.114], dtype=np.float32)


def grey_msg(header, img):
    out = Image()
    out.header = header
    out.height, out.width = img.shape
    out.encoding, out.is_bigendian, out.step = "mono8", 0, img.shape[1]
    out.data = array.array("B", img.tobytes())      # not bytes: those get checked byte by byte in Python (slow)
    return out


class RgbToMono(Node):
    def __init__(self, a):
        super().__init__("rgb_to_mono")
        self.a = a
        self.sd = tuple(int(v) for v in a.sd.split("x")) if a.sd else None
        self.gyro = np.zeros(3)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        extras = (["blur"] if a.blur_deg else []) + (["noise"] if a.noise else [])
        for cam in a.cams:
            pubs = {"info": self.create_publisher(CameraInfo, f"/vio/{cam}_info", 10)}
            for kind in [""] + extras:
                name = f"_{kind}" if kind else ""
                pubs[kind] = self.create_publisher(Image, f"/vio/{cam}{name}", qos)
                if self.sd:
                    pubs[kind + "_sd"] = self.create_publisher(Image, f"/vio/{cam}{name}_sd", qos)
            self.create_subscription(Image, f"/vio/{cam}_rgb", lambda msg, pubs=pubs: self.convert(msg, pubs), qos)
        if a.blur_deg:
            self.create_subscription(Imu, "/vio/imu", self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(f"converting {', '.join(a.cams)} to grey" + (f", also as {a.sd}" if a.sd else "")
                               + (f", blurred {a.blur_deg} deg" if a.blur_deg else "")
                               + (f", with noise {a.noise}" if a.noise else ""))

    def on_imu(self, msg):
        w = msg.angular_velocity
        self.gyro = np.array([w.x, w.y, w.z])

    def shrink(self, img):
        w, h = self.sd
        rows = min(img.shape[0], round(img.shape[1] * h / w))            # same shape, cut around the centre
        top = (img.shape[0] - rows) // 2
        return cv2.resize(img[top:top + rows], (w, h), interpolation=cv2.INTER_AREA)

    def blurred(self, img):
        """A straight smear blur_deg long. The Gazebo camera module's IMU axes are x down (the lens), y left,
        z forward, and the picture's top is the nose: turning about y smears up/down, about z sideways."""
        length = self.a.blur_deg * img.shape[1] / self.a.hfov_deg          # in pixels
        wx, wy, wz = self.gyro
        du, dv = -wz, -wy
        if math.hypot(du, dv) < 0.05:
            du, dv = 0.0, 1.0                                              # nearly still: along the flight
        ang = math.atan2(dv, du)
        k = int(math.ceil(length)) | 1
        kernel = np.zeros((k, k), np.float32)
        c, r = k // 2, length / 2
        p0 = (int(round(c - r * math.cos(ang))), int(round(c - r * math.sin(ang))))
        p1 = (int(round(c + r * math.cos(ang))), int(round(c + r * math.sin(ang))))
        cv2.line(kernel, p0, p1, 1.0, 1)
        kernel /= max(kernel.sum(), 1e-6)
        return cv2.filter2D(img, -1, kernel, borderType=cv2.BORDER_REFLECT)

    def noisy(self, img):
        n = np.empty(img.shape, np.int16)
        cv2.randn(n, 0, self.a.noise)
        return np.clip(img.astype(np.int16) + n, 0, 255).astype(np.uint8)

    def convert(self, msg, pubs):
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        rgb = np.ascontiguousarray(rows).reshape(msg.height, msg.width, 3)
        if cv2 is not None:
            grey = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY if msg.encoding == "bgr8" else cv2.COLOR_RGB2GRAY)
        else:
            rgb = rgb.astype(np.float32)[..., ::-1] if msg.encoding == "bgr8" else rgb.astype(np.float32)
            grey = np.clip(rgb @ WEIGHTS + 0.5, 0, 255).astype(np.uint8)
        versions = {"": grey}
        if cv2 is not None:
            if "blur" in pubs:
                versions["blur"] = self.blurred(grey)
            if "noise" in pubs:
                versions["noise"] = self.noisy(grey)
        info = CameraInfo()
        info.header, info.height, info.width = msg.header, msg.height, msg.width
        pubs["info"].publish(info)
        for kind, img in versions.items():
            pubs[kind].publish(grey_msg(msg.header, img))
            if kind + "_sd" in pubs and cv2 is not None:
                pubs[kind + "_sd"].publish(grey_msg(msg.header, self.shrink(img)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cams", nargs="*", default=["camera", "camera_t30", "camera_t45"])
    ap.add_argument("--sd", help="WxH: also publish /vio/CAM_sd at this smaller size, e.g. 848x480")
    ap.add_argument("--blur-deg", type=float, default=0.0, help="also publish /vio/CAM_blur, smeared this many degrees")
    ap.add_argument("--noise", type=float, default=0.0, help="also publish /vio/CAM_noise with this much noise")
    ap.add_argument("--hfov-deg", type=float, default=91.0, help="the camera's width in degrees (for --blur-deg)")
    a, _ = ap.parse_known_args()
    rclpy.init()
    node = RgbToMono(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
