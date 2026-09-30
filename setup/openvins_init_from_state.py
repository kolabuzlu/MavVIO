#!/usr/bin/env python3
"""Adds "start from a given state" to OpenVINS's ROS 2 node (~/ws_ov), then rebuild ov_msckf.

OpenVINS normally starts itself: standing still then a jolt (static), or from enough motion (dynamic).
With this change run_subscribe_msckf can instead start from a known state of its IMU - position,
orientation, velocity in its world frame (z up) - so the VIO works in the flight controller's frame
from the first moment:
  * -p init_from_topic:=true  waits for that state on /ov_msckf/init_state (nav_msgs/Odometry: pose of
    the IMU, twist.linear = its velocity in the world frame, header stamp = IMU time); tools/vio_bridge.py
    sends it from the flight controller's own state while GPS is still good - as on the real plane
  * -p path_gt:=FILE -p init_from_gt:=true  takes it from a EuRoC ground-truth file at the first IMU
    message (tools/bag_gt_init.py writes one from a simulator recording)
Until then camera pictures are ignored, so the built-in initializers cannot start on their own.
Applied to a clean checkout each time (git checkout of the two files first); git diff in
~/ws_ov/src/open_vins shows the change, setup/openvins_init_from_state.patch keeps a copy.

Run inside Ubuntu:  python3 /mnt/c/Users/funfo/vio/setup/openvins_init_from_state.py
Then rebuild:       cd ~/ws_ov && source /opt/ros/humble/setup.bash && \\
                    colcon build --executor sequential --packages-select ov_msckf --cmake-args -DCMAKE_BUILD_TYPE=Release
"""
import subprocess
from pathlib import Path

REPO = Path.home() / "ws_ov/src/open_vins"
SRC = REPO / "ov_msckf/src/ros"
MARK = "vio project"


def patch(path, old, new):
    text = path.read_text()
    assert text.count(old) == 1, f"{path.name}: expected text not found once: {old[:60]!r}"
    path.write_text(text.replace(old, new))


subprocess.run(["git", "-C", str(REPO), "checkout", "--", "ov_msckf/src/ros/ROS2Visualizer.cpp",
                "ov_msckf/src/ros/ROS2Visualizer.h"], check=True)

h = SRC / "ROS2Visualizer.h"
patch(h, "  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;\n",
      "  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;\n\n"
      f"  // {MARK}: start from a given state (path_gt file or init_state topic) instead of the built-in initializers\n"
      "  bool init_from_gt = false;\n"
      "  bool init_from_topic = false;\n"
      "  bool init_from_gt_done = false;\n"
      "  std::mutex init_state_mtx;\n"
      "  bool init_state_pending = false;\n"
      "  Eigen::Matrix<double, 17, 1> init_state_value = Eigen::Matrix<double, 17, 1>::Zero();\n"
      "  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_init_state;\n")

cpp = SRC / "ROS2Visualizer.cpp"
patch(cpp, """    DatasetReader::load_gt_file(path_to_gt, gt_states);
    PRINT_DEBUG("gt file path is: %s\\n", path_to_gt.c_str());
  }
""", """    DatasetReader::load_gt_file(path_to_gt, gt_states);
    PRINT_DEBUG("gt file path is: %s\\n", path_to_gt.c_str());
  }
  // """ + MARK + """: optionally start from a given state (see callback_inertial)
  if (node->has_parameter("init_from_gt")) {
    node->get_parameter<bool>("init_from_gt", init_from_gt);
  }
  if (node->has_parameter("init_from_topic")) {
    node->get_parameter<bool>("init_from_topic", init_from_topic);
  }
  if (init_from_topic) {
    // pose of the IMU in the world frame, twist.linear = its velocity in the world frame
    sub_init_state = node->create_subscription<nav_msgs::msg::Odometry>(
        "init_state", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          Eigen::Matrix<double, 17, 1> s = Eigen::Matrix<double, 17, 1>::Zero();
          s(0) = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
          // a Hamilton IMU-to-world quaternion has the same x, y, z, w as OpenVINS's JPL world-to-IMU one
          s(1) = msg->pose.pose.orientation.x;
          s(2) = msg->pose.pose.orientation.y;
          s(3) = msg->pose.pose.orientation.z;
          s(4) = msg->pose.pose.orientation.w;
          s(5) = msg->pose.pose.position.x;
          s(6) = msg->pose.pose.position.y;
          s(7) = msg->pose.pose.position.z;
          s(8) = msg->twist.twist.linear.x;
          s(9) = msg->twist.twist.linear.y;
          s(10) = msg->twist.twist.linear.z;
          std::lock_guard<std::mutex> lck(init_state_mtx);
          init_state_value = s;
          init_state_pending = true;
        });
    PRINT_INFO(GREEN "[INIT]: waiting for the start state on init_state\\n" RESET);
  }
""")
patch(cpp, """  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // send it to our VIO system
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp);
""", """  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // """ + MARK + """: start from the given state at the first IMU message that has it (as a flight
  // controller would give it while GPS is still good) instead of OpenVINS's own initializers
  if ((init_from_gt || init_from_topic) && !init_from_gt_done) {
    Eigen::Matrix<double, 17, 1> imustate = Eigen::Matrix<double, 17, 1>::Zero();
    bool have_state = false;
    if (init_from_gt && !gt_states.empty()) {
      have_state = DatasetReader::get_gt_state(message.timestamp, imustate, gt_states);
    } else if (init_from_topic) {
      std::lock_guard<std::mutex> lck(init_state_mtx);
      if (init_state_pending && message.timestamp >= init_state_value(0)) {
        imustate = init_state_value;
        have_state = true;
      }
    }
    if (have_state) {
      imustate(0, 0) = message.timestamp;
      _app->initialize_with_gt(imustate);
      init_from_gt_done = true;
      PRINT_INFO(GREEN "[INIT]: started from the given state at %.3f s\\n" RESET, message.timestamp);
    }
  }

  // send it to our VIO system
  _app->feed_measurement_imu(message);
  visualize_odometry(message.timestamp);
""")
patch(cpp, """void ROS2Visualizer::callback_monocular(const sensor_msgs::msg::Image::SharedPtr msg0, int cam_id0) {
""", """void ROS2Visualizer::callback_monocular(const sensor_msgs::msg::Image::SharedPtr msg0, int cam_id0) {

  // """ + MARK + """: no pictures before the start from the given state
  if ((init_from_gt || init_from_topic) && !init_from_gt_done)
    return;
""")
subprocess.run(f"git -C {REPO} diff > /mnt/c/Users/funfo/vio/setup/openvins_init_from_state.patch", shell=True, check=True)
print("patched - now rebuild ov_msckf")
