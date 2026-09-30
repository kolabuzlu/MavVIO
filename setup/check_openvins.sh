#!/bin/bash
# Read-only check that the OpenVINS build is usable from ROS 2.
source /opt/ros/humble/setup.bash
source ~/ws_ov/install/setup.bash
echo "OpenVINS packages known to ROS 2: $(ros2 pkg list | grep -E '^ov_' | tr '\n' ' ')"
echo "launch file: $(ls ~/ws_ov/install/ov_msckf/share/ov_msckf/launch/ 2>/dev/null | tr '\n' ' ')"
echo "EuRoC config: $(ls ~/ws_ov/src/open_vins/config/euroc_mav/ | tr '\n' ' ')"
echo "run_subscribe_msckf program: $(ls ~/ws_ov/install/ov_msckf/lib/ov_msckf/ 2>/dev/null | tr '\n' ' ')"
