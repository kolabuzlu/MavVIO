#!/bin/bash
# Quick health check of the Ubuntu 22.04 / ROS 2 setup (read-only).
. /etc/os-release; echo "system: $PRETTY_NAME, user $(whoami), $(nproc) CPU threads"
echo "memory available to Linux: $(free -h | awk '/^Mem:/{print $2}') (swap $(free -h | awk '/^Swap:/{print $2}'))"
echo "disk: $(df -h / | awk 'NR==2{print $4}') free inside Ubuntu"
source /opt/ros/humble/setup.bash 2>/dev/null && echo "ROS 2: $ROS_DISTRO, $(ros2 pkg list 2>/dev/null | wc -l) packages" || echo "ROS 2: not found"
for p in colcon git cmake; do printf "%-6s %s\n" "$p:" "$(command -v $p >/dev/null && echo ok || echo MISSING)"; done
for h in /usr/include/eigen3/Eigen/Core /usr/include/ceres/ceres.h /usr/include/boost/version.hpp; do printf "%-38s %s\n" "$h" "$([ -e $h ] && echo ok || echo MISSING)"; done
