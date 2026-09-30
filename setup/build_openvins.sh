#!/bin/bash
# Builds OpenVINS (ROS 2 Humble) in ~/ws_ov inside Ubuntu 22.04. No sudo needed.
# Builds one package at a time with 2 compile jobs, to stay inside WSL's 6 GB memory limit.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/build_openvins.sh
set -e
source /opt/ros/humble/setup.bash

mkdir -p ~/ws_ov/src
cd ~/ws_ov/src
if [ ! -d open_vins ]; then
    git clone https://github.com/rpng/open_vins.git
fi
cd open_vins
echo "OpenVINS version: $(git log -1 --format='%h %cd %s' --date=short)"

cd ~/ws_ov
export MAKEFLAGS="-j2"
colcon build --executor sequential --event-handlers console_cohesion+ \
    --packages-select ov_core ov_init ov_msckf ov_eval \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

grep -q "ws_ov/install/setup.bash" ~/.bashrc || echo "source ~/ws_ov/install/setup.bash" >> ~/.bashrc
echo
echo "Done: OpenVINS is built in ~/ws_ov."
