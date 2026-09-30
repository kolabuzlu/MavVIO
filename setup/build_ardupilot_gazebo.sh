#!/bin/bash
# Builds ArduPilot's plane simulator (SITL) and ArduPilot's Gazebo plugin in your Ubuntu home folder.
# Needs install_gazebo_harmonic.sh and get_ardupilot_sources.sh first. No password needed.
# ArduPilot's Python build tools go into their own environment (~/venv-ardupilot), so the system
# Python that ROS 2 uses is not touched. Few compile jobs at a time, to stay inside the 6 GB WSL limit.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/build_ardupilot_gazebo.sh
set -e

echo "=== 1/3 Python environment for the ArduPilot build (~/venv-ardupilot)"
[ -d ~/venv-ardupilot ] || python3 -m venv ~/venv-ardupilot
. ~/venv-ardupilot/bin/activate
pip install --quiet --upgrade pip
pip install --quiet empy==3.3.4 future lxml pymavlink pexpect dronecan setuptools

echo "=== 2/3 ArduPlane simulator (SITL)"
cd ~/ardupilot
# the parts of ArduPilot the simulator needs (not ChibiOS etc., which only real flight controllers use);
# without this the first ./waf call only fetches waf itself and stops
git submodule update --init --recursive --depth 1 modules/waf modules/mavlink modules/DroneCAN \
    modules/littlefs modules/lwip modules/gtest
./waf configure --board sitl
./waf plane -j4

echo "=== 3/3 ArduPilot's Gazebo plugin"
export GZ_VERSION=harmonic
mkdir -p ~/ardupilot_gazebo/build
cd ~/ardupilot_gazebo/build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j2

echo
ls -la ~/ardupilot/build/sitl/bin/arduplane ~/ardupilot_gazebo/build/*.so
df -h / | tail -1
echo "Done: simulator and Gazebo plugin are built."
