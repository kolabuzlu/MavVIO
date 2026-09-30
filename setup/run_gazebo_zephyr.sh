#!/bin/bash
# Gazebo step 1: ArduPilot's plane simulator (ArduPlane SITL, built by build_ardupilot_gazebo.sh)
# flying the Zephyr flying wing in Gazebo Harmonic. Opens the Gazebo window (through WSLg) and starts
# SITL connected to it. MAVLink on TCP 5760 (our tools) and 5762 (Mission Planner) - from Windows
# both are reachable as 127.0.0.1. Stop with Ctrl+C.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/run_gazebo_zephyr.sh
set -e
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # render on the RTX GPU rather than the built-in Intel one

WORK=$HOME/sitl_gazebo
mkdir -p "$WORK"
cd "$WORK"
AP=$HOME/ardupilot

echo "Starting Gazebo (log: $WORK/gazebo.log) ..."
gz sim -v3 -r zephyr_runway.sdf > gazebo.log 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null' EXIT
sleep 8

echo "Starting ArduPlane SITL (log: $WORK/sitl.log) - MAVLink on TCP 5760 and 5762"
# home = the world's origin (zephyr_runway.sdf <spherical_coordinates>), so SITL and Gazebo agree
"$AP/build/sitl/bin/arduplane" --model JSON --speedup 1 --slave 0 -I0 -w \
    --defaults "$AP/Tools/autotest/default_params/gazebo-zephyr.parm" \
    --sim-address=127.0.0.1 --home -35.363262,149.165237,584,0 > sitl.log 2>&1
