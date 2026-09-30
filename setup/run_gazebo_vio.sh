#!/bin/bash
# Gazebo step 2: the VIO test world (made by tools/make_scenery.py into ~/vio_gazebo) with the Zephyr
# carrying a downward camera, and ArduPilot's plane simulator (ArduPlane SITL) connected to it.
# Two ways to watch: in the Windows browser, http://localhost:8080 (a chase camera behind the plane and
# the VIO camera, rendered inside the simulation - the smoothest view), and the Gazebo window (on the
# Intel graphics, ~20 pictures/s, following the plane rigidly; a watchdog reopens it every ~15-20 min
# against WSL's graphics memory leak - see gazebo_window.sh). Without the window: GZ_WINDOW=0.
# Other cameras in the browser (world made with make_scenery.py --tilts 30,45): GZ_VIEW=vio,t45,t30
# The simulation server itself also leaks graphics memory under WSL (~500 MB/min with three or four
# cameras): keep sessions under ~10 minutes, or the Linux VM runs out and stops.
# Reopen a closed window with:  bash /mnt/c/Users/funfo/vio/setup/gazebo_window.sh
# MAVLink on TCP 5760 (our tools) and 5762 (Mission Planner) - from Windows both are 127.0.0.1.
# Stop everything with Ctrl+C, or:  bash /mnt/c/Users/funfo/vio/setup/stop_gazebo_vio.sh
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/run_gazebo_vio.sh
set -e
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/vio_gazebo/models:$HOME/vio_gazebo/worlds:$HOME/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH
# Graphics for the simulation's cameras: GZ_GPU=NVIDIA (default, fast) or Intel (the built-in one). Under WSL
# the server leaks memory for every picture drawn: ~530 MB/min on the RTX with four cameras (step 2-3) -
# too much for long recordings in 6 GB; the Intel graphics leaked much less with the window (step 2-1).
export MESA_D3D12_DEFAULT_ADAPTER_NAME=${GZ_GPU:-NVIDIA}

WORK=$HOME/sitl_gazebo
mkdir -p "$WORK"
cd "$WORK"
AP=$HOME/ardupilot
VIO=/mnt/c/Users/funfo/vio

echo "Starting the Gazebo simulation server with the VIO world (log: $WORK/gazebo_server.log) ..."
gz sim -s -r -v3 --headless-rendering vio_world.sdf > gazebo_server.log 2>&1 &
SERVER=$!
PIDS=$SERVER
trap 'kill $PIDS 2>/dev/null' EXIT
sleep 10
trap 'kill $PIDS 2>/dev/null; pkill -f "gazebo_window.sh watchdog"; pkill -f "^gz sim -g"' EXIT
echo "Live view for the Windows browser on http://localhost:8080 (log: $WORK/web_view.log)"
python3 "$VIO/tools/gz_web_view.py" --streams "${GZ_VIEW:-chase,vio}" > web_view.log 2>&1 &
PIDS="$PIDS $!"
if [ "${GZ_WINDOW:-1}" = "1" ]; then
    # the window, with a watchdog against WSL's graphics memory leak (see gazebo_window.sh)
    echo "Opening the Gazebo window (log: $WORK/gazebo_window.log) ..."
    bash "$VIO/setup/gazebo_window.sh"
fi

echo "Starting ArduPlane SITL (log: $WORK/sitl.log) - MAVLink on TCP 5760, 5762 and 5763 (VIO companion)"
# the real plane's VIO setup (vio_plane.parm + the switch-over script), then the simulator's own settings
mkdir -p scripts
cp "$VIO/scripts/vio_gps_switch.lua" scripts/
# home = the world's origin (<spherical_coordinates>), so SITL and Gazebo agree
"$AP/build/sitl/bin/arduplane" --model JSON --speedup 1 --slave 0 -I0 -w \
    --defaults "$AP/Tools/autotest/default_params/gazebo-zephyr.parm,$VIO/params/vio_plane.parm,$VIO/params/gazebo_vio.parm" \
    --sim-address=127.0.0.1 --home -35.363262,149.165237,584,0 > sitl.log 2>&1
