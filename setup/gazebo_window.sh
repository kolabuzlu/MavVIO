#!/bin/bash
# (Re)opens the Gazebo window on a simulation that is already running (started by run_gazebo_vio.sh),
# and makes the view follow the plane. Closing the window does not stop the simulation.
# The window draws on the Intel graphics. Through WSL's GPU layer (d3d12) a Gazebo window leaks memory -
# about 120 MB a minute on the Intel graphics, 600 on the RTX - until Ubuntu runs out and everything
# crashes, so a watchdog here reopens the window when it passes WINDOW_MB (default 2500). Software
# rendering does not leak but needs 2-8 CPU cores and draws too slowly. The camera itself renders
# off-screen on the RTX, without a leak. If you close the window yourself, it stays closed.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/gazebo_window.sh
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH=$HOME/vio_gazebo/models:$HOME/vio_gazebo/worlds:$HOME/ardupilot_gazebo/models:$GZ_SIM_RESOURCE_PATH
export MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel
export QT_QPA_PLATFORM=xcb      # an ordinary X11 window, which Windows can move and maximize
LIMIT_MB=${WINDOW_MB:-2500}
CONFIG=/mnt/c/Users/funfo/vio/setup/gazebo_window.config
LOG=$HOME/sitl_gazebo/gazebo_window.log

pkill -f "gazebo_window.sh watchdog" 2>/dev/null   # only one window (and one watchdog) at a time
pkill -f "^gz sim -g" 2>/dev/null
sleep 1

follow() {
    # follow the plane rigidly (pgain 1): the default lazy follow makes the plane rock back and forth
    sleep 10
    gz topic -t /gui/track -m gz.msgs.CameraTrack \
        -p 'track_mode: FOLLOW, follow_target: {name: "zephyr_vio"}, follow_offset: {x: -8, y: 0, z: 4}, follow_pgain: 1.0'
}

watchdog() {
    while true; do
        gz sim -g -v3 --gui-config "$CONFIG" >> "$LOG" 2>&1 &
        PID=$!
        follow &
        reason=closed
        while kill -0 $PID 2>/dev/null; do
            rss=$(ps -o rss= -p $PID 2>/dev/null || echo 0)
            if [ $((rss / 1024)) -gt $LIMIT_MB ]; then
                reason=memory
                kill $PID
                break
            fi
            sleep 5
        done
        wait $PID 2>/dev/null
        [ "$reason" = closed ] && break
        echo "$(date +%T) window reopened (it had reached ${LIMIT_MB} MB)" >> "$LOG"
    done
}

: > "$LOG"
# run the watchdog in the background under a recognisable name, so this script returns at once
nohup bash -c "$(declare -f follow watchdog); CONFIG='$CONFIG' LOG='$LOG' LIMIT_MB=$LIMIT_MB; watchdog" \
    gazebo_window.sh watchdog > /dev/null 2>&1 &
sleep 12
echo "Gazebo window open (Intel graphics), following the plane"
