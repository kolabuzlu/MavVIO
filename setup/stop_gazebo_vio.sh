#!/bin/bash
# Stops the Gazebo VIO simulation started by run_gazebo_vio.sh (and run_vio_live.sh / sim_watch.py if they
# run): simulator, Gazebo server and window, the window watchdog, the browser live view and the VIO chain.
# The Gazebo server is killed at once (SIGKILL): asked nicely it crashes on its way out, and for every such
# crash WSL writes a ~230 MB crash dump into Windows' Temp folder (up to 10 of them - that filled the C:
# drive once, 2026-09-29). The rest is asked nicely first; in lock-step the simulator and the Gazebo server
# can wait for each other forever, so whatever is left after 5 s is killed too.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/stop_gazebo_vio.sh
# (the [x] in the patterns keeps pkill from matching its own command line)
PATTERN="[g]z sim|[a]rduplane --model JSON|[g]z_web_view.py|[g]azebo_window.sh watchdog"
pkill -f "[s]im_watch.py"
pkill -f "[r]un_vio_live.sh"
pkill -f "[r]un_vio_variants.sh"
pkill -f "[r]os_truth_log.py"
pkill -f "[v]io_bridge.py"
# OpenVINS: straight SIGKILL - after SIGINT its ROS node crashes on the way out, and WSL keeps a
# 130-180 MB dump of every crash (%LOCALAPPDATA%\Temp\wsl-crashes)
pkill -9 -f "lib/ov_msckf/[r]un_subscribe_msckf"
pkill -f "[r]os_rgb_to_mono"
pkill -f "[p]arameter_bridge"
pkill -f "[g]azebo_window.sh watchdog"
pkill -9 -f "[g]z sim"
pkill -f "[a]rduplane --model JSON"
pkill -f "[g]z_web_view.py"
for i in $(seq 1 5); do
    pgrep -f "$PATTERN" > /dev/null || break
    sleep 1
done
pkill -9 -f "$PATTERN" && sleep 1
pkill -9 -f "lib/ov_msckf/[r]un_subscribe_msckf"
pgrep -af "$PATTERN" | cut -c1-80 || echo "all stopped"
