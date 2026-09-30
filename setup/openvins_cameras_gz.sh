#!/bin/bash
# Step 2-3 - which way should the VIO camera look? Runs OpenVINS on each camera of one Gazebo recording
# (setup/record_gazebo_flight.sh: /vio/camera straight down, /vio/camera_t45 and /vio/camera_t30 tilted
# towards the nose) with the same settings (openvins_config/gz_down, gz_t45, gz_t30), all started from
# the same true state in level flight (tools/bag_gt_init.py - on the real plane: the flight controller's
# state while GPS is still good), side by side. Compare afterwards with tools/compare_cameras.py.
# Results: results/BAG_NAME_down, _t45, _t30 and the start state results/BAG_NAME_state.csv
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/openvins_cameras_gz.sh [BAG_NAME] ["down t45 t30"]
BAG_NAME=${1:-gz_flight2}
CAMS=${2:-down t45 t30}
BAG=$HOME/datasets/gazebo/$BAG_NAME
VIO=/mnt/c/Users/funfo/vio
STATE=$VIO/results/${BAG_NAME}_state.csv
source /opt/ros/humble/setup.bash

python3 "$VIO/tools/bag_gt_init.py" "$BAG" "$STATE" > "$VIO/results/${BAG_NAME}_state.log" || exit 1
cat "$VIO/results/${BAG_NAME}_state.log"
START=$(awk '/^START_OFFSET/ {print $2}' "$VIO/results/${BAG_NAME}_state.log")

domain=20
for cam in $CAMS; do
    out=$VIO/results/${BAG_NAME}_$cam
    mkdir -p "$out"
    # each run in its own ROS 2 domain, so the playbacks do not reach each other's OpenVINS
    ROS_DOMAIN_ID=$domain bash "$VIO/setup/run_openvins_bag.sh" "$BAG" "$VIO/openvins_config/gz_$cam" "$out" \
        1.0 "$START" "$STATE" > "$out/run.log" 2>&1 &
    domain=$((domain + 1))
    sleep 3
done
wait
for cam in $CAMS; do echo "$cam: $(tail -1 "$VIO/results/${BAG_NAME}_$cam/run.log")"; done
