#!/bin/bash
# Run OpenVINS on a ROS 2 recording without a display and save its estimated trajectory.
# Usage (inside Ubuntu):  bash run_openvins_bag.sh BAG_DIR [CONFIG] [OUT_DIR] [RATE] [START_S] [STATE_FILE]
#   BAG_DIR     ROS 2 bag folder (the one with metadata.yaml)
#   CONFIG      OpenVINS config name, e.g. euroc_mav (default) or tum_vi, or a folder with our own
#               estimator_config.yaml + kalibr files (e.g. /mnt/c/Users/funfo/vio/openvins_config/fw_zed2i)
#   OUT_DIR     where to put ov_estimate.txt, ov_estimate_std.txt and the logs (default /tmp/ov_run)
#   RATE        playback speed, 1.0 = real time (default)
#   START_S     start playing this many seconds into the recording (default 0)
#   STATE_FILE  start OpenVINS from the known state in this file (EuRoC ground-truth csv from
#               tools/bag_gt_init.py) at the first IMU message, instead of its own initializers
#               (needs the change from setup/openvins_init_from_state.py)
# Several runs can go side by side if each gets its own ROS_DOMAIN_ID (setup/openvins_cameras_gz.sh).
set -e
BAG=$1
CONFIG=${2:-euroc_mav}
OUT=${3:-/tmp/ov_run}
RATE=${4:-1.0}
START_S=${5:-0}
STATE_FILE=${6:-}
STATE_ARGS=()
[ -n "$STATE_FILE" ] && STATE_ARGS=(-p path_gt:="$STATE_FILE" -p init_from_gt:=true)
[ -f "$BAG/metadata.yaml" ] || { echo "not a ROS 2 bag folder: $BAG"; exit 1; }

source /opt/ros/humble/setup.bash
source ~/ws_ov/install/setup.bash
mkdir -p "$OUT"
TMP=$(mktemp -d /tmp/ov_run.XXXXXX)   # this run's own files
if [ -f "$CONFIG/estimator_config.yaml" ]; then   # a folder with our own config files
    CONFIG_PATH="$CONFIG/estimator_config.yaml"
else                                             # one of OpenVINS's built-in configs
    CONFIG_PATH="$(ros2 pkg prefix ov_msckf)/share/ov_msckf/config/$CONFIG/estimator_config.yaml"
fi
[ -f "$CONFIG_PATH" ] || { echo "no OpenVINS config '$CONFIG' ($CONFIG_PATH)"; exit 1; }
# mono or stereo as the config says (the node parameters below override the file)
STEREO=$(awk '/^use_stereo:/ {print $2}' "$CONFIG_PATH")
NCAM=$(awk '/^max_cameras:/ {print $2}' "$CONFIG_PATH")
# play only the topics this config listens to (a recording may hold several cameras)
TOPICS=$(awk '/rostopic:/ {print $2}' "$(dirname "$CONFIG_PATH")"/kalibr_imu*.yaml 2>/dev/null | tr -d '"\r')

# Started directly instead of through subscribe.launch.py: in ROS 2, OpenVINS only reads the
# filepath_* settings from node parameters, which the launch file does not pass - without them it
# falls back to a bare file name and crashes with "create_directories: Invalid argument".
ros2 run ov_msckf run_subscribe_msckf --ros-args -r __ns:=/ov_msckf \
    -p config_path:="$CONFIG_PATH" -p verbosity:=INFO -p use_stereo:=${STEREO:-true} -p max_cameras:=${NCAM:-2} \
    -p save_total_state:=true -p filepath_est:="$TMP/ov_estimate.txt" \
    -p filepath_std:="$TMP/ov_estimate_std.txt" -p filepath_gt:="$TMP/ov_groundtruth.txt" "${STATE_ARGS[@]}" \
    > "$OUT/openvins.log" 2>&1 &
LAUNCH_PID=$!
sleep 5   # give OpenVINS time to start and subscribe before the recording plays

echo "playing $BAG at ${RATE}x from ${START_S} s ..."
ros2 bag play "$BAG" --rate "$RATE" --start-offset "$START_S" ${TOPICS:+--topics $TOPICS} > "$OUT/bag_play.log" 2>&1
sleep 3   # let OpenVINS finish the last frames

# Stop this run's OpenVINS; it sometimes ignores the request, so force it after 10 s.
NODE="lib/ov_msckf/run_subscribe_msckf.*$TMP/"
pkill -INT -f "$NODE" 2>/dev/null || true
for i in $(seq 1 10); do pgrep -f "$NODE" >/dev/null || break; sleep 1; done
pkill -KILL -f "$NODE" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
cp "$TMP/ov_estimate.txt" "$TMP/ov_estimate_std.txt" "$OUT/" 2>/dev/null || true
rm -rf "$TMP"
echo "OpenVINS estimate: $(wc -l < "$OUT/ov_estimate.txt" 2>/dev/null || echo 0) states saved in $OUT"
