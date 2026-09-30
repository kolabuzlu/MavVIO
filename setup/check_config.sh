#!/bin/bash
# Start OpenVINS for a few seconds with each given config folder (no data) and report whether it loads.
# Usage (inside Ubuntu):  bash check_config.sh CONFIG_DIR [CONFIG_DIR ...]
source /opt/ros/humble/setup.bash
source ~/ws_ov/install/setup.bash
for cfg in "$@"; do
    log=$(timeout 8 ros2 run ov_msckf run_subscribe_msckf --ros-args -r __ns:=/ov_check \
        -p config_path:="$cfg/estimator_config.yaml" -p verbosity:=INFO 2>&1)
    if echo "$log" | grep -q "subscribing to IMU"; then result="loads fine"
    elif echo "$log" | grep -qi "segmentation"; then result="SEGMENTATION FAULT"
    else result="unclear"; fi
    echo "$(basename "$cfg"): $result"
    echo "$log" | grep -iE "error|fault|could not|failed|unable" | head -n 3 | sed 's/^/    /'
done
