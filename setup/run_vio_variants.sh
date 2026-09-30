#!/bin/bash
# Side-by-side test of OpenVINS settings on the same live camera. Next to the VIO that setup/run_vio_live.sh
# runs for ArduPilot, one more OpenVINS per variant (openvins_config/variants/NAME), each in its own ROS
# namespace (/ov_v_NAME), all started from the same start state (/ov_msckf/init_state) at the same moment -
# and restarted with it (the companion's restart stops every OpenVINS). Nothing of theirs reaches ArduPilot:
# tools/ros_truth_log.py logs their live estimates (LOG_DIR/odom_NAME.csv) and the simulator's truth
# (LOG_DIR/truth.csv) for tools/compare_variants.py; OpenVINS's own files (est_NAME_N.txt) lose their last
# half minute when the companion's restart kills it.
# Needs the 1280 x 800 world (make_scenery.py --model-only --no-chase --camera d455-800) and run_vio_live.sh
# started with VIO_CONFIG=gz_d455_800 MONO_ARGS="--sd 848x480" (the sd variants use the 848 x 480 copy).
# Run inside Ubuntu, after run_vio_live.sh:  bash run_vio_variants.sh LOG_DIR [NAME ...]   (default: all)
source /opt/ros/humble/setup.bash
source ~/ws_ov/install/setup.bash
VIO=/mnt/c/Users/funfo/vio
LOG=${1:?usage: run_vio_variants.sh LOG_DIR [NAME ...]}
shift
NAMES=${*:-$(ls "$VIO/openvins_config/variants")}
mkdir -p "$LOG"
# SIGKILL for OpenVINS: after SIGINT it crashes on the way out, and WSL keeps a big dump of each crash
trap 'kill $(jobs -p) 2>/dev/null; pkill -KILL -f "lib/ov_msckf/[r]un_subscribe_msckf.*__ns:=/ov_v_"' EXIT

python3 "$VIO/tools/ros_truth_log.py" "$LOG" --odom $NAMES > "$LOG/truth_log.log" 2>&1 &
for name in $NAMES; do
    (
        run=0
        while true; do
            run=$((run + 1))
            ros2 run ov_msckf run_subscribe_msckf --ros-args -r __ns:=/ov_v_$name \
                -r /ov_v_$name/init_state:=/ov_msckf/init_state \
                -p config_path:="$VIO/openvins_config/variants/$name/estimator_config.yaml" -p verbosity:=WARNING \
                -p use_stereo:=false -p max_cameras:=1 -p init_from_topic:=true -p save_total_state:=true \
                -p filepath_est:="$LOG/est_${name}_$run.txt" -p filepath_std:="$LOG/std_${name}_$run.txt" \
                -p filepath_gt:="$LOG/gt_${name}_$run.txt" >> "$LOG/ov_$name.log" 2>&1
            sleep 1
        done
    ) &
done
echo "variants running: $(echo $NAMES) - logs in $LOG"
wait
