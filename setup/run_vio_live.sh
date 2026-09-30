#!/bin/bash
# Step 3: the VIO running live in the Gazebo simulation, as it will on the plane's Jetson:
#   Gazebo camera + VIO IMU -> ros_gz_bridge -> grey pictures (tools/ros_rgb_to_mono.py) -> OpenVINS
#   (openvins_config/$VIO_CONFIG, waiting for its start state) <-> tools/vio_bridge.py <-> ArduPilot SITL
#   (MAVLink TCP 5763). The bridge starts OpenVINS from the flight controller's state once the plane
#   flies steadily above 40 m with GPS, feeds the VIO to ArduPilot, and restarts it now and then to keep
#   it fresh - OpenVINS runs in a loop here, so a stopped OpenVINS comes straight back and waits for its
#   next start state. (The plane's version of this script: setup/run_vio_d455.sh.)
# VIO_CONFIG: gz_d455_800 (default - the D455's whole 1280x800 sensor, make_scenery.py --camera d455-800,
#   every 2nd picture), gz_d455 (848x480, --camera d455) or gz_down (the older 640x480 camera, --camera
#   sim640) - the config must match the world's camera. BRIDGE_ARGS: extra
#   options for vio_bridge.py; MONO_ARGS: for ros_rgb_to_mono.py (--sd 848x480 for setup/run_vio_variants.sh).
# Start after setup/run_gazebo_vio.sh is up; Ctrl+C (or killing this script) stops it all.
# Logs, OpenVINS's paths (one per VIO start) and the bridge's CSV go to LOG_DIR (default ~/sitl_gazebo/vio_live).
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/run_vio_live.sh [LOG_DIR]
source /opt/ros/humble/setup.bash
source ~/ws_ov/install/setup.bash
export GZ_VERSION=harmonic
VIO=/mnt/c/Users/funfo/vio
LOG=${1:-$HOME/sitl_gazebo/vio_live}
CONFIG=$VIO/openvins_config/${VIO_CONFIG:-gz_d455_800}/estimator_config.yaml
[ -f "$CONFIG" ] || { echo "no OpenVINS config $CONFIG"; exit 1; }
mkdir -p "$LOG"
# OpenVINS gets SIGKILL: after SIGINT it crashes on the way out, and WSL keeps a big dump of each crash
trap 'kill $(jobs -p) 2>/dev/null; pkill -KILL -f "lib/ov_msckf/[r]un_subscribe_msckf"' EXIT

ros2 run ros_gz_bridge parameter_bridge \
    '/vio/camera@sensor_msgs/msg/Image[gz.msgs.Image' \
    '/vio/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
    '/model/zephyr_vio/pose@geometry_msgs/msg/TransformStamped[gz.msgs.Pose' \
    --ros-args -r /vio/camera:=/vio/camera_rgb -r /model/zephyr_vio/pose:=/vio/truth > "$LOG/gz_bridge.log" 2>&1 &
python3 "$VIO/tools/ros_rgb_to_mono.py" camera $MONO_ARGS > "$LOG/mono.log" 2>&1 &

# OpenVINS, started again whenever it stops (the bridge stops it to refresh the VIO)
(
    run=0
    while true; do
        run=$((run + 1))
        echo "===== OpenVINS start $run, $(date +%T)" >> "$LOG/openvins.log"
        ros2 run ov_msckf run_subscribe_msckf --ros-args -r __ns:=/ov_msckf \
            -p config_path:="$CONFIG" -p verbosity:=INFO -p use_stereo:=false -p max_cameras:=1 \
            -p init_from_topic:=true -p save_total_state:=true -p filepath_est:="$LOG/ov_estimate_$run.txt" \
            -p filepath_std:="$LOG/ov_estimate_std_$run.txt" -p filepath_gt:="$LOG/ov_groundtruth_$run.txt" \
            >> "$LOG/openvins.log" 2>&1
        sleep 1
    done
) &

# pymavlink comes from the ArduPilot build tools' venv (added after the ROS paths)
PYTHONPATH=$PYTHONPATH:$HOME/venv-ardupilot/lib/python3.10/site-packages \
    python3 -u "$VIO/tools/vio_bridge.py" --log "$LOG/vio_bridge.csv" $BRIDGE_ARGS > "$LOG/vio_bridge.log" 2>&1 &
echo "VIO chain running ($(basename "$(dirname "$CONFIG")")) - logs in $LOG"
wait
