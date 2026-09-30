#!/bin/bash
# The VIO on the plane: RealSense D455 looking straight down (top of the picture towards the nose)
#   -> OpenVINS (openvins_config/d455_800, waiting for its start state) <-> tools/vio_bridge.py
#   <-> the flight controller over a serial cable (MAVLink).
# The same chain as the simulator's setup/run_vio_live.sh - only the camera driver and the link differ.
#
# NOT YET TESTED ON THE REAL HARDWARE. README "D455 bring-up" has the order: bench first, then flights
# with the VIO only logged, before the flight controller may use it.
#
# Needs on the Jetson (JetPack 6 = Ubuntu 22.04): ROS 2 Humble, ros-humble-realsense2-camera,
# OpenVINS built in ~/ws_ov with setup/openvins_init_from_state.py applied, pymavlink (pip), this folder.
# Settings (environment variables):
#   MAVLINK  flight controller link, default /dev/ttyTHS0,921600 - the UART on the 40-pin header (pins 8 TX,
#            10 RX; check which /dev/ttyTHS* it is on your board). The user must be in the dialout group.
#            On the flight controller: SERIALx_PROTOCOL 2, SERIALx_BAUD 921 (params/vio_plane.parm).
#   LEVER    where the camera sits relative to the flight controller: FORWARD,RIGHT,DOWN in metres
#   RES      1280x800 (default: the whole sensor, openvins_config/d455_800) or 848x480 (openvins_config/d455) -
#            in the simulator's side-by-side test 1280x800 drifted less (README, "Choosing the camera settings")
#   RECORD   1 = also record the camera and IMU (ros2 bag, about 1 GB a minute) to replay flights later
#   CAMERA_ONLY  1 = only the camera (and RECORD): for the hand-held OpenVINS test and calibration recordings
#   LOG_DIR  default ~/vio_logs/<date>_<time>
# Run:  bash setup/run_vio_d455.sh        (Ctrl+C stops it all)
source /opt/ros/humble/setup.bash
source ~/ws_ov/install/setup.bash
VIO=$(cd "$(dirname "$0")/.." && pwd)
LOG=${LOG_DIR:-$HOME/vio_logs/$(date +%Y%m%d_%H%M%S)}
MAVLINK=${MAVLINK:-/dev/ttyTHS0,921600}
RES=${RES:-1280x800}
case $RES in
    1280x800) CONFIG=$VIO/openvins_config/d455_800/estimator_config.yaml ;;
    848x480) CONFIG=$VIO/openvins_config/d455/estimator_config.yaml ;;
    *) echo "RES must be 1280x800 or 848x480"; exit 1 ;;
esac
mkdir -p "$LOG"
dev=${MAVLINK%%,*}
if [[ $dev == /dev/* && ! -e $dev ]]; then
    echo "no $dev - serial ports here: $(ls /dev/ttyTHS* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
    exit 1
fi
# OpenVINS gets SIGKILL: after SIGINT it crashes on the way out (and a crash dump each time fills the disk)
trap 'kill $(jobs -p) 2>/dev/null; pkill -KILL -f "lib/ov_msckf/[r]un_subscribe_msckf"; pkill -INT -f "[r]ealsense2_camera_node"' EXIT

# The camera: colour $RES at 30 per second (the D455's colour sensor has a global shutter), IMU at 400
# per second with gyro and accelerometer in one message. No depth, no infrared, projector off - a VIO at
# 50-150 m has no use for them. The node is named d455, so its topics are /d455/... as the config expects.
# From RealSense users' outdoor VIO experience (realsense-ros issue 3321): auto-exposure priority off (it
# lowers the frame rate to lengthen exposure), no frame sync (it holds the colour picture back for other
# streams), accelerometer 200 + gyro 400 per second united by interpolation. Also worth trying on the bench:
# backlight compensation off (ros2 param set /d455 rgb_camera.backlight_compensation ...).
# (in a loop: a USB reset can take the driver down - it comes back, and the companion restarts the VIO)
(
    while true; do
        echo "===== camera start, $(date +%T)" >> "$LOG/camera.log"
        ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=d455 \
            -p initial_reset:=true -p enable_color:=true -p rgb_camera.color_profile:=${RES/x/,},30 \
            -p rgb_camera.auto_exposure_priority:=false \
            -p enable_depth:=false -p enable_infra1:=false -p enable_infra2:=false -p depth_module.emitter_enabled:=0 \
            -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=400 -p accel_fps:=200 -p unite_imu_method:=2 \
            -p enable_sync:=false \
            >> "$LOG/camera.log" 2>&1
        sleep 2
    done
) &
for i in $(seq 30); do
    topics=$(ros2 topic list 2>/dev/null)
    grep -qx /d455/imu <<< "$topics" && grep -qx /d455/color/image_raw <<< "$topics" && break
    sleep 1
done
if ! grep -qx /d455/imu <<< "$topics" || ! grep -qx /d455/color/image_raw <<< "$topics"; then
    echo "no /d455/imu or /d455/color/image_raw after 30 s - see $LOG/camera.log"
    exit 1
fi
# OpenVINS's numbers are for $RES - a driver that ignored the profile setting would give another size
info=$(timeout 10 ros2 topic echo --once /d455/color/camera_info 2>/dev/null)
size=$(awk '/^width:/ {w=$2} /^height:/ {h=$2} END {print w "x" h}' <<< "$info")
if [ "$size" != "$RES" ]; then
    echo "the camera gives $size pictures, OpenVINS expects $RES - check the profile setting ($LOG/camera.log)"
    exit 1
fi
echo "$info" > "$LOG/camera_info.txt"      # this camera's factory numbers, for comparing with the calibration
echo "camera up: $RES colour and IMU"

if [ "${RECORD:-0}" = 1 ]; then
    ros2 bag record -o "$LOG/bag" --compression-mode message --compression-format zstd \
        /d455/color/image_raw /d455/color/camera_info /d455/imu /ov_msckf/odomimu /ov_msckf/init_state \
        /fc/odom > "$LOG/record.log" 2>&1 &
fi

if [ "${CAMERA_ONLY:-0}" = 1 ]; then
    echo "camera only - Ctrl+C stops it"
    wait
    exit
fi

# The flight controller's VIO settings and the free space, before the companion takes the serial port
# (only warns: the table is in preflight.txt, one line goes to the ground station as "VIO check: ...")
python3 -u "$VIO/tools/preflight_check.py" --mavlink "$MAVLINK" --log-dir "$(dirname "$LOG")" \
    $([ "${RECORD:-0}" = 1 ] && echo --record) 2>&1 | tee "$LOG/preflight.txt" || true

# OpenVINS, started again whenever it stops (the companion stops it to refresh the VIO)
(
    run=0
    while true; do
        run=$((run + 1))
        echo "===== OpenVINS start $run, $(date +%T)" >> "$LOG/openvins.log"
        ros2 run ov_msckf run_subscribe_msckf --ros-args -r __ns:=/ov_msckf \
            -p config_path:="$CONFIG" -p verbosity:=WARNING -p use_stereo:=false -p max_cameras:=1 \
            -p init_from_topic:=true -p save_total_state:=true -p filepath_est:="$LOG/ov_estimate_$run.txt" \
            -p filepath_std:="$LOG/ov_estimate_std_$run.txt" -p filepath_gt:="$LOG/ov_groundtruth_$run.txt" \
            >> "$LOG/openvins.log" 2>&1
        sleep 1
    done
) &

python3 -u "$VIO/tools/vio_bridge.py" --mavlink "$MAVLINK" --mount realsense-down --imu-topic /d455/imu \
    --camera-topic /d455/color/camera_info \
    ${LEVER:+--lever "$LEVER"} --log "$LOG/vio_bridge.csv" > "$LOG/vio_bridge.log" 2>&1 &
echo "VIO chain running - logs in $LOG"
wait
