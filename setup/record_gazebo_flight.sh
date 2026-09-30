#!/bin/bash
# Records the plane's VIO cameras (downward /vio/camera and the tilted /vio/camera_t30, /vio/camera_t45),
# VIO IMU and true pose from the running Gazebo VIO simulation (run_gazebo_vio.sh) into a compressed
# ROS 2 bag ~/datasets/gazebo/NAME, for OpenVINS (run_openvins_bag.sh with openvins_config/gz_*).
# Stops after SECONDS (default 420) or Ctrl+C. CAMERAS="camera camera_t30 camera_t45" also records the
# tilted cameras (world made with make_scenery.py --tilts 30,45); default: the downward camera only.
# The cameras draw in colour (Gazebo's grey format is far too dark); tools/ros_rgb_to_mono.py turns the
# pictures grey on the way, so the recording holds grey (mono8) pictures like a real VIO camera's.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/record_gazebo_flight.sh NAME [SECONDS]
set -e
NAME=${1:?usage: record_gazebo_flight.sh NAME [SECONDS]}
DURATION=${2:-420}
CAMERAS=${CAMERAS:-camera}
OUT=$HOME/datasets/gazebo/$NAME
[ -e "$OUT" ] && { echo "$OUT already exists"; exit 1; }
mkdir -p "$HOME/datasets/gazebo"
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic

# Gazebo -> ROS 2 (header times are simulation time); colour pictures arrive as /vio/CAM_rgb
IMAGES=() TOPICS=() REMAPS=()
for c in $CAMERAS; do
    IMAGES+=("/vio/$c@sensor_msgs/msg/Image[gz.msgs.Image"); TOPICS+=("/vio/$c"); REMAPS+=(-r "/vio/$c:=/vio/${c}_rgb")
done
ros2 run ros_gz_bridge parameter_bridge "${IMAGES[@]}" \
    '/vio/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
    '/vio/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
    '/model/zephyr_vio/pose@geometry_msgs/msg/TransformStamped[gz.msgs.Pose' \
    --ros-args -r /model/zephyr_vio/pose:=/vio/truth "${REMAPS[@]}" > /tmp/gz_bridge.log 2>&1 &
BRIDGE=$!
python3 /mnt/c/Users/funfo/vio/tools/ros_rgb_to_mono.py $CAMERAS > /tmp/gz_mono.log 2>&1 &
MONO=$!
trap 'kill $BRIDGE $MONO 2>/dev/null' EXIT
sleep 4
echo "recording $DURATION s into $OUT ..."
timeout -s INT "$DURATION" ros2 bag record -o "$OUT" --compression-mode message --compression-format zstd \
    "${TOPICS[@]}" /vio/camera_info /vio/imu /vio/truth > /tmp/gz_record.log 2>&1 || true
sleep 2
ros2 bag info "$OUT" | grep -E "Duration|Messages|Topic:|Bag size"
