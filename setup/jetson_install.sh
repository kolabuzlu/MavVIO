#!/bin/bash
# One-shot setup of the plane's Jetson (Orin Nano Super, JetPack 6 = Ubuntu 22.04, arm64) for the VIO:
#   ROS 2 Humble (base), the RealSense driver, OpenVINS (the same version as on the desktop) with the
#   "start from a given state" change, pymavlink, serial port access, and the start-on-boot service (not
#   switched on). NOT YET TESTED ON A JETSON: it stops at the first error - read the message; the likely
#   snags are named below. Asks for your password (sudo). OpenVINS takes ~30-60 minutes to compile.
# Run on the Jetson, in this folder copied over (e.g. ~/vio):  bash setup/jetson_install.sh
set -e
. /etc/os-release
[ "$VERSION_CODENAME" = jammy ] || { echo "needs Ubuntu 22.04 (JetPack 6); this is $PRETTY_NAME"; exit 1; }
[ "$(uname -m)" = aarch64 ] || echo "note: this is $(uname -m), not a Jetson - carrying on anyway"
VIO=$(cd "$(dirname "$0")/.." && pwd)
OPENVINS_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b    # the version the change was made for (2025-11-30)

echo "=== 1/7 ROS 2 Humble package source"
sudo apt update
sudo apt install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe
if ! dpkg -s ros2-apt-source > /dev/null 2>&1; then
    V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${UBUNTU_CODENAME:-jammy}_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
fi
sudo apt update

echo "=== 2/7 ROS 2 Humble (base - no desktop tools on the plane) and build tools"
# Known JetPack snag: its own OpenCV 4.8 packages can clash with the OpenCV ROS's cv_bridge wants. If apt
# says so, look at: dpkg -l | grep -i opencv  - removing NVIDIA's OpenCV packages lets ROS bring its own.
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-numpy python3-pip
sudo apt install -y libeigen3-dev libboost-all-dev libceres-dev ros-humble-cv-bridge ros-humble-image-transport

echo "=== 3/7 RealSense driver (librealsense + realsense2_camera)"
# Intel's advice for Jetsons is librealsense with the RSUSB back end (no kernel patches; the IMU works).
# The ROS packages are tried first; if rs-enumerate-devices then does not list the accelerometer and gyro,
# build librealsense from source with -DFORCE_RSUSB_BACKEND=true (github.com/IntelRealSense/librealsense).
sudo apt install -y ros-humble-realsense2-camera ros-humble-librealsense2-tools || \
    sudo apt install -y ros-humble-realsense2-camera

echo "=== 4/7 OpenVINS $OPENVINS_COMMIT in ~/ws_ov, with the vio project change"
mkdir -p ~/ws_ov/src
if [ ! -d ~/ws_ov/src/open_vins ]; then
    git clone https://github.com/rpng/open_vins.git ~/ws_ov/src/open_vins
fi
git -C ~/ws_ov/src/open_vins fetch --quiet origin
git -C ~/ws_ov/src/open_vins checkout --quiet "$OPENVINS_COMMIT"
python3 "$VIO/setup/openvins_init_from_state.py"
source /opt/ros/humble/setup.bash
cd ~/ws_ov
# the Orin Nano's 8 GB are shared with the graphics: one package at a time, 4 compile jobs
MAKEFLAGS="-j4" colcon build --executor sequential --packages-select ov_core ov_init ov_msckf \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
grep -q "ws_ov/install/setup.bash" ~/.bashrc || echo "source ~/ws_ov/install/setup.bash" >> ~/.bashrc
cd "$VIO"

echo "=== 5/7 pymavlink (for the companion) and serial port access"
pip3 install --user pymavlink
sudo usermod -aG dialout "$USER"

echo "=== 6/7 start-on-boot service (installed, NOT switched on)"
[ -f "$VIO/setup/vio.env" ] || cp "$VIO/setup/vio.env.example" "$VIO/setup/vio.env"
sed -e "s#@USER@#$USER#g" -e "s#@VIO@#$VIO#g" "$VIO/setup/vio.service" | sudo tee /etc/systemd/system/vio.service > /dev/null
sudo systemctl daemon-reload

echo "=== 7/7 checks"
source ~/ws_ov/install/setup.bash
ros2 pkg prefix ov_msckf > /dev/null && echo "OpenVINS: ok"
ros2 pkg prefix realsense2_camera > /dev/null && echo "realsense2_camera: ok"
python3 -c "import pymavlink" && echo "pymavlink: ok"
command -v rs-enumerate-devices > /dev/null && rs-enumerate-devices -s || echo "(rs-enumerate-devices not found)"
echo
echo "Done. Next:"
echo "  - log out and in again (serial port access)"
echo "  - power mode: sudo nvpmodel -q --verbose lists the modes - pick the fastest (MAXN SUPER); sudo jetson_clocks"
echo "  - edit $VIO/setup/vio.env (the flight controller's serial port, the camera position)"
echo "  - bench test: bash setup/run_vio_d455.sh   (README, D455 bring-up)"
echo "  - only after the tests: sudo systemctl enable vio   (starts the VIO chain at every boot)"
