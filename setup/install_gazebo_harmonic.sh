#!/bin/bash
# Installs Gazebo Harmonic, what ArduPilot's Gazebo plugin and simulator need to build, and the
# Gazebo <-> ROS 2 Humble bridge, on Ubuntu 22.04 (WSL). Follows the official Gazebo instructions
# (gazebosim.org/docs/harmonic/install_ubuntu) and the ardupilot_gazebo README.
# Asks for your Linux password (sudo). About 2 GB of downloads.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/install_gazebo_harmonic.sh
set -e

. /etc/os-release
if [ "$VERSION_CODENAME" != "jammy" ]; then
    echo "This script is for Ubuntu 22.04 (jammy); this system is $PRETTY_NAME. Stopping."
    exit 1
fi

echo "=== 1/4 Gazebo package source (packages.osrfoundation.org)"
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

echo "=== 2/4 Gazebo Harmonic"
sudo apt-get install -y gz-harmonic

echo "=== 3/4 build dependencies: ArduPilot's Gazebo plugin and ArduPilot's simulator"
sudo apt-get install -y libgz-sim8-dev rapidjson-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    python3-venv mesa-utils

echo "=== 4/4 Gazebo <-> ROS 2 Humble bridge (for the camera and IMU in the next step)"
sudo apt-get install -y ros-humble-ros-gzharmonic || echo "NOTE: the ROS 2 bridge did not install - not needed yet, we'll look at it in step 2"

echo
gz sim --versions
df -h / | tail -1
echo "Done: Gazebo Harmonic is installed."
