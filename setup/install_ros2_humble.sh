#!/bin/bash
# Installs ROS 2 Humble (desktop) and the OpenVINS build dependencies on Ubuntu 22.04 (WSL).
# Follows the official ROS 2 Humble deb-package instructions (docs.ros.org). Asks for your
# Linux password (sudo); if it runs longer than 15 minutes it may ask again.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/install_ros2_humble.sh
set -e

. /etc/os-release
if [ "$VERSION_CODENAME" != "jammy" ]; then
    echo "This script is for Ubuntu 22.04 (jammy); this system is $PRETTY_NAME. Stopping."
    exit 1
fi

echo "=== 1/6 UTF-8 locale"
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== 2/6 ROS 2 package source"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "=== 3/6 bring Ubuntu up to date first (recommended by the ROS docs)"
sudo apt update
sudo apt upgrade -y

echo "=== 4/6 ROS 2 Humble desktop and developer tools"
sudo apt install -y ros-humble-desktop ros-dev-tools

echo "=== 5/6 OpenVINS build dependencies"
sudo apt install -y git libeigen3-dev libboost-all-dev libceres-dev

echo "=== 6/6 load ROS 2 in every new terminal"
grep -q "/opt/ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo
echo "Done: ROS 2 Humble and the OpenVINS dependencies are installed."
