#!/bin/bash
# Downloads ArduPilot (latest stable ArduPlane release, without history) and ArduPilot's Gazebo
# plugin into your Ubuntu home folder. No password needed. Skips what is already there.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/get_ardupilot_sources.sh
set -e
cd ~
if [ ! -d ardupilot_gazebo ]; then
    git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git
fi
if [ ! -d ardupilot ]; then
    git clone --depth 1 --branch ArduPlane-stable https://github.com/ArduPilot/ardupilot.git
fi
grep -m1 "THISFIRMWARE" ardupilot/ArduPlane/version.h
du -sh ardupilot ardupilot_gazebo
