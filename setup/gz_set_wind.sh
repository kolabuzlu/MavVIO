#!/bin/bash
# Sets the wind in the running Gazebo VIO world (setup/run_gazebo_vio.sh): SPEED m/s from FROM_DEG
# (the direction it blows from, like ArduPilot's SIM_WIND_DIR: 0 = from north, 270 = from west).
# The Zephyr's lift and drag use the air's speed, so it drifts with the wind. ArduPilot's simulated
# airspeed sensor needs the same wind (SIM_WIND_SPD / SIM_WIND_DIR): tools/gz_gps_loss.py --wind sets both.
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/gz_set_wind.sh SPEED FROM_DEG
export GZ_VERSION=harmonic
SPEED=${1:?usage: gz_set_wind.sh SPEED FROM_DEG}
FROM=${2:?usage: gz_set_wind.sh SPEED FROM_DEG}
# the air moves towards FROM + 180 deg; Gazebo x = east, y = north
read -r X Y < <(python3 -c "import math; s, d = $SPEED, math.radians($FROM); print(f'{-s * math.sin(d):.3f} {-s * math.cos(d):.3f}')")
gz topic -t /world/vio_world/wind -m gz.msgs.Wind -p "linear_velocity: {x: $X, y: $Y, z: 0}, enable_wind: true"
echo "Gazebo wind: $SPEED m/s from $FROM deg (the air moves east $X, north $Y m/s)"
