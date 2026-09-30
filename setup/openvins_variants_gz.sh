#!/bin/bash
# Tuning experiments on the recorded Gazebo flight: makes variants of openvins_config/gz_zephyr and runs
# OpenVINS on the same recording with each (one after another, at normal playback speed).
# Run inside Ubuntu:  bash /mnt/c/Users/funfo/vio/setup/openvins_variants_gz.sh [BAG_NAME]
BAG_NAME=${1:-gz_flight1}
BAG=$HOME/datasets/gazebo/$BAG_NAME
VIO=/mnt/c/Users/funfo/vio
BASE=$VIO/openvins_config/gz_zephyr

make_variant() {   # NAME then sed expressions for estimator_config.yaml, then -- and for kalibr_imu_chain.yaml
    local name=$1; shift
    local dir=$VIO/openvins_config/$name
    rm -rf "$dir"; cp -r "$BASE" "$dir"
    local est=() imu=() target=est
    for a in "$@"; do
        if [ "$a" = "--" ]; then target=imu; continue; fi
        if [ $target = est ]; then est+=(-e "$a"); else imu+=(-e "$a"); fi
    done
    [ ${#est[@]} -gt 0 ] && sed -i "${est[@]}" "$dir/estimator_config.yaml"
    [ ${#imu[@]} -gt 0 ] && sed -i "${imu[@]}" "$dir/kalibr_imu_chain.yaml"
    echo "$dir"
}

A=$(make_variant gz_zephyr_rw -- \
    's/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.002 # 10x, lets the bias adapt/' \
    's/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 0.0002 # 10x, lets the bias adapt/')
B=$(make_variant gz_zephyr_clones \
    's/^max_clones: .*/max_clones: 20 # longer window, longer baselines/' \
    's/^max_slam: .*/max_slam: 75 # more long-lived map points/')
C=$(make_variant gz_zephyr_rw_clones \
    's/^max_clones: .*/max_clones: 20 # longer window, longer baselines/' \
    's/^max_slam: .*/max_slam: 75 # more long-lived map points/' -- \
    's/^  accelerometer_random_walk: .*/  accelerometer_random_walk: 0.002 # 10x, lets the bias adapt/' \
    's/^  gyroscope_random_walk: .*/  gyroscope_random_walk: 0.0002 # 10x, lets the bias adapt/')

for cfg in "$A" "$B" "$C"; do
    name=$(basename "$cfg")
    out=$VIO/results/${BAG_NAME}_${name#gz_zephyr_}
    mkdir -p "$out"
    echo "=== $name -> $out"
    bash "$VIO/setup/run_openvins_bag.sh" "$BAG" "$cfg" "$out" > "$out/run.log" 2>&1
    tail -1 "$out/run.log"
done
echo "all variants done"
