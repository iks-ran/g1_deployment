#!/usr/bin/env zsh
# Replay a recorded G1 bag in place of the robot (offline). Run in a `G1_DDS=loopback source ./setup.zsh` shell,
# then `ros2 launch g1_bringup g1.launch.py` in another one (same env). Plays only the topics the bringup uses.
#   scripts/play_bag.zsh [bag_dir] [extra ros2 bag play args, e.g. --loop --rate 0.5]
WS=${0:A:h:h}
BAG=${1:-$WS/bags/run_2026-10-24_2052}
(( $# )) && shift
exec ros2 bag play $BAG --topics /lowstate /odommodestate /utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 "$@"
