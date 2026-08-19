#!/usr/bin/env bash
# Command mode, for systemd.
#
# Exists because systemd cannot `source` a setup file, and ROS 2 is entirely
# built around sourcing two of them: the distro overlay and this workspace's.
# Without both, `ros2 launch` is not on PATH and robot_interfaces cannot be
# imported.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

# Keep DDS off the wifi. Without this every ROS process multicasts to the whole
# subnet looking for peers, which costs real CPU on a Pi and finds your laptop.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONUNBUFFERED=1

cd "$REPO"
exec ros2 launch robot_bringup command.launch.py "$@"
