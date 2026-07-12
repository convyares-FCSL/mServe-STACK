#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

source "$ROOT_DIR/scripts/setup/env_setup.sh"
cd "$ROOT_DIR/ws"

colcon build --symlink-install --allow-overriding launch --cmake-args -DBUILD_TESTING=OFF --packages-select \
  interfaces \
  utils \
  mserve_base \
  mserve_drivechain \
  mserve_description \
  launch \
  btcpp_ros2_interfaces \
  behaviortree_ros2 \
  lifecycle_manager
