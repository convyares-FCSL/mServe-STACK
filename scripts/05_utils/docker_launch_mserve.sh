#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$ROOT_DIR"

docker compose exec robot-mserve bash -lc '
  cd /ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  export AMENT_PREFIX_PATH=/ws/install/mserve_bringup:/ws/install/mserve_description:/ws/install/mserve_base:/ws/install/mserve_drivechain:/ws/install/mserve_utils:/ws/install/mserve_interfaces:$AMENT_PREFIX_PATH &&
  ros2 launch mserve_bringup mserve_min.launch.py
'
