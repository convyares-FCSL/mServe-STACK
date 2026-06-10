#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$ROOT_DIR"

docker compose up -d robot-mserve >/dev/null

if ! docker compose exec -T robot-mserve test -f /ws/install/setup.bash; then
  echo "ERROR: /ws/install/setup.bash was not found inside the container."
  echo "The workspace has not been built yet."
  echo
  echo "Build it first with:"
  echo "  scripts/05_utils/docker_build_workspace.sh"
  echo
  echo "Or run the raw build command:"
  echo "  docker compose exec robot-mserve bash -lc 'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --symlink-install --allow-overriding launch --packages-select interfaces utils mserve_base mserve_drivechain mserve_description mserve_bringup'"
  exit 1
fi

docker compose exec robot-mserve bash -lc '
  cd /ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  export AMENT_PREFIX_PATH=/ws/install/mserve_bringup:/ws/install/mserve_description:/ws/install/mserve_base:/ws/install/mserve_drivechain:/ws/install/utils:/ws/install/interfaces:$AMENT_PREFIX_PATH &&
  ros2 launch mserve_bringup mserve_min.launch.py
'
