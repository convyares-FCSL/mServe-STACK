#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$ROOT_DIR"

docker compose up -d --build robot-mserve

docker compose exec robot-mserve bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /ws &&
  colcon build --symlink-install \
    --packages-select \
      mserve_interfaces \
      mserve_utils \
      mserve_base \
      mserve_drivechain \
      mserve_description \
      mserve_bringup
'
