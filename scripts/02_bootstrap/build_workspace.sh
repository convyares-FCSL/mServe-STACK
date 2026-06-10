#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

source "$ROOT_DIR/scripts/01_setup/env_setup.sh"
cd "$ROOT_DIR/ws"

colcon build --symlink-install --allow-overriding launch --packages-select \
  interfaces \
  utils \
  mserve_base \
  mserve_drivechain \
  mserve_description \
  launch \
  lifecycle_manager
