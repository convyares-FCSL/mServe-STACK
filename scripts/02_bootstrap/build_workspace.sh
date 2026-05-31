#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

source "$ROOT_DIR/scripts/01_setup/env_setup.sh"
cd "$ROOT_DIR/ws"

colcon build --symlink-install --packages-select mserve_interfaces mserve_utils mserve_base mserve_esp32 mserve_bringup
