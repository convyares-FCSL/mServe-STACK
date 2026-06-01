#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

source "$ROOT_DIR/scripts/01_setup/env_setup.sh"
cd "$ROOT_DIR/ws"
source install/setup.bash

colcon test --packages-select mserve_utils mserve_base mserve_drivechain --event-handlers console_direct+
colcon test-result --verbose
