#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

source "$ROOT_DIR/scripts/01_setup/env_setup.sh"
cd "$ROOT_DIR/ws"

if [ "$#" -eq 0 ]; then
  echo "Usage: $0 <package1> [package2 ...]"
  exit 1
fi

colcon build --symlink-install --packages-select "$@"
