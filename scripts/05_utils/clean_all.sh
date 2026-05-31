#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

rm -rf "$ROOT_DIR/ws/build" "$ROOT_DIR/ws/install" "$ROOT_DIR/ws/log"
printf "Cleaned ws/build, ws/install, and ws/log\n"
