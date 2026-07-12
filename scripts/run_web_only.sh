#!/usr/bin/env bash
# Standalone static web server (no ROS nodes, no rosbridge) — for when the
# drive stack is already running some other way and you just need the page.
# Most of the time you want scripts/run_stack.sh instead.
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
cd "$ROOT_DIR/web"
python3 -m http.server 6240
