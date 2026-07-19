#!/usr/bin/env bash
# Standalone Foxglove Bridge starter — for when the drive stack is already
# running some other way (scripts/run_stack.sh, or the systemd service) and
# you just want to point Foxglove Studio/Desktop at the robot.
#
# Usage:
#   ./scripts/run_foxglove_bridge.sh [port]   # default port 8765
#
# In Foxglove: Open Connection -> Foxglove WebSocket (NOT "Rosbridge" -
# that's a different protocol on a different port/server) -> ws://<pi-ip>:8765
set -eo pipefail
# Deliberately not `set -u` — /opt/ros/lyrical/setup.bash references
# AMENT_TRACE_SETUP_FILES without a default and isn't nounset-safe; sourcing
# it under -u aborts immediately (see run_stack.sh, which avoids -u for the
# same reason).
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
PORT="${1:-8765}"

if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/lyrical/setup.bash
fi

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "ERROR: foxglove_bridge not installed."
  echo "       sudo apt install ros-lyrical-foxglove-bridge"
  exit 1
fi

# Source the whole workspace, not just interfaces: safe here because we
# invoke the node directly via
# `ros2 run` below, not `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
# — this repo has a package literally named "launch" (see
# ws/src/launch/package.xml's comment) that shadows the real ROS 2 `launch`
# framework package once the workspace is sourced, which breaks the XML
# launch frontend specifically. `ros2 run` never touches that frontend, so
# there's nothing to work around — full sourcing means every custom msg/srv
# package (interfaces, slam_toolbox, ...) resolves automatically, including
# ones added after this script was written.
WS_SETUP="$ROOT_DIR/ws/install/setup.bash"
if [[ -f "$WS_SETUP" ]]; then
  source "$WS_SETUP"
else
  echo "WARNING: $WS_SETUP not found — build the workspace first."
  echo "         Custom message/service schemas won't resolve without it."
fi

echo "Starting Foxglove Bridge on ws://0.0.0.0:$PORT"
exec ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:="$PORT"
