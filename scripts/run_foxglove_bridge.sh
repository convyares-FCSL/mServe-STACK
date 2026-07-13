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

# Deliberately source ONLY interfaces' local_setup.bash, not the workspace's
# aggregate install/setup.bash: this repo has a package literally named
# "launch" (see ws/src/launch/package.xml's comment on the same collision),
# which shadows the real ROS 2 `launch` framework package on the ament index
# once sourced — and foxglove_bridge_launch.xml needs the real one to parse
# at all ("FileNotFoundError: .../install/launch/share/launch/frontend/
# grammar.lark"). Sourcing just interfaces is enough to resolve the custom
# msg/srv schemas (DriveStatus, DriveMotorFeedback, Drive.srv, ...) without
# pulling in the collision.
INTERFACES_SETUP="$ROOT_DIR/ws/install/interfaces/share/interfaces/local_setup.bash"
if [[ -f "$INTERFACES_SETUP" ]]; then
  source "$INTERFACES_SETUP"
else
  echo "WARNING: $INTERFACES_SETUP not found — build the workspace first."
  echo "         Custom interfaces/ message and service schemas won't resolve without it."
fi

echo "Starting Foxglove Bridge on ws://0.0.0.0:$PORT"
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$PORT"
