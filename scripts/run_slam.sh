#!/usr/bin/env bash
# Standalone SLAM Toolbox starter — for building a map while driving the
# robot around (scripts/run_stack.sh or the systemd service must already be
# running, for /scan + odom -> base_link TF). Kept separate from
# run_stack.sh: mapping is an occasional, opt-in session, not something you
# want starting on every boot alongside the always-on drive stack.
#
# Usage:
#   ./scripts/run_slam.sh
#
# Once running: drive the robot around (web UI, Foxglove teleop, keyboard —
# whatever you're already using) and watch /map fill in. To save the result:
#   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
set -eo pipefail
# Deliberately not `set -u` — see scripts/run_foxglove_bridge.sh's comment;
# /opt/ros/lyrical/setup.bash isn't nounset-safe.
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)

if ! command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/lyrical/setup.bash
fi

WS_SETUP="$ROOT_DIR/ws/install/setup.bash"
if [[ ! -f "$WS_SETUP" ]]; then
  echo "ERROR: workspace not built — run:"
  echo "       colcon build --packages-select interfaces utils launch slam_toolbox --cmake-args -DBUILD_TESTING=OFF --symlink-install"
  exit 1
fi
source "$WS_SETUP"

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "ERROR: slam_toolbox not built — see ws/src/third_party/README.md, or:"
  echo "       bash scripts/setup/deps_setup.sh"
  exit 1
fi

echo "Starting SLAM Toolbox — /map will start publishing once configure/activate completes."
exec ros2 launch launch mserve_slam.launch.py
