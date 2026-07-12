#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

set +u
source "$ROOT_DIR/scripts/setup/env_setup.sh" >/dev/null
set -u

cd "$ROOT_DIR/ws"

BUILD_BASE="$ROOT_DIR/ws/build_host"
INSTALL_BASE="$ROOT_DIR/ws/install_host"
LOG_BASE="$ROOT_DIR/ws/log_host"
ROS_LOG_DIR="$LOG_BASE/ros"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

if ! command -v xacro >/dev/null 2>&1; then
  echo "ERROR: xacro is not installed or not on PATH."
  echo "Install it with:"
  echo "  sudo apt install ros-jazzy-xacro"
  exit 1
fi

if ! ros2 pkg prefix joint_state_publisher_gui >/dev/null 2>&1; then
  echo "ERROR: joint_state_publisher_gui is not installed."
  echo "Install it with:"
  echo "  sudo apt install ros-jazzy-joint-state-publisher-gui"
  exit 1
fi

if ! ros2 pkg prefix rviz2 >/dev/null 2>&1; then
  echo "ERROR: rviz2 is not installed."
  exit 1
fi

echo "[mserve_description] building host RViz workspace"
colcon --log-base "$LOG_BASE" build \
  --build-base "$BUILD_BASE" \
  --install-base "$INSTALL_BASE" \
  --symlink-install \
  --packages-select mserve_description

set +u
source "$INSTALL_BASE/setup.bash"
set -u

echo "[mserve_description] launching RViz description view"
ros2 launch mserve_description mserve_rviz.launch.py "$@"
