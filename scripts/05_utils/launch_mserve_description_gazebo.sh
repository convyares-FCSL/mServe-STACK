#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

set +u
source "$ROOT_DIR/scripts/01_setup/env_setup.sh" >/dev/null
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

if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  echo "ERROR: ros_gz_sim is not installed."
  echo "Install it with:"
  echo "  sudo apt install ros-jazzy-ros-gz"
  exit 1
fi

if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo "ERROR: ros_gz_bridge is not installed."
  echo "Install it with:"
  echo "  sudo apt install ros-jazzy-ros-gz"
  exit 1
fi

echo "[mserve_description] building host Gazebo workspace"
colcon --log-base "$LOG_BASE" build \
  --build-base "$BUILD_BASE" \
  --install-base "$INSTALL_BASE" \
  --symlink-install \
  --packages-select mserve_description

set +u
source "$INSTALL_BASE/setup.bash"
set -u

echo "[mserve_description] launching Gazebo description view"
ros2 launch mserve_description mserve_gazebo.launch.py "$@"
