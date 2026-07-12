#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WS_SRC="$ROOT_DIR/ws/src"

# ---------------------------------------------------------------------------
# apt dependencies
# ---------------------------------------------------------------------------

echo "[deps_setup] Installing apt dependencies..."

sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ros-jazzy-behaviortree-cpp \
  ros-jazzy-generate-parameter-library

echo "[deps_setup] apt dependencies installed."

# ---------------------------------------------------------------------------
# behaviortree_ros2 — source build (not in apt for Jazzy)
# ---------------------------------------------------------------------------

BT_ROS2_DIR="$WS_SRC/BehaviorTree.ROS2"
BT_ROS2_REPO="https://github.com/BehaviorTree/BehaviorTree.ROS2.git"

if [ -d "$BT_ROS2_DIR" ]; then
  echo "[deps_setup] BehaviorTree.ROS2 already cloned at $BT_ROS2_DIR — skipping."
else
  echo "[deps_setup] Cloning BehaviorTree.ROS2..."
  git clone "$BT_ROS2_REPO" "$BT_ROS2_DIR"
  echo "[deps_setup] Cloned."
fi

# ---------------------------------------------------------------------------
# Build source dependencies in order
# ---------------------------------------------------------------------------

echo "[deps_setup] Building btcpp_ros2_interfaces..."
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd "$ROOT_DIR/ws"
colcon build --packages-select btcpp_ros2_interfaces

echo "[deps_setup] Building behaviortree_ros2..."
colcon build --packages-select behaviortree_ros2

echo "[deps_setup] All dependencies ready."
echo ""
echo "Next: build the workspace with:"
echo "  bash scripts/02_bootstrap/build_workspace.sh"
