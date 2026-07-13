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
  ros-lyrical-behaviortree-cpp \
  ros-lyrical-generate-parameter-library \
  libceres-dev libsuitesparse-dev libomp-dev qtbase5-dev \
  ros-lyrical-rviz-common ros-lyrical-rviz-default-plugins ros-lyrical-rviz-ogre-vendor ros-lyrical-rviz-rendering \
  ros-lyrical-bondcpp ros-lyrical-bond ros-lyrical-interactive-markers ros-lyrical-tf2-sensor-msgs \
  ros-lyrical-tf2-geometry-msgs ros-lyrical-message-filters ros-lyrical-pluginlib

echo "[deps_setup] apt dependencies installed."

# ---------------------------------------------------------------------------
# Vendored source builds — see ws/src/third_party/README.md for why these
# specifically aren't apt-installable on this distro.
# ---------------------------------------------------------------------------

THIRD_PARTY_DIR="$WS_SRC/third_party"
mkdir -p "$THIRD_PARTY_DIR"

BT_ROS2_DIR="$THIRD_PARTY_DIR/BehaviorTree.ROS2"
if [ -d "$BT_ROS2_DIR" ]; then
  echo "[deps_setup] BehaviorTree.ROS2 already cloned at $BT_ROS2_DIR — skipping."
else
  echo "[deps_setup] Cloning BehaviorTree.ROS2..."
  git clone --branch humble https://github.com/BehaviorTree/BehaviorTree.ROS2.git "$BT_ROS2_DIR"
  touch "$BT_ROS2_DIR/btcpp_ros2_samples/COLCON_IGNORE"
fi

SLAM_TOOLBOX_DIR="$THIRD_PARTY_DIR/slam_toolbox"
if [ -d "$SLAM_TOOLBOX_DIR" ]; then
  echo "[deps_setup] slam_toolbox already cloned at $SLAM_TOOLBOX_DIR — skipping."
else
  echo "[deps_setup] Cloning slam_toolbox..."
  git clone --branch ros2 --depth 1 https://github.com/SteveMacenski/slam_toolbox.git "$SLAM_TOOLBOX_DIR"
fi

# ---------------------------------------------------------------------------
# Build source dependencies in order
# ---------------------------------------------------------------------------

source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
cd "$ROOT_DIR/ws"

echo "[deps_setup] Building btcpp_ros2_interfaces + behaviortree_ros2..."
colcon build --packages-select btcpp_ros2_interfaces behaviortree_ros2 --cmake-args -DBUILD_TESTING=OFF --symlink-install

echo "[deps_setup] Building slam_toolbox..."
colcon build --packages-select slam_toolbox --cmake-args -DBUILD_TESTING=OFF --symlink-install

echo "[deps_setup] All dependencies ready."
echo ""
echo "Next: build the workspace with:"
echo "  bash scripts/build/build_workspace.sh"
