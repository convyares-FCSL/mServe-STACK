#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

set +u
source "$ROOT_DIR/scripts/01_setup/env_setup.sh" >/dev/null
set -u

SHOW_HELP=false
USER_WANTS_HEADLESS=false
LAUNCH_ARGS=()

for ARG in "$@"; do
  case "$ARG" in
    --help|-h)
      SHOW_HELP=true
      ;;
    --headless)
      USER_WANTS_HEADLESS=true
      ;;
    *)
      LAUNCH_ARGS+=("$ARG")
      ;;
  esac
done

if [ "$SHOW_HELP" = true ]; then
  cat <<'EOF'
Usage:
  launch_mserve_description_gazebo.sh [--headless] [ros2 launch args...]

Options:
  --headless   Run Gazebo server-only with headless rendering enabled.

Examples:
  scripts/05_utils/launch_mserve_description_gazebo.sh
  scripts/05_utils/launch_mserve_description_gazebo.sh --headless
  scripts/05_utils/launch_mserve_description_gazebo.sh world:=empty.sdf
EOF
  exit 0
fi

cd "$ROOT_DIR/ws"

BUILD_BASE="$ROOT_DIR/ws/build_host"
INSTALL_BASE="$ROOT_DIR/ws/install_host"
LOG_BASE="$ROOT_DIR/ws/log_host"
ROS_LOG_DIR="$LOG_BASE/ros"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

# Keep Gazebo Transport local by default. On hosts with multiple network
# interfaces, automatic IP selection can prevent the GUI and spawn client from
# discovering the server, which looks like Gazebo has frozen.
export GZ_IP="${GZ_IP:-127.0.0.1}"

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

if [ "$USER_WANTS_HEADLESS" = true ]; then
  LAUNCH_ARGS=("headless:=true" "${LAUNCH_ARGS[@]}")
fi

HAS_GZ_ARGS=false
for ARG in "${LAUNCH_ARGS[@]}"; do
  if [[ "$ARG" == gz_args:=* ]]; then
    HAS_GZ_ARGS=true
    break
  fi
done

if [ "$HAS_GZ_ARGS" = false ]; then
  if [ -n "${MSERVE_GZ_ARGS:-}" ]; then
    LAUNCH_ARGS=("gz_args:=$MSERVE_GZ_ARGS" "${LAUNCH_ARGS[@]}")
  elif [ "$(uname -m)" = "aarch64" ]; then
    # OGRE2 can render Gazebo primitives black or incomplete on some Jetson
    # EGL stacks. Prefer OGRE on ARM unless the caller chooses another renderer.
    LAUNCH_ARGS=("gz_args:=--render-engine ogre --render-engine-gui ogre" "${LAUNCH_ARGS[@]}")
  fi
fi

echo "[mserve_description] launching Gazebo description view"
ros2 launch mserve_description mserve_gazebo.launch.py "${LAUNCH_ARGS[@]}"
