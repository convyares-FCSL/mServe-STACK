#!/usr/bin/env bash
set -e
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

export ROS_DISTRO="${ROS_DISTRO:-lyrical}"

ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  source "$ROS_SETUP"
else
  echo "ERROR: ROS setup not found: $ROS_SETUP"
  if [ -d "/opt/ros" ]; then
    echo "Available ROS distros in /opt/ros:" 
    ls -1 /opt/ros
  else
    echo "No /opt/ros directory found. ROS does not appear to be installed on this machine."
  fi
  echo "Please install ROS or set ROS_DISTRO to an installed distribution before retrying."
  return 1 2>/dev/null || exit 1
fi

export MSERVE_STACK_ROOT="$ROOT_DIR"
export ROS_WS="$ROOT_DIR/ws"

command -v ros2 >/dev/null 2>&1 || {
  echo "ERROR: ros2 command is not available after sourcing $ROS_SETUP."
  echo "Make sure ROS is installed correctly and that your shell can access ros2."
  return 1 2>/dev/null || exit 1
}

command -v colcon >/dev/null 2>&1 || {
  echo "ERROR: colcon command is not available."
  echo "Install colcon_common_extensions (apt package: python3-colcon-common-extensions or pip install colcon-common-extensions)."
  return 1 2>/dev/null || exit 1
}

export MSERVE_STACK_ROOT="$ROOT_DIR"
export ROS_WS="$ROOT_DIR/ws"

printf "Environment configured for mServe-STACK at %s\n" "$ROOT_DIR"
