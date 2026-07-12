#!/bin/bash
# Launches RViz pointed at a Zenoh router running on the mServe Pi, instead of
# relying on DDS multicast discovery (which doesn't reliably cross Tailscale
# or multi-interface boxes like Thor). Companion script — run on the Pi
# first, leave it running: scripts/remote/start_zenoh_router.sh
#
# Usage:
#   ./scripts/remote/launch_remote_rviz_zenoh.sh [pi_ip] [rviz_config]
#
# Examples:
#   ./scripts/remote/launch_remote_rviz_zenoh.sh                        # LAN default below
#   ./scripts/remote/launch_remote_rviz_zenoh.sh 172.16.68.73           # explicit LAN IP
#   ./scripts/remote/launch_remote_rviz_zenoh.sh 100.122.150.74         # Tailscale IP (off-LAN)

set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

PI_IP=${1:-172.16.68.73}
RVIZ_CONFIG=${2:-$ROOT_DIR/ws/src/mserve_description/rviz/mserve.rviz}

echo ""
echo "mServe Remote RViz (Zenoh)"
echo ""
echo "  Pi router:   tcp/$PI_IP:7447"
echo "  RViz config: $RVIZ_CONFIG"
echo ""

# Stale env from a previous Fast-DDS discovery-server attempt would otherwise
# silently take precedence / conflict with the Zenoh config below.
unset ROS_DISCOVERY_SERVER

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$PI_IP:7447\"]"

echo "ROS 2 Network Configuration:"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"
echo ""

# Distro-agnostic: try common distro names in order rather than hardcoding one.
SOURCED=false
for distro in jazzy lyrical humble iron rolling; do
  if [ -f "/opt/ros/$distro/setup.bash" ]; then
    source "/opt/ros/$distro/setup.bash"
    SOURCED=true
    break
  fi
done
if [ "$SOURCED" != true ]; then
  echo "Error: Could not find a ROS 2 setup.bash under /opt/ros/*."
  exit 1
fi

if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
  echo "Error: rmw_zenoh_cpp is not installed."
  echo "Install it with (package name depends on your distro):"
  echo "  sudo apt install ros-\$ROS_DISTRO-rmw-zenoh-cpp"
  exit 1
fi

# The daemon caches whatever discovery config was active when it last
# started — switching RMW/Zenoh config requires a restart before anything
# will actually see the new graph.
echo "Restarting ros2 daemon to pick up the new discovery config..."
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "Verifying connection to the Pi..."
sleep 2
if ros2 node list 2>/dev/null | grep -q mserve; then
  echo "Found mserve nodes:"
  ros2 node list | sed 's/^/  /'
else
  echo "Warning: no mserve_* nodes visible yet."
  echo "  - Is scripts/remote/start_zenoh_router.sh running on the Pi?"
  echo "  - Is the robot stack (./scripts/run_stack.sh) running on the Pi?"
  echo "  - Continuing anyway — the graph may still be settling."
fi
echo ""

if [ -f "$RVIZ_CONFIG" ]; then
  echo "Launching RViz with config: $RVIZ_CONFIG"
  rviz2 -d "$RVIZ_CONFIG"
else
  echo "Launching RViz (config not found at $RVIZ_CONFIG, using defaults)"
  rviz2
fi
