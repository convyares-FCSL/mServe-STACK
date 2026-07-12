#!/bin/bash
# Starts a Zenoh router on the Pi so a remote RViz (e.g. on Thor) can see this
# machine's ROS graph without relying on DDS multicast discovery, which
# doesn't reliably cross Tailscale/multi-interface boxes. Companion script:
# scripts/remote/launch_remote_rviz_zenoh.sh, run on the *remote* machine.
#
# Run this once, leave it running, then start the robot stack as normal
# (./scripts/run_stack.sh) — the router and the robot nodes are
# independent processes; either can be (re)started without touching the
# other. Ctrl+C stops the router.
#
# Usage:
#   ./scripts/remote/start_zenoh_router.sh

set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)

source /opt/ros/lyrical/setup.bash 2>/dev/null || {
  echo "Error: Could not source ROS 2 setup (expected /opt/ros/lyrical)."
  exit 1
}

if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
  echo "Error: rmw_zenoh_cpp is not installed."
  echo "Install it with:"
  echo "  sudo apt install ros-lyrical-rmw-zenoh-cpp"
  exit 1
fi

echo ""
echo "mServe Zenoh Router"
echo ""
echo "  Reachable at (once started, see below for the exact addresses):"
echo "    tcp/$(hostname -I 2>/dev/null | awk '{print $1}'):7447   (LAN)"
command -v tailscale >/dev/null 2>&1 && echo "    tcp/$(tailscale ip -4 2>/dev/null):7447   (Tailscale)"
echo ""
echo "On the remote machine, point at whichever address it can actually"
echo "reach and run:"
echo "  $ROOT_DIR/scripts/remote/launch_remote_rviz_zenoh.sh <this-pi's-ip>"
echo ""
echo "Press Ctrl+C to stop the router."
echo ""

exec ros2 run rmw_zenoh_cpp rmw_zenohd
