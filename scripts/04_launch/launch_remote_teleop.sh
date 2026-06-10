#!/bin/bash
# Quick launcher for keyboard teleop from a remote laptop / WSL session.
# Assumes a Fast DDS discovery server is already running on Thor.
#
# Usage:
#   ./scripts/launch_remote_teleop.sh [thor_ip]

THOR_IP=${1:-100.110.87.8}
DISCOVERY_PORT=${ROS_DISCOVERY_PORT:-11888}

echo ""
echo "mServe Remote Teleop"
echo ""
echo "  Thor IP: $THOR_IP"
echo ""

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-$THOR_IP:$DISCOVERY_PORT}

echo "ROS 2 Network Configuration:"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY=0"
echo "  ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo ""

source /opt/ros/jazzy/setup.bash 2>/dev/null || {
    echo "Error: Could not source ROS 2 setup."
    echo "Run: source /opt/ros/jazzy/setup.bash"
    exit 1
}

if ! ros2 pkg prefix teleop_twist_keyboard >/dev/null 2>&1; then
    echo "Error: teleop_twist_keyboard is not installed."
    echo "Install it with:"
    echo "  sudo apt install ros-jazzy-teleop-twist-keyboard"
    exit 1
fi

echo "Starting keyboard teleop. Keep focus in this terminal."
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
