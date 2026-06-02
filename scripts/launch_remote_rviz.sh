#!/bin/bash
# Quick launcher script for remote RViz connection to NVIDIA Thor
# Assumes a Fast DDS discovery server is already running on Thor.
# 
# Usage: 
#   ./launch_remote_rviz.sh [thor_ip] [config_file]
#
# Examples:
#   ./launch_remote_rviz.sh 100.110.87.8              # Tailscale IP
#   ./launch_remote_rviz.sh 192.168.1.100             # Local network IP
#   ./launch_remote_rviz.sh 100.110.87.8 mserve.rviz  # With config file

THOR_IP=${1:-100.110.87.8}
RVIZ_CONFIG=${2:-mserve_remote.rviz}
DISCOVERY_PORT=${ROS_DISCOVERY_PORT:-11888}

echo ""
echo "╔═════════════════════════════════════════════════════════╗"
echo "║          mServe Remote RViz Launcher                    ║"
echo "╚═════════════════════════════════════════════════════════╝"
echo ""
echo "  Thor IP:        $THOR_IP"
echo "  RViz Config:    $RVIZ_CONFIG"
echo ""

# Set ROS 2 environment for distributed networking
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-$THOR_IP:$DISCOVERY_PORT}

echo "ROS 2 Network Configuration:"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY=0"
echo "  ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
echo ""
echo "This requires a Fast DDS discovery server running on Thor."
echo ""

# Source ROS 2
source /opt/ros/jazzy/setup.bash 2>/dev/null || {
    echo "❌ Error: Could not source ROS 2 setup"
    echo "   Run: source /opt/ros/jazzy/setup.bash"
    exit 1
}

# Wait for network to stabilize
echo "Initializing ROS 2 network discovery..."
sleep 2

ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

# Test connectivity
echo "Verifying connection to Thor..."
if ros2 node list &>/dev/null; then
    echo "✓ ROS 2 network active"
    echo ""
    echo "Available nodes:"
    ros2 node list | sed 's/^/  /'
    echo ""
else
    echo "⚠ Warning: ROS 2 network not responding"
    echo "  (Continuing anyway — network may stabilize)"
    echo ""
fi

# Launch RViz
if [ -f "$RVIZ_CONFIG" ]; then
    echo "Launching RViz with config: $RVIZ_CONFIG"
    echo ""
    rviz2 -d "$RVIZ_CONFIG"
else
    echo "Launching RViz (default empty config)"
    echo "  Tip: Save config as '$RVIZ_CONFIG' for next time"
    echo ""
    rviz2
fi
