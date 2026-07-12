#!/usr/bin/env bash
set -euo pipefail

PATTERNS=(
  "ros2 launch mserve_description mserve_gazebo.launch.py"
  "mserve_description/launch/mserve_gazebo.launch.py"
  "name:=mserve_gz_bridge"
  "executable='parameter_bridge'"
  "name:=robot_state_publisher"
  "share/mserve_description/rviz/mserve_gazebo.rviz"
  "share/mserve_description/rviz/mserve.rviz"
  "gz sim"
  "ruby .*gz sim"
)

FOUND=false

for PATTERN in "${PATTERNS[@]}"; do
  if pgrep -af "$PATTERN" >/dev/null 2>&1; then
    FOUND=true
    echo "[mserve_description] stopping processes matching: $PATTERN"
    pkill -f "$PATTERN" || true
  fi
done

sleep 1

for PATTERN in "${PATTERNS[@]}"; do
  if pgrep -af "$PATTERN" >/dev/null 2>&1; then
    echo "[mserve_description] force stopping remaining processes matching: $PATTERN"
    pkill -9 -f "$PATTERN" || true
  fi
done

if [ "$FOUND" = false ]; then
  echo "[mserve_description] no matching Gazebo description processes found"
else
  echo "[mserve_description] stop complete"
fi
