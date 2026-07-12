#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
cd "$SCRIPT_DIR"

if command -v ros2 >/dev/null 2>&1; then
  echo "Starting rosbridge server on ws://localhost:9090"
  ros2 run rosbridge_server rosbridge_websocket --port 9090
else
  echo "Starting rosbridge server inside Docker on ws://localhost:9090"
  cd "$ROOT_DIR"
  docker compose up -d robot-mserve
  docker compose exec -d robot-mserve bash -lc "source /opt/ros/jazzy/setup.bash && apt-get update >/dev/null && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-rosbridge-server >/dev/null && ros2 run rosbridge_server rosbridge_websocket --port 9090"
  echo "rosbridge started in Docker"
fi
