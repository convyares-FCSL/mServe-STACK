#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
COMMAND=${1:-both}

cd "$ROOT_DIR"

docker compose up -d robot-mserve

start_rosbridge() {
  echo "Starting rosbridge_server inside Docker on ws://localhost:9090"
  docker compose exec -d robot-mserve bash -lc \
    "source /opt/ros/jazzy/setup.bash && apt-get update >/dev/null && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-jazzy-rosbridge-server >/dev/null && ros2 run rosbridge_server rosbridge_websocket --port 9090"
}

start_web() {
  echo "Starting web UI server on http://localhost:8080"
  docker compose exec -d robot-mserve bash -c "cd /web && python3 -m http.server 8080 >> /tmp/mserve_web.log 2>&1"
  echo "Web server started inside Docker (serving /web on port 8080)"
}

case "$COMMAND" in
  rosbridge)
    start_rosbridge
    ;;
  web)
    start_web
    ;;
  both)
    start_rosbridge
    start_web
    ;;
  *)
    echo "Usage: $0 {rosbridge|web|both}"
    exit 1
    ;;
esac
