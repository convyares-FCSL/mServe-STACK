#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_drivechain_hw.sh
#
# Starts the full drivechain stack — works both natively (if ROS is installed)
# and inside the mserve Docker container (Raspberry Pi / Debian).
#
# Usage:
#   ./web/run_drivechain_hw.sh [--sim] [uart_device]
#
# Examples:
#   ./web/run_drivechain_hw.sh --sim             # sim, no hardware needed
#   ./web/run_drivechain_hw.sh                   # hardware, /dev/ttyAMA0 (Pi 5 GPIO UART)
#   ./web/run_drivechain_hw.sh /dev/ttyACM0      # hardware, custom device (e.g. USB)
# ─────────────────────────────────────────────────────────────────────────────
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
WS_DIR="$ROOT_DIR/ws"

# ── Parse arguments ───────────────────────────────────────────────────────────
SIM_MODE=false
if [[ "${1:-}" == "--sim" ]]; then
  SIM_MODE=true
  shift
fi
UART_DEVICE="${1:-/dev/ttyAMA0}"

# ── Detect native vs Docker ───────────────────────────────────────────────────
if command -v ros2 >/dev/null 2>&1; then
  USE_DOCKER=false
  echo "ROS 2 found — running natively"
else
  USE_DOCKER=true
  echo "ROS 2 not found — using Docker container"
  if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not found. Install Docker or ROS 2 Jazzy."
    exit 1
  fi
fi

# ── Helpers ───────────────────────────────────────────────────────────────────
ros_exec() {
  # Run a ROS command either natively or inside the container
  if [[ "$USE_DOCKER" == true ]]; then
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc "$*"
  else
    bash -lc "$*"
  fi
}

ros_exec_bg() {
  # Run a ROS command in the background (container or native)
  if [[ "$USE_DOCKER" == true ]]; then
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "$*"
  else
    bash -lc "$*" &
    echo $!
  fi
}

# ── Cleanup on exit ───────────────────────────────────────────────────────────
NATIVE_PIDS=()
cleanup() {
  echo ""
  echo "Shutting down…"
  for pid in "${NATIVE_PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  if [[ "$USE_DOCKER" == true ]]; then
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc \
      "pkill -f drivechain_node || true; pkill -f rosbridge_websocket || true" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
  echo "Done."
}
trap cleanup SIGINT SIGTERM EXIT

# ── Docker: ensure container is running, build packages ──────────────────────
if [[ "$USE_DOCKER" == true ]]; then
  echo "Starting Docker container…"
  docker compose -f "$ROOT_DIR/docker-compose.yml" up -d robot-mserve
  sleep 2

  echo "Building ROS packages inside container…"
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    cd /ws
    colcon build \
      --packages-select mserve_interfaces mserve_utils mserve_drivechain \
      --cmake-args -DBUILD_TESTING=OFF \
      --symlink-install 2>&1
  "
  echo "Build complete."
else
  # Native: source workspace
  SETUP="$WS_DIR/install/setup.bash"
  if [[ ! -f "$SETUP" ]]; then
    echo "ERROR: workspace not built — run:"
    echo "       colcon build --packages-select mserve_interfaces mserve_utils mserve_drivechain"
    exit 1
  fi
  source "$SETUP"
fi

# ── Check UART (hardware only) ────────────────────────────────────────────────
if [[ "$SIM_MODE" == false ]]; then
  if [[ "$USE_DOCKER" == true ]]; then
    if ! docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve \
        bash -lc "test -e $UART_DEVICE" 2>/dev/null; then
      echo "WARNING: $UART_DEVICE not visible inside container."
      echo "         Check docker-compose.yml devices: and that raspi-config serial is enabled."
    fi
  elif [[ ! -e "$UART_DEVICE" ]]; then
    echo "WARNING: $UART_DEVICE not found. Check raspi-config serial port settings."
    echo "         Use --sim to run without hardware."
  fi
fi

# ── Kill any stale ROS processes from previous run ───────────────────────────
if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc \
    "pkill -f rosbridge_websocket 2>/dev/null; pkill -f drivechain_node 2>/dev/null; sleep 1; true" 2>/dev/null || true
else
  pkill -f rosbridge_websocket 2>/dev/null || true
  pkill -f drivechain_node     2>/dev/null || true
fi

# ── Start rosbridge ───────────────────────────────────────────────────────────
echo "Starting rosbridge on ws://localhost:9090…"
if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 run rosbridge_server rosbridge_websocket --port 9090
  "
else
  ros2 run rosbridge_server rosbridge_websocket --port 9090 &
  NATIVE_PIDS+=($!)
fi
sleep 1

# ── Start drivechain node ─────────────────────────────────────────────────────
if [[ "$SIM_MODE" == true ]]; then
  echo "Starting drivechain node (sim)…"
  NODE_ARGS="--ros-args -p drive.backend:=sim"
else
  echo "Starting drivechain node (hardware, $UART_DEVICE)…"
  NODE_ARGS="--ros-args -p drive.backend:=hardware -p hardware.uart_device:=$UART_DEVICE"
fi

if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 run mserve_drivechain drivechain_node $NODE_ARGS
  "
else
  ros2 run mserve_drivechain drivechain_node $NODE_ARGS &
  NATIVE_PIDS+=($!)
fi

# ── Wait for node to appear ───────────────────────────────────────────────────
echo "Waiting for drivechain node…"
NODE_UP=false
for i in $(seq 1 30); do
  if [[ "$USE_DOCKER" == true ]]; then
    CHECK=$(docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc "
      source /opt/ros/jazzy/setup.bash
      source /ws/install/setup.bash
      ros2 lifecycle get /mserve_drivechain 2>/dev/null
    " 2>/dev/null || true)
  else
    CHECK=$(ros2 lifecycle get /mserve_drivechain 2>/dev/null || true)
  fi
  if [[ "$CHECK" == *"unconfigured"* ]]; then
    NODE_UP=true
    break
  fi
  echo "  ($i/30) not ready yet…"
  sleep 1
done

if [[ "$NODE_UP" == false ]]; then
  echo "ERROR: drivechain node did not appear after 30 s. Check logs."
  exit 1
fi

# ── Lifecycle ─────────────────────────────────────────────────────────────────
echo "Configuring drivechain node…"
if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 lifecycle set /mserve_drivechain configure
    ros2 lifecycle set /mserve_drivechain activate
  "
else
  ros2 lifecycle set /mserve_drivechain configure
  ros2 lifecycle set /mserve_drivechain activate
fi

# ── Web server (always native — Python is always available) ───────────────────
echo "Starting web server on http://localhost:8080…"
cd "$SCRIPT_DIR"
python3 -m http.server 8080 &
NATIVE_PIDS+=($!)

# ── Print URL ─────────────────────────────────────────────────────────────────
LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")
BACKEND_LABEL=$([ "$SIM_MODE" == true ] && echo "sim" || echo "hardware ($UART_DEVICE)")
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Drivechain ready  [$BACKEND_LABEL]"
echo ""
echo "  Open in browser:"
echo "    http://${LOCAL_IP}:8080/drivechain.html"
echo ""
echo "  Press Ctrl+C to stop everything."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

wait
