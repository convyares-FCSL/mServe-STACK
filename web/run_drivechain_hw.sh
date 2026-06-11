#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_drivechain_hw.sh
#
# Starts the full drive stack (mserve_drivechain + mserve_base) — works both
# natively (if ROS is installed) and inside the mserve Docker container
# (Raspberry Pi / Debian).
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
    # Each pkill must be its own `exec` — chaining them in one `bash -lc "a; b; c"`
    # lets `pkill -f a` match the wrapper shell itself (its cmdline contains
    # "a; b; c"), killing it before b/c ever run.
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f drivechain_node     2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f base_node           2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f rosbridge_websocket 2>/dev/null || true
  else
    # `ros2 run` does not always exec-replace itself, so $! may only be the
    # wrapper PID — pkill the actual node binaries by name as well.
    pkill -f drivechain_node     2>/dev/null || true
    pkill -f base_node           2>/dev/null || true
    pkill -f rosbridge_websocket 2>/dev/null || true
    pkill -f "http.server 6240"  2>/dev/null || true
  fi
  # rosbridge can get stuck in rclpy shutdown; force-kill after 2s
  sleep 2
  for pid in "${NATIVE_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  if [[ "$USE_DOCKER" != true ]]; then
    pkill -9 -f drivechain_node     2>/dev/null || true
    pkill -9 -f base_node           2>/dev/null || true
    pkill -9 -f rosbridge_websocket 2>/dev/null || true
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
      --packages-select interfaces utils mserve_drivechain mserve_base \
      --cmake-args -DBUILD_TESTING=OFF \
      --symlink-install 2>&1
  "
  echo "Build complete."
else
  # Native: source workspace
  SETUP="$WS_DIR/install/setup.bash"
  if [[ ! -f "$SETUP" ]]; then
    echo "ERROR: workspace not built — run:"
    echo "       colcon build --packages-select interfaces utils mserve_drivechain mserve_base"
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
  # See note in cleanup() — one exec per pkill so each pattern actually runs.
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f rosbridge_websocket 2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f drivechain_node     2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f base_node           2>/dev/null || true
  sleep 1
  # Force-kill any survivors so the new nodes don't end up with duplicate
  # /mserve_base or /mserve_drivechain registrations.
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f rosbridge_websocket 2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f drivechain_node     2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f base_node           2>/dev/null || true
else
  pkill -f rosbridge_websocket  2>/dev/null || true
  pkill -f drivechain_node      2>/dev/null || true
  pkill -f base_node            2>/dev/null || true
  pkill -f "http.server 6240"   2>/dev/null || true
  sleep 2  # wait for port 9090/6240 to be released before restarting
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
  # rosbridge throws a known rclpy/Tornado traceback on SIGINT shutdown (Jazzy
  # bug) — redirect to a log file so it doesn't spam the terminal on Ctrl+C.
  ros2 run rosbridge_server rosbridge_websocket --port 9090 > /tmp/rosbridge.log 2>&1 &
  NATIVE_PIDS+=($!)
fi
sleep 1

# ── Start drivechain node ─────────────────────────────────────────────────────
if [[ "$SIM_MODE" == true ]]; then
  echo "Starting drivechain node (sim)…"
  NODE_ARGS="--ros-args -p drive.backend:=sim --log-level mserve_drivechain:=debug"
else
  echo "Starting drivechain node (hardware, $UART_DEVICE)…"
  NODE_ARGS="--ros-args -p drive.backend:=hardware -p hardware.uart_device:=$UART_DEVICE --log-level mserve_drivechain:=debug"
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

# ── Start base node ────────────────────────────────────────────────────────────
echo "Starting base node…"
BASE_NODE_ARGS="--ros-args --log-level mserve_base:=debug"

if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 run mserve_base base_node $BASE_NODE_ARGS
  "
else
  ros2 run mserve_base base_node $BASE_NODE_ARGS &
  NATIVE_PIDS+=($!)
fi

# ── Wait for a lifecycle node to appear ──────────────────────────────────────
wait_for_lifecycle_node() {
  local node_name="$1"
  local up=false
  for i in $(seq 1 30); do
    if [[ "$USE_DOCKER" == true ]]; then
      CHECK=$(docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /ws/install/setup.bash
        ros2 lifecycle get $node_name 2>/dev/null
      " 2>/dev/null || true)
    else
      CHECK=$(ros2 lifecycle get "$node_name" 2>/dev/null || true)
    fi
    if [[ "$CHECK" == *"unconfigured"* ]]; then
      up=true
      break
    fi
    echo "  ($i/30) $node_name not ready yet…"
    sleep 1
  done
  if [[ "$up" == false ]]; then
    echo "ERROR: $node_name did not appear after 30 s. Check logs."
    exit 1
  fi
}

echo "Waiting for drivechain node…"
wait_for_lifecycle_node /mserve_drivechain

echo "Waiting for base node…"
wait_for_lifecycle_node /mserve_base

# ── Lifecycle ─────────────────────────────────────────────────────────────────
echo "Configuring drivechain + base nodes…"
if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 lifecycle set /mserve_drivechain configure
    ros2 lifecycle set /mserve_drivechain activate
    ros2 lifecycle set /mserve_base configure
    ros2 lifecycle set /mserve_base activate
  "
else
  ros2 lifecycle set /mserve_drivechain configure
  ros2 lifecycle set /mserve_drivechain activate
  ros2 lifecycle set /mserve_base configure
  ros2 lifecycle set /mserve_base activate
fi

# ── Web server (always native — Python is always available) ───────────────────
echo "Starting web server on http://localhost:6240…"
cd "$SCRIPT_DIR"
python3 -m http.server 6240 &
NATIVE_PIDS+=($!)

# ── Print URL ─────────────────────────────────────────────────────────────────
LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")
BACKEND_LABEL=$([ "$SIM_MODE" == true ] && echo "sim" || echo "hardware ($UART_DEVICE)")
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Drivechain + Base ready  [$BACKEND_LABEL]"
echo ""
echo "  Open in browser:"
echo "    http://${LOCAL_IP}:6240/drivechain.html"
echo "    http://${LOCAL_IP}:6240/base.html"
echo ""
echo "  rosbridge log: /tmp/rosbridge.log"
echo ""
echo "  Press Ctrl+C to stop everything."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

wait
