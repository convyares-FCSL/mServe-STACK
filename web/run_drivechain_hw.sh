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
  # Signal lifecycle_manager directly (SIGINT) so its shutdown tree can
  # gracefully deactivate mserve_drivechain/mserve_base via change_state
  # calls before anything is force-killed. Relying on `ros2 launch`'s own
  # SIGINT cascade to its children proved unreliable when the signal is sent
  # programmatically from this script's trap rather than an interactive
  # shell, so target lifecycle_manager's process directly instead — it exits
  # on its own once the shutdown tree completes.
  if [[ "$USE_DOCKER" == true ]]; then
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -INT -f lifecycle_manager 2>/dev/null || true
  else
    pkill -INT -f lifecycle_manager 2>/dev/null || true
  fi
  sleep 2

  for pid in "${NATIVE_PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  if [[ "$USE_DOCKER" == true ]]; then
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f rosbridge_websocket 2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f web_video_server    2>/dev/null || true
  else
    pkill -f rosbridge_websocket 2>/dev/null || true
    pkill -f web_video_server    2>/dev/null || true
    pkill -f "http.server 6240"  2>/dev/null || true
  fi

  # Give everything a moment to settle before force-killing survivors.
  sleep 1

  if [[ "$USE_DOCKER" == true ]]; then
    # Each pkill must be its own `exec` — chaining them in one `bash -lc "a; b; c"`
    # lets `pkill -f a` match the wrapper shell itself (its cmdline contains
    # "a; b; c"), killing it before b/c ever run.
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f drivechain_node       2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f base_node             2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f camera_node           2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f robot_state_publisher 2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f lifecycle_manager     2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f rosbridge_websocket   2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f web_video_server      2>/dev/null || true
  else
    pkill -9 -f drivechain_node       2>/dev/null || true
    pkill -9 -f base_node             2>/dev/null || true
    pkill -9 -f camera_node           2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f lifecycle_manager     2>/dev/null || true
    pkill -9 -f rosbridge_websocket   2>/dev/null || true
    pkill -9 -f web_video_server      2>/dev/null || true
  fi
  for pid in "${NATIVE_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
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
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f rosbridge_websocket   2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f web_video_server      2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f drivechain_node       2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f base_node             2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f camera_node           2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f robot_state_publisher 2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f lifecycle_manager     2>/dev/null || true
  sleep 1
  # Force-kill any survivors so the new nodes don't end up with duplicate
  # /mserve_base or /mserve_drivechain registrations.
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f rosbridge_websocket   2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f web_video_server      2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f drivechain_node       2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f base_node             2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f camera_node           2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f robot_state_publisher 2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f lifecycle_manager     2>/dev/null || true
else
  pkill -f rosbridge_websocket   2>/dev/null || true
  pkill -f web_video_server      2>/dev/null || true
  pkill -f drivechain_node       2>/dev/null || true
  pkill -f base_node             2>/dev/null || true
  pkill -f camera_node           2>/dev/null || true
  pkill -f robot_state_publisher 2>/dev/null || true
  pkill -f lifecycle_manager    2>/dev/null || true
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

# ── Start web_video_server ────────────────────────────────────────────────────
# Transcodes camera/image_raw (raw YUYV, not browser-decodable) to MJPEG over
# plain HTTP on port 8080 — camera.html/base.html <img> tags point here.
echo "Starting web_video_server on http://localhost:8080…"
if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 run web_video_server web_video_server
  "
else
  ros2 run web_video_server web_video_server > /tmp/web_video_server.log 2>&1 &
  NATIVE_PIDS+=($!)
fi
sleep 1

# ── Launch drivechain + base + lifecycle_manager ─────────────────────────────
# All node startup and lifecycle configure/activate is owned by
# mserve_min.launch.py + lifecycle_manager — this script only picks the
# backend/uart_device launch args and waits for the result.
BACKEND=$([ "$SIM_MODE" == true ] && echo "sim" || echo "hardware")
echo "Launching drivechain + base + lifecycle_manager (backend=$BACKEND)…"
LAUNCH_ARGS="backend:=$BACKEND uart_device:=$UART_DEVICE"

if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 launch launch mserve_min.launch.py $LAUNCH_ARGS
  "
else
  ros2 launch launch mserve_min.launch.py $LAUNCH_ARGS > /tmp/mserve_launch.log 2>&1 &
  NATIVE_PIDS+=($!)
fi

# ── Wait for both nodes to reach 'active' ────────────────────────────────────
wait_for_active() {
  local node_name="$1"
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
    if [[ "$CHECK" == *"active"* ]]; then
      return 0
    fi
    echo "  ($i/30) $node_name not active yet…"
    sleep 1
  done
  echo "ERROR: $node_name did not reach 'active' after 30 s. Check logs (/tmp/mserve_launch.log)."
  exit 1
}

echo "Waiting for drivechain node to activate…"
wait_for_active /mserve_drivechain

echo "Waiting for base node to activate…"
wait_for_active /mserve_base

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
