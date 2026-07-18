#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_stack.sh
#
# Starts the full mServe stack (mserve_drivechain + mserve_base + mserve_camera
# + mserve_lidar + robot_state_publisher, all lifecycle-managed by
# lifecycle_manager) plus
# rosbridge and the debug web UI — works both natively (if ROS is installed)
# and inside the mserve Docker container (Raspberry Pi / Debian).
#
# Usage:
#   ./scripts/run_stack.sh [--sim] [--foxglove] [--slam-map|--slam-local] [uart_device]
#
# Examples:
#   ./scripts/run_stack.sh --sim             # sim, no hardware needed
#   ./scripts/run_stack.sh                   # hardware, /dev/ttyAMA0 (Pi 5 GPIO UART)
#   ./scripts/run_stack.sh /dev/ttyACM0      # hardware, custom device (e.g. USB)
#   ./scripts/run_stack.sh --foxglove        # also start Foxglove Bridge (ws://<pi-ip>:8765)
#   ./scripts/run_stack.sh --slam-map        # also start SLAM Toolbox, building/extending a map
#   ./scripts/run_stack.sh --slam-local      # also start SLAM Toolbox, localizing against a saved map
# ─────────────────────────────────────────────────────────────────────────────
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
WS_DIR="$ROOT_DIR/ws"

# ── Parse arguments ───────────────────────────────────────────────────────────
# Order-independent: --sim, --foxglove, --slam-map/--slam-local can appear in
# any order before the optional positional uart_device.
SIM_MODE=false
FOXGLOVE=false
SLAM=false
SLAM_MODE=""
ARGS=()
for arg in "$@"; do
  case "$arg" in
    --sim) SIM_MODE=true ;;
    --foxglove) FOXGLOVE=true ;;
    --slam-map) SLAM=true; SLAM_MODE="map" ;;
    --slam-local) SLAM=true; SLAM_MODE="local" ;;
    *) ARGS+=("$arg") ;;
  esac
done
UART_DEVICE="${ARGS[0]:-/dev/ttyAMA0}"

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
CLEANING_UP=false
cleanup() {
  # A second SIGINT/SIGTERM arriving mid-cleanup (someone hitting Ctrl+C
  # again while this is already tearing things down — the shutdown sequence
  # below takes a few seconds) re-entered this function via the trap and
  # corrupted bash's own function-call stack (`pop_var_context: head of
  # shell_variables not a function context`), not just this script's state.
  # Disarm the trap and bail out on re-entry instead.
  if [[ "$CLEANING_UP" == true ]]; then
    return
  fi
  CLEANING_UP=true
  trap - SIGINT SIGTERM EXIT

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
    pkill -f rosbridge_websocket    2>/dev/null || true
    pkill -f web_video_server       2>/dev/null || true
    pkill -f foxglove_bridge        2>/dev/null || true
    pkill -f async_slam_toolbox_node 2>/dev/null || true
    pkill -f "http.server 6240"     2>/dev/null || true
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
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f lidar_node            2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f robot_state_publisher 2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f lifecycle_manager     2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f rosbridge_websocket   2>/dev/null || true
    docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f web_video_server      2>/dev/null || true
  else
    pkill -9 -f drivechain_node       2>/dev/null || true
    pkill -9 -f base_node             2>/dev/null || true
    pkill -9 -f camera_node           2>/dev/null || true
    pkill -9 -f lidar_node            2>/dev/null || true
    pkill -9 -f robot_state_publisher 2>/dev/null || true
    pkill -9 -f lifecycle_manager     2>/dev/null || true
    pkill -9 -f rosbridge_websocket   2>/dev/null || true
    pkill -9 -f web_video_server      2>/dev/null || true
    pkill -9 -f foxglove_bridge       2>/dev/null || true
    pkill -9 -f async_slam_toolbox_node 2>/dev/null || true
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
      --packages-select interfaces utils mserve_drivechain mserve_base launch mserve_description \
        lifecycle_manager btcpp_ros2_interfaces behaviortree_ros2 mserve_camera mserve_lidar \
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

# ── Zenoh: join a local router if one is already running ────────────────────
# scripts/remote/start_zenoh_router.sh starts rmw_zenohd independently of this
# script. Without this, the nodes below launch on the default RMW (plain DDS)
# and never connect to that router at all — it ends up with zero peers, so a
# remote RViz pointed at it (scripts/remote/launch_remote_rviz_zenoh.sh) sees
# nothing (no TF, no topics), even though the router itself is up. Joining is
# automatic here: rmw_zenoh_cpp's default local scouting finds a router on the
# same host with no extra config.
#
# mode=client: the default session mode is "peer", which gossips with other
# peers behind the router and then tries to connect to them directly using
# whatever locator they advertised — that can be a loopback address, which is
# fine locally but breaks the moment a remote machine (Thor) is one of the
# gossiped peers. "client" mode never gossips or attempts direct peer links;
# every node here talks only through the router, and so does the remote side
# (scripts/remote/launch_remote_rviz_zenoh.sh sets the same mode).
if [[ "$USE_DOCKER" == false ]] && pgrep -f rmw_zenohd >/dev/null 2>&1; then
  echo "Zenoh router detected — joining it (RMW_IMPLEMENTATION=rmw_zenoh_cpp, mode=client)"
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\""
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
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
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -f lidar_node            2>/dev/null || true
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
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f lidar_node            2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f robot_state_publisher 2>/dev/null || true
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -T robot-mserve pkill -9 -f lifecycle_manager     2>/dev/null || true
else
  pkill -f rosbridge_websocket    2>/dev/null || true
  pkill -f web_video_server       2>/dev/null || true
  pkill -f foxglove_bridge        2>/dev/null || true
  pkill -f async_slam_toolbox_node 2>/dev/null || true
  pkill -f drivechain_node       2>/dev/null || true
  pkill -f base_node             2>/dev/null || true
  pkill -f camera_node           2>/dev/null || true
  pkill -f lidar_node            2>/dev/null || true
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
    ros2 run web_video_server web_video_server > /tmp/web_video_server.log 2>&1
  "
else
  ros2 run web_video_server web_video_server > /tmp/web_video_server.log 2>&1 &
  NATIVE_PIDS+=($!)
fi
sleep 1

# ── Start Foxglove Bridge (opt-in, --foxglove) ────────────────────────────────
# Runs the node directly via `ros2 run`, not `ros2 launch ...xml`: this repo
# has a package literally named "launch" that shadows the real ROS 2 launch
# framework package once the workspace is sourced (already done above by this
# point), which breaks the XML launch frontend specifically — see
# scripts/run_foxglove_bridge.sh's comment. `ros2 run` doesn't need that
# frontend at all, so it sidesteps the collision rather than working around
# it with extra sourcing gymnastics.
if [[ "$FOXGLOVE" == true ]]; then
  if [[ "$USE_DOCKER" == true ]]; then
    echo "NOTE: --foxglove is native-only (not wired into the Docker path, same as camera/lidar) — skipping."
  else
    echo "Starting Foxglove Bridge on ws://0.0.0.0:8765…"
    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 > /tmp/foxglove_bridge.log 2>&1 &
    NATIVE_PIDS+=($!)
    sleep 1
  fi
fi

# ── Launch drivechain + base + lifecycle_manager ─────────────────────────────
# All node startup and lifecycle configure/activate is owned by
# mserve_min.launch.py + lifecycle_manager — this script only picks the
# backend/uart_device launch args and waits for the result.
BACKEND=$([ "$SIM_MODE" == true ] && echo "sim" || echo "hardware")
echo "Launching drivechain + base + lifecycle_manager (backend=$BACKEND)…"
LAUNCH_ARGS="backend:=$BACKEND uart_device:=$UART_DEVICE"
# with_camera/with_lidar default true (mserve_min.launch.py) in both native
# and Docker mode now that the Docker image has the driver deps — see
# docker-compose.yml's /dev/video0 and /dev/ttyUSB0 device passthrough.

if [[ "$USE_DOCKER" == true ]]; then
  docker compose -f "$ROOT_DIR/docker-compose.yml" exec -d robot-mserve bash -lc "
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 launch launch mserve_min.launch.py $LAUNCH_ARGS > /tmp/mserve_launch.log 2>&1
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
  if [[ "$USE_DOCKER" == true ]]; then
    echo "ERROR: $node_name did not reach 'active' after 30 s. Check logs (docker compose exec robot-mserve cat /tmp/mserve_launch.log)."
  else
    echo "ERROR: $node_name did not reach 'active' after 30 s. Check logs (/tmp/mserve_launch.log)."
  fi
  exit 1
}

echo "Waiting for drivechain node to activate…"
wait_for_active /mserve_drivechain

echo "Waiting for base node to activate…"
wait_for_active /mserve_base

# ── Start SLAM Toolbox (opt-in, --slam-map / --slam-local) ────────────────────
# Placed after drivechain/base are confirmed active so /scan (mserve_lidar)
# and odom -> base_link TF (mserve_base) are already flowing by the time it
# starts — not strictly required (it just waits quietly otherwise, same as
# every other node here catching up to its dependencies), but avoids the
# handful of seconds of "why is /map empty" that starting it first would give.
# Plain `ros2 launch launch mserve_slam.launch.py` (.py, not .xml) — doesn't
# hit the launch-package-name collision that foxglove_bridge's XML file does.
if [[ "$SLAM" == true ]]; then
  if [[ "$USE_DOCKER" == true ]]; then
    echo "NOTE: --slam-$SLAM_MODE is native-only (not wired into the Docker path, same as camera/lidar) — skipping."
  else
    if [[ "$SLAM_MODE" == "map" ]]; then
      echo "Starting SLAM Toolbox (mapping) — /map will start publishing once configure/activate completes…"
    else
      echo "Starting SLAM Toolbox (localization) — needs a real map_file_name already set in slam_params_local.yaml…"
    fi
    ros2 launch launch mserve_slam.launch.py mode:="$SLAM_MODE" > /tmp/mserve_slam.log 2>&1 &
    NATIVE_PIDS+=($!)
  fi
fi

# ── Web server (always native — Python is always available) ───────────────────
echo "Starting web server on http://localhost:6240…"
cd "$ROOT_DIR/web"
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
if [[ "$FOXGLOVE" == true ]]; then
  echo "  Foxglove: ws://${LOCAL_IP}:8765  (Open Connection -> Foxglove WebSocket)"
  echo ""
fi
if [[ "$SLAM" == true && "$SLAM_MODE" == "map" ]]; then
  echo "  SLAM Toolbox running (mapping) — /map building live."
  echo "  Save for viewing (a .pgm + .yaml):"
  echo "    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: 'my_map'}}\""
  echo "  Save for --slam-local (the actual pose-graph it needs):"
  echo "    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '/absolute/path/my_map'}\""
  echo ""
elif [[ "$SLAM" == true && "$SLAM_MODE" == "local" ]]; then
  echo "  SLAM Toolbox running (localization) against the map set in slam_params_local.yaml."
  echo ""
fi
echo "  rosbridge log: /tmp/rosbridge.log"
echo ""
echo "  Press Ctrl+C to stop everything."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

wait
