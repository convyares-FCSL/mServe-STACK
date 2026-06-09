#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_drivechain_hw.sh
#
# Starts the full drivechain stack on real hardware:
#   1. drivechain node (hardware backend, /dev/serial0)
#   2. rosbridge websocket on port 9090
#   3. static web server on port 8080
#
# Run this on the Raspberry Pi (or any host with the workspace built and
# the DDSM Driver HAT accessible at /dev/serial0).
#
# Usage:
#   ./run_drivechain_hw.sh [--sim] [uart_device]
#
# Examples:
#   ./run_drivechain_hw.sh                   # hardware, defaults to /dev/serial0
#   ./run_drivechain_hw.sh /dev/ttyAMA0      # hardware, custom device
#   ./run_drivechain_hw.sh --sim             # sim mode, no UART needed
# ─────────────────────────────────────────────────────────────────────────────
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "$SCRIPT_DIR/../ws" && pwd)

# Parse --sim flag
SIM_MODE=false
if [[ "${1:-}" == "--sim" ]]; then
  SIM_MODE=true
  shift
fi

UART_DEVICE="${1:-/dev/serial0}"

# ── Source workspace ──────────────────────────────────────────────────────────
SETUP="$WS_DIR/install/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "ERROR: workspace not built — $SETUP not found"
  echo "       Run: colcon build --packages-select mserve_interfaces mserve_drivechain"
  exit 1
fi
# shellcheck disable=SC1090
source "$SETUP"

# ── Check UART device (hardware only) ────────────────────────────────────────
if [[ "$SIM_MODE" == false && ! -e "$UART_DEVICE" ]]; then
  echo "WARNING: UART device $UART_DEVICE not found."
  echo "         Check raspi-config → serial port is enabled and login shell is disabled."
  echo "         Use --sim to run without hardware."
fi

# ── Cleanup on exit ───────────────────────────────────────────────────────────
PIDS=()
cleanup() {
  echo ""
  echo "Shutting down…"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "Done."
}
trap cleanup SIGINT SIGTERM EXIT

# ── Start rosbridge ───────────────────────────────────────────────────────────
echo "Starting rosbridge on ws://localhost:9090 …"
ros2 run rosbridge_server rosbridge_websocket --port 9090 &
PIDS+=($!)
sleep 1

# ── Start drivechain node ─────────────────────────────────────────────────────
if [[ "$SIM_MODE" == true ]]; then
  echo "Starting drivechain node (sim) …"
  ros2 run mserve_drivechain drivechain_node \
    --ros-args -p drive.backend:=sim &
else
  echo "Starting drivechain node (hardware, $UART_DEVICE) …"
  ros2 run mserve_drivechain drivechain_node \
    --ros-args \
    -p drive.backend:=hardware \
    -p hardware.uart_device:="$UART_DEVICE" &
fi
PIDS+=($!)
sleep 1

# ── Lifecycle: configure + activate ──────────────────────────────────────────
echo "Configuring drivechain node …"
ros2 lifecycle set /mserve_drivechain configure
echo "Activating drivechain node …"
ros2 lifecycle set /mserve_drivechain activate

# ── Start web server ──────────────────────────────────────────────────────────
echo "Starting web server on http://localhost:8080 …"
cd "$SCRIPT_DIR"
python3 -m http.server 8080 &
PIDS+=($!)

# ── Print access URL ──────────────────────────────────────────────────────────
LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
BACKEND_LABEL=$([ "$SIM_MODE" == true ] && echo "sim" || echo "hardware ($UART_DEVICE)")
echo "  Drivechain ready  [$BACKEND_LABEL]"
echo ""
echo "  Open in browser:"
echo "    http://${LOCAL_IP}:8080/drivechain.html"
echo ""
echo "  CONNECT the motors from the web UI, then use the"
echo "  D-pad or W/A/S/D / arrow keys to drive."
echo ""
echo "  Press Ctrl+C to stop everything."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

wait
