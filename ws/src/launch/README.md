# launch

Launch package for mServe. Starts all nodes and the lifecycle manager in the correct order.

## Launch files

- `launch/mserve_min.launch.py` — minimal runtime: drivechain + base + lifecycle manager

## What gets started

| Node | Package | Executable | Delay |
|---|---|---|---|
| `mserve_drivechain` | `mserve_drivechain` | `drivechain_node` | immediate |
| `mserve_base` | `mserve_base` | `base_node` | immediate |
| `lifecycle_manager` | `lifecycle_manager` | `lifecycle_manager` | 2s (waits for nodes to come up) |

The lifecycle manager automatically configures and activates all managed nodes via BT
(see `ws/src/lifecycle_manager/`), and runs a shutdown tree on SIGINT/SIGTERM.

## Launch arguments

- `backend` (default `hardware`) — `mserve_drivechain` backend, `hardware` or `sim`
- `uart_device` (default `/dev/ttyAMA0`) — UART device for the hardware backend

## Run

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch launch mserve_min.launch.py backend:=sim
```

`web/run_drivechain_hw.sh` is the normal entry point — it calls this launch
file with the right args for `--sim`/hardware mode and waits for both nodes
to reach `active` before serving the debug web UI.

## Notes

- Shared parameters loaded from `interfaces/config/mserve_params.yaml`
- Lifecycle manager is idempotent — safe to restart without restarting managed nodes
