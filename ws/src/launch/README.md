# mserve_launch

Launch package for mServe. Starts all nodes and the lifecycle manager in the correct order.

## Launch files

- `launch/mserve_min.launch.py` — minimal runtime: base + drivechain + lifecycle manager

## What gets started

| Node | Package | Delay |
|---|---|---|
| `mserve_base` | `mserve_base` | immediate |
| `mserve_drivechain` | `mserve_drivechain` | immediate |
| `lifecycle_manager` | `mserve_lifecycle_manager` | 2s (waits for nodes to come up) |

The lifecycle manager automatically configures and activates all managed nodes via BT.

## Run

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch mserve_launch mserve_min.launch.py
```

## Notes

- Shared parameters loaded from `mserve_interfaces/config/mserve_params.yaml`
- Lifecycle manager is idempotent — safe to restart without restarting managed nodes
