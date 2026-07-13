# launch

Launch package for mServe. Starts all nodes and the lifecycle manager in the correct order.

## Launch files

- `launch/mserve_min.launch.py` — minimal runtime: drivechain + base + camera + lidar + robot_state_publisher + lifecycle manager
- `launch/mserve_slam.launch.py` — SLAM Toolbox, see its own section below

## What gets started (`mserve_min.launch.py`)

| Node | Package | Executable | Delay |
|---|---|---|---|
| `mserve_drivechain` | `mserve_drivechain` | `drivechain_node` | immediate |
| `mserve_base` | `mserve_base` | `base_node` | immediate |
| `mserve_camera` | `mserve_camera` | `camera_node` | immediate |
| `mserve_lidar` | `mserve_lidar` | `lidar_node` | immediate |
| `robot_state_publisher` | `robot_state_publisher` | `robot_state_publisher` | immediate |
| `lifecycle_manager` | `lifecycle_manager` | `lifecycle_manager` | 2s (waits for nodes to come up) |

The lifecycle manager automatically configures and activates all four lifecycle
nodes above via BT (see `ws/src/lifecycle_manager/`), and runs a shutdown tree
on SIGINT/SIGTERM. `robot_state_publisher` isn't lifecycle-managed — it's a
plain topic publisher with nothing to configure/activate.

## SLAM Toolbox (`mserve_slam.launch.py`)

Vendored source at `ws/src/third_party/slam_toolbox/` (gitignored, cloned from the
`ros2` branch of `SteveMacenski/slam_toolbox` — no apt package exists for
this distro, same situation as `mserve_lidar`'s RPLIDAR SDK). Params in
`interfaces/config/slam_toolbox_params.yaml` (a copy of upstream's own
`mapper_params_online_async.yaml` with `base_frame` corrected to `base_link`
— this robot has no separate footprint frame).

`slam_toolbox`'s own node (`async_slam_toolbox_node`) is already a lifecycle
node, same as every `mserve_*` node — this launch file drives it through
configure/activate itself via `launch_ros`'s `ChangeState`/`OnStateTransition`
event handlers, rather than pulling in the BT-based `lifecycle_manager` for
what's a single node with no bringup sequencing/retries to coordinate.

Deliberately **not** part of `mserve_min.launch.py` or the boot service:
mapping is an occasional, opt-in session, not something you want starting on
every boot alongside the always-on drive stack. Run it with
`scripts/run_slam.sh` once `mserve_min.launch.py` (or the systemd service)
is already up, for `/scan` and the `odom -> base_link` TF it needs.

Once `/map` looks right in RViz/Foxglove:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

## Launch arguments

- `backend` (default `hardware`) — `mserve_drivechain` backend, `hardware` or `sim`
- `uart_device` (default `/dev/ttyAMA0`) — UART device for the hardware backend

## Run

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch launch mserve_min.launch.py backend:=sim
```

`scripts/run_stack.sh` is the normal entry point — it calls this launch
file with the right args for `--sim`/hardware mode and waits for both nodes
to reach `active` before serving the debug web UI.

## Notes

- Shared parameters loaded from `interfaces/config/mserve_params.yaml`
- Lifecycle manager is idempotent — safe to restart without restarting managed nodes
