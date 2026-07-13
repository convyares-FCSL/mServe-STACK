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
this distro, same situation as `mserve_lidar`'s RPLIDAR SDK). A `mode` launch
argument (`map` or `local`, default `map`) picks between two params files in
`interfaces/config/` — `slam_params_map.yaml` (`mode: mapping`, build/extend
a map) or `slam_params_local.yaml` (`mode: localization`, localize against a
previously saved map). Both are copies of upstream's own
`mapper_params_online_async.yaml` with `base_frame` corrected to `base_link`
(this robot has no separate footprint frame). `slam_params_local.yaml`'s
`map_file_name` needs to be set to a real saved map before `local` mode does
anything useful — see that file's header comment: it needs `serialize_map`'s
output specifically, not `save_map`'s (different file formats, see below).

`slam_toolbox`'s own node (`async_slam_toolbox_node`) is already a lifecycle
node, same as every `mserve_*` node — this launch file drives it through
configure/activate/shutdown with a **second `lifecycle_manager` instance**
(node name `slam_lifecycle_manager`, to avoid colliding with the always-on
one from `mserve_min.launch.py`), pointed at its own
`trees/slam_bringup.xml`/`trees/slam_shutdown.xml` pair via the
`bringup_tree_file`/`shutdown_tree_file` params (see
`ws/src/lifecycle_manager/`) — same retry-on-failure bringup and
graceful-shutdown-on-SIGINT behavior as drivechain/base/camera/lidar get,
just scoped to one node. This replaced an earlier version that used plain
`launch_ros` `ChangeState`/`OnStateTransition` event handlers for
configure/activate — that approach had no shutdown path at all, so Ctrl+C
just SIGTERM'd `slam_toolbox` with no deactivate/cleanup transition.

Kept as a *separate* `lifecycle_manager` instance rather than folding
`slam_toolbox` into the main `bringup.xml`/`shutdown.xml`: since SLAM is
opt-in (`--slam-map`/`--slam-local`), adding it to the tree that always runs
would mean every normal startup pays a ~4s dead-service-lookup timeout (2s
each for the configure/activate `IsInState` checks) waiting on a node that
isn't there.

Deliberately **not** part of `mserve_min.launch.py` or the boot service:
mapping is an occasional, opt-in session, not something you want starting on
every boot alongside the always-on drive stack. Run it with
`scripts/run_slam.sh [map|local]` once `mserve_min.launch.py` (or the
systemd service) is already up, for `/scan` and the `odom -> base_link` TF
it needs.

Once `/map` looks right in RViz/Foxglove, there are two different things you
might want to save — they are **not interchangeable**:

```bash
# For viewing/serving as a static map (nav_msgs-style .pgm + .yaml):
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"

# For `local` mode to actually load (slam_toolbox's own pose-graph, .posegraph + .data):
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/absolute/path/my_map'}"
```

Then set `slam_params_local.yaml`'s `map_file_name` to that same absolute
path (no extension) before using `local` mode.

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
