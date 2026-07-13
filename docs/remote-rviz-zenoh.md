# Remote RViz over Zenoh

Current procedure for viewing/driving the robot from RViz on a remote
machine (e.g. the NVIDIA Thor, which is where Gazebo + RViz actually run —
the Pi 5 has no compute GPU and stays hardware-only). Supersedes
`remote-rviz-setup.md` (Fast DDS discovery server + WSL — kept for
reference, not the current workflow).

## Why Zenoh, not plain DDS discovery

DDS multicast discovery doesn't reliably cross Tailscale or multi-interface
boxes (a machine with more than one active network interface, like a laptop
on both Wi-Fi and Tailscale, or Thor). `rmw_zenoh_cpp` avoids this: the Pi
runs a Zenoh router with a known address, and the remote machine connects to
that address directly instead of relying on multicast to find it.

Two scripts, one per machine, both in `scripts/remote/`:

## 1. On the Pi — start the router

```bash
source /opt/ros/lyrical/setup.bash
./scripts/remote/start_zenoh_router.sh
```

Leave it running. It's independent of `./scripts/run_stack.sh` — start it
before or after, either order works, and restarting one doesn't require
restarting the other.

It prints the addresses it's reachable at, e.g. on this Pi:

```text
tcp/172.16.68.73:7447     (LAN)
tcp/100.122.150.74:7447   (Tailscale)
```

## 2. On the remote machine — launch RViz

```bash
./scripts/remote/launch_remote_rviz_zenoh.sh 172.16.68.73     # LAN
./scripts/remote/launch_remote_rviz_zenoh.sh 100.122.150.74   # Tailscale, off-LAN
```

This script:

- sources ROS 2 itself (tries Lyrical, then a few other distros in order —
  no manual `source` needed on the remote side)
- sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and
  `ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/<pi-ip>:7447"]`
  — `client` mode only ever talks through the router, unlike the default
  `peer` mode, which gossips with other peers and then tries to connect to
  them directly using whatever address they advertised (often a loopback
  address on the Pi, unreachable from the remote side)
- restarts the local `ros2 daemon`, since it caches whatever discovery
  config was active when it last started — switching `RMW_IMPLEMENTATION`
  without this restart silently does nothing
- opens RViz with `ws/src/mserve_description/rviz/mserve.rviz`, which has
  `RobotModel` (on `/robot_description`), `TF`, and `LaserScan` (on `/scan`)
  already configured

## Verifying it worked

On the remote machine, once RViz is up:

```bash
ros2 node list       # should show mserve_base, mserve_drivechain, mserve_camera, mserve_lidar, etc.
ros2 topic echo /scan --once
```

If nodes are visible but no topic data arrives, that's the classic
discovery-vs-data-plane split — double check both machines are using the
same Zenoh router address and that nothing (firewall, VPN split-tunnel) is
blocking TCP 7447 between them.

## Driving from the remote side

`teleop_twist_keyboard` or any other `/cmd_vel` publisher works the same way
— just make sure it's launched with the same `RMW_IMPLEMENTATION`/
`ZENOH_CONFIG_OVERRIDE` env vars set (the safety clamp in `mserve_base` still
applies regardless of where the command came from).
