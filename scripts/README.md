# scripts

All mServe automation, flat and topic-named. If you're looking for "how do I
run the robot," you want `run_stack.sh`; everything else is a focused helper.

```text
scripts/
├── run_stack.sh           main entry point: full stack (hardware or --sim), what systemd runs on boot
├── run_rosbridge.sh       standalone rosbridge only
├── run_foxglove_bridge.sh standalone Foxglove Bridge only (ws://<pi-ip>:8765)
├── run_slam.sh            standalone SLAM Toolbox (needs run_stack.sh already up)
├── run_web_only.sh        standalone static web server only
├── boot_splash.py         paints closed eyes + IP to /dev/fb0 before the container is up (systemd ExecStartPre)
├── clean_all.sh           remove ws/build, ws/install, ws/log
├── setup/
│   ├── deps_setup.sh      apt deps + clone/build vendored third_party (fresh machine, once)
│   └── env_setup.sh       source ROS env — native-ROS machines only (Thor); the Pi has no native ROS
├── remote/
│   ├── start_zenoh_router.sh        Pi-side: Zenoh router for remote RViz (not yet ported to Docker — see docs/TODO.md)
│   └── launch_remote_rviz_zenoh.sh  Thor-side: RViz via the Zenoh router
└── sim/
    ├── launch_mserve_description_gazebo.sh   Gazebo sim (Thor)
    ├── launch_mserve_description_rviz.sh     RViz-only sim view (Thor)
    └── stop_mserve_description_gazebo.sh     stop the above cleanly
```

Zenoh is used for remote RViz (rather than a Fast-DDS discovery server)
because it doesn't depend on matching DDS versions between Pi and Thor and
works across Tailscale without multicast.

## Day to day

```bash
./scripts/run_stack.sh              # hardware, /dev/ttyAMA0 — Foxglove Bridge on by default
./scripts/run_stack.sh --sim        # sim backend, no hardware needed
./scripts/run_stack.sh /dev/ttyACM0 # hardware, custom UART device
./scripts/run_stack.sh --no-foxglove # skip Foxglove Bridge
./scripts/run_stack.sh --slam-map   # + SLAM Toolbox, building/extending a map
./scripts/run_stack.sh --slam-local # + SLAM Toolbox, localizing against a saved map
./scripts/run_stack.sh --nav2       # + Nav2 (implies --slam-local if no SLAM flag given)
```

Flags combine in any order (`--slam-map`/`--slam-local` are mutually
exclusive with each other). The base stack is always started: drivechain,
base, camera, lidar, display, joystick, sensehat, robot_state_publisher,
lifecycle_manager, rosbridge, web UI. Full flag semantics:
`docs/operations.md`.

This is also what `mserve-drivechain.service` runs on boot.

## Notes

- Docker is the only workflow on this Pi (no native ROS install at all) —
  `run_stack.sh` detects this automatically (`ros2` not on PATH) and routes
  everything through `docker compose exec`.
- The Docker workspace is mounted at `/ws`. If you build in Docker with
  `--symlink-install`, do not source that same `ws/install/setup.bash` from
  the host filesystem path — rebuild in the environment you intend to run
  from.
