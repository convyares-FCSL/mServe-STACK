# scripts

All mServe automation lives here, flat and topic-named — no numbered phase
folders. If you're looking for "how do I run the robot," you want
`run_stack.sh` below; everything else is either one-time setup or a
specialized helper.

```text
scripts/
├── run_stack.sh          main entry point: full stack (hardware or --sim)
├── run_rosbridge.sh      standalone rosbridge only
├── run_foxglove_bridge.sh standalone Foxglove Bridge only (ws://<pi-ip>:8765)
├── run_web_only.sh       standalone static web server only
├── clean_all.sh          remove ws/build, ws/install, ws/log
├── setup/
│   ├── deps_setup.sh     apt deps + clone/build BehaviorTree.ROS2 (fresh machine, once)
│   └── env_setup.sh      configure shell environment for ROS 2 / workspace commands
├── build/
│   ├── build_workspace.sh   build all mServe packages
│   └── build_packages.sh    build a specific package selection
├── remote/
│   ├── start_zenoh_router.sh        Pi-side: start a Zenoh router for remote RViz
│   ├── launch_remote_rviz_zenoh.sh  Thor-side: RViz via the Zenoh router (recommended)
│   ├── launch_remote_rviz.sh        Thor-side: RViz via Fast-DDS discovery server (legacy)
│   └── launch_remote_teleop.sh      Thor-side: keyboard teleop via Fast-DDS discovery server (legacy)
├── sim/
│   ├── launch_mserve_description_gazebo.sh   Gazebo sim (Thor)
│   ├── launch_mserve_description_rviz.sh     RViz-only sim view (Thor)
│   └── stop_mserve_description_gazebo.sh     stop the above cleanly
├── docker/
│   ├── docker_build_workspace.sh   build the workspace inside the legacy Docker container
│   ├── docker_launch_mserve.sh     launch bringup inside the container
│   └── docker_webbridge.sh         start rosbridge + web UI inside the container
└── test/
    └── run_tests.sh      run unit tests for the current milestones
```

## Fresh machine setup (run in order)

```bash
bash scripts/setup/deps_setup.sh       # apt deps + clone/build BehaviorTree.ROS2
bash scripts/build/build_workspace.sh  # build all mServe packages
```

## Day to day

```bash
./scripts/run_stack.sh              # hardware, /dev/ttyAMA0
./scripts/run_stack.sh --sim        # sim backend, no hardware needed
./scripts/run_stack.sh /dev/ttyACM0 # hardware, custom UART device
```

This is also what `mserve-drivechain.service` runs on boot — see the
top-level `readme.md`'s "Running on boot" section.

## Remote RViz (Thor)

`remote/start_zenoh_router.sh` (Pi) + `remote/launch_remote_rviz_zenoh.sh`
(Thor) is the current recommended path — see `docs/session.md` for why
(Fast-DDS discovery server had a version mismatch between Pi/Thor; Zenoh
sidesteps it). `launch_remote_rviz.sh`/`launch_remote_teleop.sh` are kept for
the Fast-DDS discovery-server path if you need it again.

## Notes

- Docker (`docker/`) is a legacy fallback only — `run_stack.sh` uses it
  automatically if `ros2` isn't found on PATH, but it's not the primary
  workflow on this Pi anymore (see root `readme.md`).
- The Docker workspace is mounted at `/ws`. If you build in Docker with
  `--symlink-install`, do not source that same `ws/install/setup.bash` from
  the host filesystem path — rebuild in the environment you intend to run
  from.
