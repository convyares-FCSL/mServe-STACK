# scripts

This folder is reserved for utility and automation scripts used during development, testing, and container setup.

## Fresh machine setup (run in order)

```bash
bash scripts/01_setup/deps_setup.sh      # apt deps + clone/build BehaviorTree.ROS2
bash scripts/02_bootstrap/build_workspace.sh  # build all mServe packages
```

## Available scripts
- `01_setup/env_setup.sh` — configure shell environment for ROS 2 and workspace commands.
- `01_setup/deps_setup.sh` — install apt deps and clone/build source dependencies (BehaviorTree.ROS2). Run once on a fresh machine.
- `02_bootstrap/build_workspace.sh` — build all mServe packages.
- `03_packages/build_packages.sh` — build selected packages.
- `04_tests/run_tests.sh` — run unit tests for the first milestones.
- `05_utils/docker_build_workspace.sh` — build the current ROS workspace inside the Docker container.
- `05_utils/clean_all.sh` — remove workspace build artifacts.
- `05_utils/docker_webbridge.sh` — start rosbridge inside Docker and serve the web UI.
- `05_utils/docker_launch_mserve.sh` — launch the mServe bringup inside the Docker container.
- `05_utils/launch_mserve_description_rviz.sh` — build the description package in a host-only install space and launch RViz safely.
- `05_utils/launch_mserve_description_gazebo.sh` — build the description package in a host-only install space and launch Gazebo safely.

Notes:
- The Docker workspace is mounted at `/ws`.
- If you build in Docker with `--symlink-install`, do not source that same
  `ws/install/setup.bash` from the host filesystem path. Rebuild in the
  environment you intend to run from.
