# scripts

This folder is reserved for utility and automation scripts used during development, testing, and container setup.

Available scripts:
- `01_setup/env_setup.sh` — configure shell environment for ROS 2 and workspace commands.
- `02_bootstrap/build_workspace.sh` — build the core mServe packages.
- `03_packages/build_packages.sh` — build selected packages.
- `04_tests/run_tests.sh` — run unit tests for the first milestones.
- `05_utils/docker_build_workspace.sh` — build the current ROS workspace inside the Docker container.
- `05_utils/clean_all.sh` — remove workspace build artifacts.
- `05_utils/docker_webbridge.sh` — start rosbridge inside Docker and serve the web UI.
- `05_utils/docker_launch_mserve.sh` — launch the mServe bringup inside the Docker container.
