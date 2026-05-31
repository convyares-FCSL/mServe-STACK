# mserve_bringup

Central bringup and launch support for mServe.

This package contains:

- `launch/mserve_min.launch.py`

The launch file starts:

- `mserve_base` drive lifecycle node
- `mserve_drivechain` drivetrain lifecycle node

Run:

```bash
cd /home/ecm/mServe-STACK/ws
source install/setup.bash
ros2 launch mserve_bringup mserve_min.launch.py
```

When running from the Docker container, the workspace install setup does not always populate `AMENT_PREFIX_PATH` for all packages, so use the helper script:

```bash
cd /home/ecm/mServe-STACK
scripts/05_utils/docker_launch_mserve.sh
```

Notes:

- Shared parameters are loaded from `mserve_interfaces/config/mserve_params.yaml`.
- This package exposes the current minimal runtime topology.
