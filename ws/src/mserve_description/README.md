# mserve_description

Robot description package for mServe.

This package now follows the standard ROS 2 description flow used in the
Articulated Robotics tutorial:

- `urdf/mserve.urdf.xacro` as the top-level robot description
- `urdf/mserve_core.xacro` for the main mobile base geometry
- `urdf/inertial_macros.xacro` for reusable inertia helpers
- `launch/mserve_view.launch.py` to start `robot_state_publisher`, RViz,
  and `joint_state_publisher_gui`
- `rviz/mserve.rviz` as the saved RViz layout

The model currently includes:

- a differential-drive base with left/right wheel joints
- a fixed caster wheel
- reserved frames for lidar, camera, display, and an arm mount

Install missing ROS packages on Ubuntu if needed:

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
```

Primary launch path:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_rviz.sh
```

This script:

- uses a host-only `build_host` / `install_host` / `log_host` workspace
- avoids conflicts with Docker `/ws` symlink installs
- launches `mserve_view.launch.py`, which loads `rviz/mserve.rviz`

Manual build and launch:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mserve_description
source install/setup.bash
ros2 launch mserve_description mserve_view.launch.py
```

Important:

- If you previously built the workspace inside Docker, the container mounts this
  workspace at `/ws`.
- A Docker build with `--symlink-install` creates install-space symlinks that
  point back into `/ws/build/...`.
- That install tree is not safe to source from the host path
  `/home/ecm/ai-workspace/projects/mServe-STACK/ws`.
- If that happens, remove `ws/build`, `ws/install`, and `ws/log`, then rebuild
  in the same environment you plan to run from.
