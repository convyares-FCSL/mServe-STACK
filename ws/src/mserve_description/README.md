# mserve_description

Robot description package for mServe.

This package now follows the standard ROS 2 description flow used in the
Articulated Robotics tutorial:

- `urdf/mserve.urdf.xacro` as the top-level robot description
- `urdf/mserve_core.xacro` for the main mobile base geometry
- `urdf/mserve_lidar.xacro` for the RPLIDAR C1 mount and Gazebo laser sensor
- `urdf/inertial_macros.xacro` for reusable inertia helpers
- `urdf/mserve_gazebo.xacro` for Gazebo-specific contact and drive-system tuning
- `params/mserve_gazebo_bridge.yaml` for ROS 2 <-> Gazebo topic bridges
- `launch/mserve_rviz.launch.py` to start `robot_state_publisher`, RViz,
  and `joint_state_publisher_gui`
- `launch/mserve_gazebo.launch.py` to start Gazebo Sim and spawn the robot
- `rviz/mserve.rviz` as the saved RViz layout

The model currently includes:

- a differential-drive base with left/right wheel joints
- a fixed caster wheel
- an RPLIDAR C1-style lidar on `lidar_link`
- reserved frames for camera, display, and an arm mount

Install missing ROS packages on Ubuntu if needed:

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
```

If you want to use Gazebo Harmonic with ROS 2 Jazzy as well:

```bash
sudo apt install ros-jazzy-ros-gz
```

If you want keyboard teleop for Gazebo testing as well:

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

## Launch Normal

Primary RViz-only path:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_rviz.sh
```

This script:

- uses a host-only `build_host` / `install_host` / `log_host` workspace
- avoids conflicts with Docker `/ws` symlink installs
- launches `mserve_rviz.launch.py`, which loads `rviz/mserve.rviz`

Manual RViz launch:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mserve_description
source install/setup.bash
ros2 launch mserve_description mserve_rviz.launch.py
```

## Launch Gazebo

Primary Gazebo path:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_gazebo.sh
```

Headless Gazebo path for RViz-only sensor inspection:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_gazebo.sh --headless
```

This script:

- uses the same host-only `build_host` / `install_host` / `log_host` workspace
- avoids conflicts with Docker `/ws` symlink installs
- launches `mserve_gazebo.launch.py`
- accepts `--headless` as a shortcut for `headless:=true`

Manual Gazebo launch:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mserve_description
source install/setup.bash
ros2 launch mserve_description mserve_gazebo.launch.py
```

Manual headless Gazebo launch:

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mserve_description
source install/setup.bash
ros2 launch mserve_description mserve_gazebo.launch.py headless:=true
```

## Teleop Quick Start

Terminal 1: launch Gazebo

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_gazebo.sh --headless
```

Terminal 2: source the host install space

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
source install_host/setup.bash
```

Terminal 2: drive with teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

Optional checks:

```bash
ros2 topic echo /odom
ros2 topic echo /scan
```

## Gazebo Integration

The modern Gazebo Sim (Harmonic) integration uses:

- **Systems**: `DiffDrive` (differential drive control) and `JointStatePublisher` defined in
  `urdf/mserve_gazebo.xacro`. Per the [Harmonic migration guide](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/),
  model-specific systems belong with the model definition, not at the world level.
- **Lidar Sensor**: `urdf/mserve_lidar.xacro` adds a `gpu_lidar` sensor on `lidar_link`
  using the RPLIDAR C1 nominal scan rate, 360 degree coverage, and 12 m max range.
- **Bridge Configuration**: `params/mserve_gazebo_bridge.yaml` bridges the following topics:
  - `cmd_vel` (ROS → Gazebo)
  - `odom` (Gazebo → ROS)
  - `tf` (Gazebo → ROS)
  - `joint_states` (Gazebo → ROS)
  - `scan` (Gazebo → ROS)
  - `clock` (Gazebo → ROS)

### Testing the Gazebo Integration

**Terminal 1**: Launch Gazebo

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_gazebo.sh --headless
```

**Terminal 2**: Publish velocity commands

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
source install_host/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
```

Or use **teleop_twist_keyboard**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

**Terminal 3**: Monitor odometry

```bash
ros2 topic echo /odom
```

### Validation

- xacro expands cleanly
- `gz sdf -p` converts the URDF and preserves both Gazebo system plugins
- `colcon build --packages-select mserve_description` passes
- Unit tests pass

### Drive System Parameters

Drive system limits are derived from motor specifications:

- **wheel_separation**: from `wheel_y_offset`
- **max_linear_velocity**: from rated wheel speed
- **max_angular_velocity**: from wheel separation
- **max_linear/angular_acceleration**: from rated torque and total robot mass

Notes:

- `mserve_gazebo.launch.py` launches `gz_sim`, expands the xacro through
  `robot_state_publisher`, and spawns the robot from `/robot_description`.
- `headless:=true` switches Gazebo to `-s --headless-rendering`, which avoids
  the Gazebo GUI while still allowing rendered sensors such as the lidar to
  publish into ROS 2 for RViz.
- The default world file is `basic.sdf`, so the default spawn service world name
  is `basic`.
- `use_bridge:=true` enables the bridge node (default).

## Troubleshooting

### Robot doesn't move when using teleop_twist_keyboard

**Terminal 1**: Launch Gazebo

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_gazebo.sh --headless
```

**Terminal 2**: Launch teleop

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
source install_host/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3**: Diagnostic checks

1. Check if `/cmd_vel` is being published:

```bash
ros2 topic echo /cmd_vel --max-msgs=3
```

(While holding a movement key in the teleop window, you should see `Twist` messages)

2. Verify the bridge is running and connected:

```bash
ros2 node list | grep bridge
ros2 topic list | grep -E "cmd_vel|odom"
```

3. Check Gazebo topic echo (requires `gz` CLI):

```bash
gz topic -e /cmd_vel --max-msgs=3
```

(While holding a movement key, you should see incoming `Twist` messages)

4. Verify robot_state_publisher is running:

```bash
ros2 node list | grep robot_state_publisher
ros2 param list | grep robot_description
```

5. Check for errors in the bridge or Gazebo logs:

```bash
# In the terminal where Gazebo was launched, look for error messages
# Or check the launch output for warnings about failed bridges
```

### Common Issues

- **Teleop publishes but robot doesn't move**: The bridge may not be forwarding correctly. Check that `mserve_gz_bridge` node is running and has no errors.
- **No `/cmd_vel` topic**: Check that `teleop_twist_keyboard` is running. It may be waiting for input focus in its terminal window.
- **Bridge shows errors about message type mismatch**: Verify the YAML bridge config has correct message types for the gazebo messages.
- **RViz shows robot but Gazebo doesn't move**: Make sure the bridge `use_bridge:=true` argument is set (it's the default).

Important:

- If you previously built the workspace inside Docker, the container mounts this
  workspace at `/ws`.
- A Docker build with `--symlink-install` creates install-space symlinks that
  point back into `/ws/build/...`.
- That install tree is not safe to source from the host path
  `/home/ecm/ai-workspace/projects/mServe-STACK/ws`.
- If that happens, remove `ws/build`, `ws/install`, and `ws/log`, then rebuild
  in the same environment you plan to run from.
