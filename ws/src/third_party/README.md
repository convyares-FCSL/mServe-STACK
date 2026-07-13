# third_party

Vendored dependencies built from source, not apt-installed. This directory
(except this file) is gitignored — nothing here is tracked, and none of it
exists after a fresh `git clone`. Run the commands below once per machine
before building the workspace.

Both entries here exist because this distro ("Lyrical"/Ubuntu Resolute, per
the root `readme.md`) is new enough that neither has a prebuilt apt package
for it yet — the pattern established across this project is: check apt
first (`apt-cache policy ros-lyrical-<name>`), and only vendor source here
if it's genuinely unavailable. `scripts/setup/deps_setup.sh` runs all of
this automatically; the commands below are what it does, spelled out for
manual/partial setup.

## BehaviorTree.ROS2

Used by `ws/src/lifecycle_manager` (BT.CPP-driven configure/activate/shutdown
for the drivechain/base/camera/lidar lifecycle nodes).

```bash
cd ws/src/third_party
git clone --branch humble https://github.com/BehaviorTree/BehaviorTree.ROS2.git
touch BehaviorTree.ROS2/btcpp_ros2_samples/COLCON_IGNORE  # not needed
sudo apt install ros-lyrical-behaviortree-cpp libboost-dev ros-lyrical-generate-parameter-library
```

Then, from `ws/`:

```bash
colcon build --packages-select btcpp_ros2_interfaces behaviortree_ros2 --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

## slam_toolbox

SLAM mapping — builds `/map` live from `/scan` while driving. See
`ws/src/launch/README.md`'s "SLAM Toolbox" section for how it's launched
(`scripts/run_slam.sh` / `mserve_slam.launch.py`), and
`ws/src/interfaces/config/slam_toolbox_params.yaml` for the mServe-specific
param overrides (kept in `interfaces/`, not edited in this vendored copy,
so they survive a fresh clone of this directory).

```bash
cd ws/src/third_party
git clone --branch ros2 --depth 1 https://github.com/SteveMacenski/slam_toolbox.git
sudo apt install libceres-dev libsuitesparse-dev libomp-dev qtbase5-dev \
  ros-lyrical-rviz-common ros-lyrical-rviz-default-plugins ros-lyrical-rviz-ogre-vendor ros-lyrical-rviz-rendering \
  ros-lyrical-bondcpp ros-lyrical-bond ros-lyrical-interactive-markers ros-lyrical-tf2-sensor-msgs \
  ros-lyrical-tf2-geometry-msgs ros-lyrical-message-filters ros-lyrical-pluginlib
```

Then, from `ws/`:

```bash
colcon build --packages-select slam_toolbox --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

Built clean on real hardware (Pi 5) in ~6 minutes, no dependency issues
beyond the apt packages above — see `docs/TODO.md` for the verification
note. `nav2_map_server` shows up in `slam_toolbox`'s `package.xml` as an
`exec_depend` but isn't needed for core mapping — it's for Nav2-integrated
map *serving*, a separate concern from SLAM *building* a map, and
`slam_toolbox` has its own `save_map` service independent of it.
