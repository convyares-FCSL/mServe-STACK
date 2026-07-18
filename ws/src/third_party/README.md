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
`ws/src/interfaces/config/slam_params_map.yaml` /
`slam_params_local.yaml` for the mServe-specific param overrides (kept in
`interfaces/`, not edited in this vendored copy, so they survive a fresh
clone of this directory).

```bash
cd ws/src/third_party
git clone --branch ros2 --depth 1 https://github.com/SteveMacenski/slam_toolbox.git
sudo apt install libceres-dev libsuitesparse-dev libomp-dev qtbase5-dev \
  ros-lyrical-rviz-common ros-lyrical-rviz-default-plugins ros-lyrical-rviz-ogre-vendor ros-lyrical-rviz-rendering \
  ros-lyrical-bondcpp ros-lyrical-bond ros-lyrical-interactive-markers ros-lyrical-tf2-sensor-msgs \
  ros-lyrical-tf2-geometry-msgs ros-lyrical-message-filters ros-lyrical-pluginlib
```

**Required patch before building** (2026-07-13) — `serialize_map` (the
service `--slam-local` needs a real save from) fails on every call without
this; `save_map`/online mapping work fine either way, so this is easy to
miss until you actually try localization mode. Upstream never registered
`slam_toolbox::LoopClosureListener` (the one concrete listener
`slam_toolbox_common.cpp` unconditionally attaches to every `Mapper`) with
Boost.Serialization, so saving a pose graph throws "unregistered class"/
"unregistered void cast" partway through and never writes the `.data` file
(only a truncated `.posegraph`). Three edits, all straightforward — see the
`// mServe:` comments left in place in each file for the full why:

1. `include/karto_sdk/Mapper.h` — `MapperLoopClosureListener::serialize()`
   was an empty no-op; add `ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(MapperListener);`
   inside it (registers the void_cast to its base).
2. `include/slam_toolbox/loop_closure_listener.hpp` — add
   `#include <boost/serialization/{access,base_object,export}.hpp>`; a
   private `LoopClosureListener() = default;` constructor (+
   `friend class boost::serialization::access;`) since Boost needs one to
   deserialize and the class only had the 3-argument constructor; a
   private `serialize()` calling
   `ar & boost::serialization::base_object<karto::MapperLoopClosureListener>(*this);`
   (same void_cast reasoning as #1, one level up — inheriting the base's
   serialize() isn't enough, it has to be redeclared at this level too);
   and `BOOST_CLASS_EXPORT_KEY(slam_toolbox::LoopClosureListener)` after
   the class.
3. `src/loop_closure_listener.cpp` — add
   `BOOST_CLASS_EXPORT_IMPLEMENT(slam_toolbox::LoopClosureListener)` right
   after the `#include`.

**Second required patch** (2026-07-18, hit wiring this into Docker — a
toolchain-generation gap like the Nav2 ones below, not something the Boost
patch above touches) — build fails with `message_filters::Subscriber<...>`
"no matching function for call" on `slam_toolbox_common.cpp`'s
`scan_filter_sub_` construction. This distro's `message_filters` (apt,
dated after this fork's `ros2` branch was last synced) changed
`Subscriber`'s constructors to be specific to its `NodeType` template
parameter (default `rclcpp::Node`) with no longer any overload generic
enough to accept an `rclcpp_lifecycle::LifecycleNode*`, and dropped the
`rclcpp::QoS`-taking overload entirely (only `rmw_qos_profile_t` now). Two
edits, `// mServe:` comments in place:

1. `include/slam_toolbox/slam_toolbox_common.hpp` — `scan_filter_sub_`'s
   type: `message_filters::Subscriber<sensor_msgs::msg::LaserScan>` →
   `message_filters::Subscriber<sensor_msgs::msg::LaserScan,
   rclcpp_lifecycle::LifecycleNode>` (explicit second template arg).
2. `src/slam_toolbox_common.cpp` — the matching construction call: same
   explicit template arg, plus
   `rclcpp::SensorDataQoS()` → `rclcpp::SensorDataQoS().get_rmw_qos_profile()`
   (explicit conversion to the only QoS type this version's constructor
   still accepts).

Then, from `ws/`:

```bash
colcon build --packages-select slam_toolbox --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

If building in a scoped workspace (like Docker's `run_stack.sh`, which only
`--packages-select`s what it needs rather than the whole `ws/src` tree) and
`ws/src/third_party/navigation2/` also happens to be checked out: colcon
discovers its *vendored source* package.xml files regardless of
`--packages-select`, and `nav2_map_server`'s local `build_depend` on
`nav2_ros_common` (→ `backward_ros`, no apt package either) cascades into a
hard error trying to build them from source — even though the already-
apt-installed `nav2_common`/`nav2_msgs`/`nav2_util`/`nav2_map_server`
satisfy `slam_toolbox`'s actual `exec_depend` just fine (colcon falls back
to them with only a warning, for everything *not* also chasing
`nav2_ros_common`). Add
`--packages-ignore nav2_common nav2_msgs nav2_util nav2_map_server nav2_ros_common`
to keep colcon from ever consulting the vendored package.xml files.

Built clean on real hardware (Pi 5) in ~5 minutes, no dependency issues
beyond the apt packages above — see `docs/TODO.md` for the verification
note. `nav2_map_server` shows up in `slam_toolbox`'s `package.xml` as an
`exec_depend` but isn't needed for core mapping — it's for Nav2-integrated
map *serving*, a separate concern from SLAM *building* a map, and
`slam_toolbox` has its own `save_map` service independent of it.

## navigation2 (Nav2)

Not yet wired into any mserve launch file (work in progress) — this is
just what's needed to get the minimal package set building. No apt package
exists for this distro either. Clone the `main` branch (rolling-tracking) —
`humble`/`jazzy` risk an older BT.CPP API than the `4.9.0` this distro's
`ros-lyrical-behaviortree-cpp` actually provides (see patch 1 below, hit
exactly that).

```bash
cd ws/src/third_party
git clone --branch main --depth 1 https://github.com/ros-navigation/navigation2.git
sudo apt install ros-lyrical-action-msgs ros-lyrical-ament-cmake-python ros-lyrical-ament-index-cpp \
  ros-lyrical-angles ros-lyrical-backward-ros ros-lyrical-behaviortree-cpp \
  ros-lyrical-bond ros-lyrical-bondcpp ros-lyrical-builtin-interfaces \
  ros-lyrical-cv-bridge ros-lyrical-diagnostic-updater ros-lyrical-geographic-msgs \
  ros-lyrical-geometry-msgs ros-lyrical-image-transport ros-lyrical-laser-geometry \
  ros-lyrical-launch ros-lyrical-launch-ros ros-lyrical-lifecycle-msgs \
  ros-lyrical-map-msgs ros-lyrical-message-filters ros-lyrical-nav-msgs \
  ros-lyrical-osrf-pycommon ros-lyrical-pluginlib ros-lyrical-point-cloud-transport \
  ros-lyrical-point-cloud-transport-plugins ros-lyrical-rclcpp ros-lyrical-rclcpp-action \
  ros-lyrical-rclcpp-components ros-lyrical-rclcpp-lifecycle ros-lyrical-rcl-interfaces \
  ros-lyrical-rclpy ros-lyrical-rmw ros-lyrical-robot-localization \
  ros-lyrical-rosidl-default-runtime ros-lyrical-sensor-msgs ros-lyrical-std-msgs \
  ros-lyrical-std-srvs ros-lyrical-tf2 ros-lyrical-tf2-geometry-msgs ros-lyrical-tf2-msgs \
  ros-lyrical-tf2-ros ros-lyrical-tf2-sensor-msgs ros-lyrical-unique-identifier-msgs \
  ros-lyrical-visualization-msgs libeigen3-dev libgraphicsmagick++1-dev
```

**Required patches before building** — every other dependency above
resolved fine via apt (unlike slam_toolbox/BT.ROS2, nothing else needed
vendoring); these two are toolchain-generation gaps between what `main` was
tested against and what's actually installed here:

1. `nav2_behavior_tree/include/nav2_behavior_tree/utils/loop_rate.hpp` —
   used `tree_->wakeUpSignal()->waitFor(...)`, an older BT.CPP API this
   distro's `4.9.0` package doesn't have (`Tree` has no `wakeUpSignal`
   member). Replace both `wake_up->waitFor(...)` call sites with
   `tree_->sleep(...)` directly (4.9.0's `Tree::sleep(timeout)` does the
   same interruptible wait — interrupted by `emitWakeUpSignal()`, returns
   `true` if interrupted before timeout — in one call), and delete the
   `auto wake_up = tree_->wakeUpSignal();` line.
2. `nav2_common/cmake/nav2_package.cmake` — drop `-Werror` from the
   `add_compile_options(...)` call for GNU/Clang (keep the rest of the
   warning flags). This GCC (15, Ubuntu 26.04/Resolute) is newer than what
   `main` was tested against and turns real-but-harmless deprecations
   (`std::atomic_load`/`store` in `nav2_costmap_2d`, superseded by
   `std::atomic<shared_ptr<T>>` in C++20) and at least one outright false
   positive (`-Wnull-dereference` inside GCC-inlined, auto-generated ROS
   message `operator==` code in `builtin_interfaces`, not Nav2's own logic)
   into hard build failures. Every warning still prints, just isn't fatal.

Minimal package set that's actually been built and verified so far (all 18
finish clean with the patches above):

```bash
colcon build --packages-up-to nav2_bt_navigator nav2_regulated_pure_pursuit_controller \
  nav2_navfn_planner nav2_behaviors nav2_velocity_smoother nav2_waypoint_follower \
  nav2_lifecycle_manager --cmake-args -DBUILD_TESTING=OFF --symlink-install
```

Deliberately scoped down from the full 34-package repo: no `nav2_amcl`
(this robot uses `slam_toolbox` in localization mode instead of AMCL, same
decision as for mapping — see `docs/plan.md`'s Current Open Questions),
`nav2_smac_planner`/`nav2_mppi_controller`/etc. (extra plugin alternatives,
not needed to have *a* working planner/controller),
`nav2_rviz_plugins`/`nav2_simple_commander`/`nav2_docking`/etc. (optional
tooling). `nav2_map_server` gets pulled in anyway as a transitive
`test_depend` of `nav2_costmap_2d` even with `BUILD_TESTING=OFF` (colcon's
dependency graph includes test_depends regardless) — needs
`libgraphicsmagick++1-dev` specifically, not the C-only
`libgraphicsmagick1-dev`, easy to grab the wrong one.
