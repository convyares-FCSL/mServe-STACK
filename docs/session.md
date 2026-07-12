# Session Notes

## 2026-05-31

Initial planning direction:

- mServe should be a ROS 2 Jazzy C++ learning project.
- The C++ lessons in `/home/ecm/ros2-systems-operability/src/2_cpp` are the style guide.
- The skeleton should be small, buildable, and easy to expand.
- Documentation should live under `docs/`.
- The root `plan.md` should stay short and point to focused docs.

Decisions captured:

- Avoid `ros2_control` at the start so motor control and communication are visible.
- Add a dedicated `mserve_esp32` package for motor-controller comms.
- Keep robot description readable; avoid heavy Xacro use in the first pass.
- Track tasks in `docs/TODO.md`.
- Track session context and decisions in this file.

Files created or reorganized:

- `plan.md`
- `docs/README.md`
- `docs/architecture.md`
- `docs/packages.md`
- `docs/milestones.md`
- `docs/testing-and-scripts.md`
- `docs/TODO.md`
- `docs/session.md`

## 2026-05-31 — Docker / Web UI / Lifecycle

### What was done

Docker and rosbridge setup:

- `docker-compose.yml` updated: `./web` mounted as `/web` inside the container, port 9090 added for rosbridge.
- `scripts/05_utils/docker_webbridge.sh` fixed: web server was being started on the host where Docker's proxy already owned port 8080. Both the web server and rosbridge now run inside the container via `docker compose exec -d`.

Web UI fixes:

- `roslib.min.js` vendored locally (`web/roslib.min.js`). The Pi 5 has no internet access so the unpkg CDN script tag was silently failing. jsdelivr was reachable for the one-time download.
- `index.html` updated to load roslib from the local file.
- `app.js`: rosbridge URL changed from hardcoded `ws://localhost:9090` to `ws://${window.location.hostname}:9090` so the UI works when opened from another machine using the Pi's IP.
- Lifecycle transition IDs corrected. The original values (`configure: 10, activate: 40, deactivate: 0, cleanup: 80`) were wrong — those are internal callback IDs. Correct values: `configure: 1, activate: 3, deactivate: 4, cleanup: 2`.
- Shutdown button added to each node card. Shutdown requires a different transition ID depending on current state (`unconfigured→5, inactive→6, active→7`), so JS now tracks each node's state and selects the right ID.
- State label colour-coded: green=active, blue=inactive, white=unconfigured, red=finalized/error/unavailable.
- All buttons disabled automatically when a node is in `finalized` or unreachable state.

### Decisions

- Gazebo simulation will run on a PC (WSL2 / Ubuntu), not on the Pi 5. The Pi 5 has no compute GPU and Gazebo Harmonic is too heavy for it. The Pi 5 runs the real robot nodes; the PC runs simulation and development tooling.
- Both machines can share `ROS_DOMAIN_ID` over the network so the web UI on the Pi can reflect live sim data.
- Repo will be pushed to GitHub and picked up on the PC for the Gazebo milestone.

## 2026-07-12 — SD-card migration: Docker → native, Jazzy → Lyrical

### What was done

- Retired the Pi OS / internal NVMe SSD install; Pi now boots Ubuntu 26.04
  ("Resolute") from the SD card. Restored `mServe-STACK` from the pre-migration
  USB backup (`backup-2026-07-12/`).
- Moved off Docker entirely — the stack now runs natively. Installed native
  build tooling (`build-essential`, `python3-colcon-common-extensions`,
  `ros-lyrical-behaviortree-cpp`, `ros-lyrical-rosbridge-server`) since this
  Pi never had a native ROS/C++ toolchain before (Docker did all the building).
- This Pi's native ROS distro is **Lyrical**, not Jazzy (which this project
  was originally built against). Found and fixed a real breaking change:
  `ament_target_dependencies()` was **removed entirely** in Lyrical (not
  deprecated — the macro doesn't exist anywhere in the install). Migrated all
  call sites in `utils`, `mserve_drivechain`, `mserve_base` CMakeLists to
  `target_link_libraries()` with modern imported targets
  (`rclcpp::rclcpp`, `<pkg>::<pkg>` aggregate targets for message packages).
- Configured the Pi 5 GPIO UART for the DDSM Driver HAT: `dtparam=uart0=on` +
  `dtoverlay=disable-bt` in `/boot/firmware/config.txt`, masked
  `serial-getty@ttyAMA0` so it can't fight for the port.
- Recreated `mserve-drivechain.service` as a native systemd unit (sources
  `/opt/ros/lyrical/setup.bash` + workspace `install/setup.bash`, no
  `Requires=docker.service`).
- Validated natively end-to-end in `--sim` mode before touching hardware:
  both lifecycle nodes configure/activate, BT trees load, rosbridge and the
  web UI come up, clean SIGTERM shutdown.
- Gazebo + RViz simulation moved to the NVIDIA Thor — both now run there, not
  a PC/WSL2 (see `docs/simulation_hil.md`, `docs/remote-rviz-setup.md`).
- Updated `readme.md`, `CLAUDE.md`, and this `docs/` folder to match. Along
  the way, confirmed (and documented) that the "ESP32 motor boundary" concept
  in `docs/architecture.md`/`docs/packages.md` was implemented differently
  than planned: the ESP32 is a physical board with its own firmware
  (`mserve_drivechain/drive_firmware/`), not a separate `mserve_esp32` ROS
  package — `mserve_drivechain` owns the JSON-over-UART client side directly.

### Known remaining staleness (not fixed this session — flagged for later)

- Package-level READMEs (e.g. `ws/src/mserve_drivechain/README.md`) reference
  a stale path (`~/ai-workspace/projects/mServe-STACK`) and port 8080 for the
  web UI; the actual `run_stack.sh` serves on port 6240.
- `docs/milestones.md`, `docs/packages.md`, `docs/TODO.md` describe an early
  package layout (`mserve_interfaces`, `mserve_bringup`, per-package folders
  prefixed `mserve_`) that doesn't match current `ws/src/` folder names
  (`interfaces`, `utils`, `launch`).

## 2026-07-12 — lifecycle_manager wired to real nodes, shutdown bug fixed

### What was done

- Removed leftover `hyfleet_subsystem`-derived scaffolding not used by
  mserve: deleted `mserve_base_archive/` and the booster/compression
  msgs+srvs (`ControlBooster`, `ControlCompressor`, `SystemState`,
  `BoosterCmd`, `CompressorCmd`, `DispenserCmd`, `GasRouterCmd`, `SetMode`,
  `Cmd`) from `interfaces/` — `interfaces/CMakeLists.txt` still listed the
  deleted files in `rosidl_generate_interfaces()`, which broke a full
  `colcon build`; fixed.
- Vendored `behaviortree_ros2`/`btcpp_ros2_interfaces` at
  `ws/src/BehaviorTree.ROS2/` (gitignored, `humble` branch) — `lifecycle_manager`
  depended on these but the directory was an empty, never-populated stub.
  Installed the two missing system deps (`libboost-dev`,
  `ros-lyrical-generate-parameter-library`).
- Fixed `lifecycle_manager`'s CMakeLists (same removed-`ament_target_dependencies()`
  issue as the July SD-card migration, just not caught yet since it carried
  no `COLCON_IGNORE` and was never built standalone).
- `bringup.xml`/`shutdown.xml` still referenced the old `hyfleet_subsystem`
  node names (`low_booster`, `high_booster`, `hyfleet_compression`) — retargeted
  to the real `mserve_drivechain`/`mserve_base` nodes. Also fixed
  `shutdown.xml`'s `transition="shutdown"`, which isn't a recognized
  transition name (only `shutdown_unconfigured`/`_inactive`/`_active` are,
  per `mserve_utils::lifecycle::transitionIdFromName`) — every shutdown
  attempt would have failed that lookup.
- **Found and fixed a real shutdown bug via testing**: `rclcpp::on_shutdown()`
  fired after the ROS context was already invalidated, so the shutdown
  tree's service calls silently failed and both nodes were left stuck
  `active` on every SIGINT. Fixed with the standard pattern: `main.cpp`
  disables rclcpp's default signal handling and installs a plain
  `std::signal` handler that flags `shutdown_requested_`; the existing
  100ms tick timer runs the shutdown tree while the context is still valid,
  then calls `rclcpp::shutdown()` itself once it completes.
- `mserve_min.launch.py` didn't launch `mserve_base` at all, and had two more
  bugs: wrong executable name (`mserve_drivechain` instead of
  `drivechain_node`), and a `name='drivechain'` override that would have
  renamed the running node away from what `lifecycle_manager`'s tree
  targets (`mserve_drivechain`) — silently breaking the whole integration.
  Fixed, and added `backend`/`uart_device` launch args.
- `run_stack.sh` simplified per request: no longer spawns nodes or
  calls `ros2 lifecycle set` itself — it launches `mserve_min.launch.py` and
  waits for both nodes to reach `active`. Its `cleanup()` signals
  `lifecycle_manager` directly (SIGINT) rather than relying on `ros2
  launch`'s own signal cascade to children, which proved unreliable when
  sent programmatically from a script's trap handler (worked fine sent
  interactively to the `ros2 launch` process directly, but not when the
  script did the same thing on its own SIGINT).
- Verified end-to-end in `--sim` mode, repeatedly: bringup drives both
  nodes `unconfigured → inactive → active` in order (drivechain then base),
  and Ctrl+C now cleanly runs the shutdown tree (`mserve_base shut down` →
  `mserve_drivechain Shutdown`) with zero leftover processes and no
  force-kills needed.
- Updated docs to match: this file, `readme.md`, `docs/TODO.md`,
  `ws/src/lifecycle_manager/README.md` + `docs/todo.md`, `ws/src/launch/README.md`,
  `web/README.md`.
- Brought the four early-planning docs (`docs/architecture.md`,
  `docs/packages.md`, `docs/milestones.md`, `docs/plan.md`) current too —
  they'd drifted since the 05-31 planning session (`mserve_interfaces` →
  `interfaces`, `mserve_bringup` → `launch`, no separate `mserve_esp32`,
  `WheelCommand`/`WheelFeedback` → `MotorCommand`/`MotorState`/
  `DriveMotorFeedback`, `lifecycle_manager` not in the plan at all,
  `docs/milestones.md` checkmarks not matching actual test/description/sim
  completion state). Kept the actual design content (control philosophy,
  lifecycle rules, package responsibilities) unchanged — this pass only
  corrected names/status against current `ws/src/`, not the decisions
  themselves.

### Not done yet

- Only tested in `--sim`; hardware backend (`/dev/ttyAMA0`) not re-verified
  against this session's changes.
- Camera/lidar bring-up (see `docs/continue.md`) — no camera or lidar
  hardware was detected connected to the Pi when checked this session.

## 2026-07-12 — camera on real hardware, remote RViz, real odometry

### Camera hardware confirmed + mserve_camera package

- User plugged in a USB webcam (UVC, `HD Web Camera` 05a3:9331, `uvcvideo`
  driver, `/dev/video0`) — confirmed via `lsusb`/`dmesg`/`v4l2-ctl`. Also has
  a mic (`hw:2,0`, capture-only — `pcmC2D0c`, no `p` node, confirmed via
  `arecord -l` after fixing a `video`/`audio` group-membership permissions
  gap for both).
- Installed `ros-lyrical-v4l2-camera`. Tried `ros-lyrical-audio-capture` for
  the mic — blocked: depends on `libgstreamer-plugins-good1.0-0`, which
  doesn't exist anywhere in this Ubuntu Resolute release's repos. Deferred
  (only `audio-common-msgs` installed, no GStreamer dependency).
- Built `mserve_camera` (new package) — lifecycle node wrapping
  `v4l2_camera`'s `V4l2CameraDevice` class directly (same pattern as
  `lifecycle_manager` wrapping BT.CPP, and `mserve_drivechain` wrapping
  `DriveUart`), not the whole `v4l2_camera_node`. YUYV @ 640x480 chosen over
  MJPG/H264 — uncompressed, zero conversion code, this camera's YUYV mode
  hits 30fps at this resolution. Frame rate observed at ~12.6Hz in practice
  (frame *interval* is never explicitly requested via `VIDIOC_S_PARM`, only
  format/resolution) — tracked as a known gap, not fixed.
- Installed `ros-lyrical-web-video-server` to transcode the raw YUYV image to
  MJPEG over plain HTTP (port 8080) for the browser — found and fixed a real
  QoS bug in the process: published `camera/image_raw`/`camera/camera_info`
  with `SensorDataQoS()` (best-effort, correct convention), but
  `web_video_server` hardcodes a reliable subscription with no per-stream
  override, so it silently dropped every frame. Switched to reliable QoS.
- Web UI: `web/camera.html`/`camera.js` (lifecycle, params, image, camera_info,
  log — same pattern as `drivechain.html`), image preview added to
  `web/base.html`, `mserve_camera` lifecycle card added to `index.html`
  (`app.js`'s per-node loops were already keyed off a `nodePrefix` map, so
  this was a 2-line change).
- Bug found via testing: forgot to add `camera_node` to
  `run_stack.sh`'s process-cleanup lists when scaffolding the new
  package — the shutdown tree correctly drove it to `finalized`, but nothing
  then killed the OS process. Fixed in all 5 spots (matches `drivechain_node`'s
  coverage).

### Remote RViz (Thor) — discovery rabbit hole, landed on same-LAN multicast

- Goal: get RViz on Thor to see the Pi's ROS graph (finishing `docs/continue.md`
  Phase 1). Found `robot_state_publisher` was never wired into the real
  hardware launch path (`mserve_min.launch.py`) at all — added it, sourcing
  the URDF from `mserve_description` via `xacro` (which had to be installed —
  wasn't on the Pi). Hit two real bugs along the way: `launch_ros` tries to
  parse a bare `Command(...)` result as YAML by default, which breaks on
  URDF/XML content — fixed by wrapping in `ParameterValue(..., value_type=str)`;
  and adding `mserve_description` as an `exec_depend` of the `launch` package
  created a real circular dependency, because `launch` is also the literal
  name of the core ROS 2 launch framework package, and `mserve_description`
  already depends on that (the framework, not us) — colcon resolves the
  ambiguous name to the local package first. Left the dependency out (it's
  Python, resolved at runtime, not needed for build ordering) and documented
  the trap in `launch/package.xml`.
- Tried Fast-DDS Discovery Server over Tailscale first (matching
  `docs/remote-rviz-setup.md`'s old WSL-era pattern, roles swapped since the
  Pi now hosts the real graph and Thor is the client). Got it fully working
  on the Pi side (server reachable, all nodes/topics visible locally) but
  Thor never saw the graph despite every individual piece checking out
  correct — env vars byte-exact, firewall/Tailscale ACLs not blocking UDP,
  no timing issue. Root cause: Fast-DDS major version mismatch — Pi runs
  3.6.1 (Lyrical), Thor runs 2.14.6 (Jazzy) — a version boundary eProsima
  made breaking discovery-server wire-protocol changes across.
  - Along the way: confirmed the Pi and Thor are actually on the **same LAN
    subnet** (`172.16.0.0/16` WiFi) — the whole Tailscale/discovery-server
    detour was unnecessary for this setup. Plain multicast discovery
    (matching `ROS_DOMAIN_ID`, no `ROS_DISCOVERY_SERVER`) worked immediately
    once tried.
  - Also explored `rmw_zenoh_cpp` as a longer-term answer for the
    off-LAN/VPN case (available on both sides: Pi `0.10.4`, Thor `0.2.9`) —
    got it fully working via a `rmw_zenohd` router with explicit
    `ZENOH_CONFIG_OVERRIDE` connect endpoints (ROS's default Zenoh config
    disables multicast scouting entirely, so this sidesteps DDS-style
    discovery pain by design). Confirmed working locally on the Pi; not
    pursued further once same-LAN multicast turned out to already work.
  - Recurring gotcha hit twice: the `ros2 daemon` caches the discovery
    config from whenever it was last started — switching
    `ROS_DISCOVERY_SERVER`/`RMW_IMPLEMENTATION`/`ZENOH_CONFIG_OVERRIDE`
    requires `ros2 daemon stop && ros2 daemon start` in the *same* shell
    before anything using that config will see the graph, including the
    launch script's own `ros2 lifecycle get` polling.
- End state: RViz on Thor successfully rendered `RobotModel` + `TF` + `Image`
  over plain LAN multicast, `Global Status: Ok`.

### Real wheel odometry in mserve_base

- User's direction: keep `mserve_drivechain` a pure motor driver (no
  kinematics) — already true in code (forward kinematics already lived in
  `mserve_base`, matching what the user wanted) but contradicted by
  `readme.md`'s Design Decisions section, which claimed the opposite. Fixed
  the doc; no code moved.
- Real, hardware-forcing discovery while building this: `position_rad` is
  **always 0** in the speed-loop mode `mserve_base` actually drives motors
  in — the DDSM115 protocol only reports position in position-control mode
  (`drivechain_uart.cpp`'s `parse_json_feedback`: *"not returned in
  speed-loop feedback"*). Caught this via live testing (drove the sim robot,
  `/odom` stayed at zero) rather than assuming the design would work.
  Redesigned from position-delta odometry (differencing an absolute
  position, with wraparound handling) to velocity-integration odometry
  (integrating `velocity_rads * dt` each tick) — the signal that's actually
  reliable in speed-loop mode. `mserve_base` also now self-integrates
  `left_wheel_angle`/`right_wheel_angle` the same way, purely for
  `/joint_states`, since `mserve_drivechain` can't provide real wheel angle
  either.
- Found and fixed two more real bugs in `mserve_drivechain` along the way:
  `velocity_rpm`/`velocity_rads` were sign-corrected for physically-reversed
  motors (`motor_signs`) but `position_rad` wasn't — live on this robot,
  since `motor_signs: [1, -1]`; and `DriveMotorFeedback.stamp` was never
  populated at all (always zero) — added a `ros_clock` blackboard entry
  (matching the existing `ros_logger` pattern) so `PublishMotorFeedback` can
  stamp it properly.
- New BT nodes `UpdateOdometry`/`PublishOdometry` in `mserve_base`, added to
  `drive_tree.xml` after the existing drive-command steps. Publishes
  `/odom` (`nav_msgs/Odometry`), broadcasts `odom -> base_link` TF, and
  publishes `/joint_states` — the last of which also fixes the
  `left_wheel_joint`/`right_wheel_joint` TF errors RViz was showing (those
  are `continuous` URDF joints; `robot_state_publisher` can't compute their
  transform at all without *some* `/joint_states` source, which nothing
  published before this).
- Verified end-to-end in `--sim`: drove straight (`/odom` position grew
  correctly, zero rotation), then rotated in place (correct wheel RPM signs,
  `/odom` orientation quaternion and `angular.z` matched the commanded rate,
  position stayed stable), `odom -> base_link` TF resolved via `tf2_echo`
  with continuously updating values, `/joint_states` positions accumulating
  correctly.

### Not done yet

- Camera frame rate still ~12.6Hz (see above) — not fixed this session.
- Mic (`audio_common`/GStreamer) still blocked — not attempted this session
  beyond confirming the blocker.
- Odometry covariance is left at zero (no real uncertainty estimate) — fine
  for RViz, but needs tuning before this feeds into Nav2/AMCL.
- Lidar bring-up (Phase 2 of `docs/continue.md`) not started.

### Odometry re-verified on real hardware

After the sim verification above, re-ran the same checks against the real
DDSM115 motors (`backend=hardware`, `/dev/ttyAMA0`). A short, bounded test
drive (`timeout`-limited `cmd_vel` burst, confirmed floor was clear first) —
forward at 0.15 m/s for ~2s — showed real-world detail sim can't produce:
`/odom` landed at `x≈0.148m` (less than the naive 0.15×2=0.3m, consistent
with the DDSM115's acceleration ramp — `motor_accel`, not instant — eating
into the short command window) with a small `y≈0.003m`/~1.6° yaw drift
(plausible small left/right motor response difference, exactly the kind of
real error dead-reckoning is honestly susceptible to — see the covariance
note above). Dead-man switch correctly zeroed the real motors after the
bounded publish ended (`velocity_rpm: 0`, no fault codes), `odom -> base_link`
TF matched `/odom` exactly and was stable at rest, and shutdown was clean
(no leftover processes, motors stopped). Odometry design is confirmed
correct on both sim and real hardware now.
