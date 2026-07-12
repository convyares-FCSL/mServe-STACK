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
  web UI; the actual `run_drivechain_hw.sh` serves on port 6240.
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
- `run_drivechain_hw.sh` simplified per request: no longer spawns nodes or
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
