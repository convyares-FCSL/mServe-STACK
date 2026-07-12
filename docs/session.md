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
