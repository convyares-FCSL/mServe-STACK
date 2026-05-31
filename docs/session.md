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
