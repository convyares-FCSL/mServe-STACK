# Transfer / Platform-Revert Runbook

**Why this file exists:** we're moving the Pi's boot drive from the current
SD card (native ROS 2 Lyrical on Ubuntu 26.04) to a separately-flashed SSD
(Raspberry Pi OS, fresh install) and reverting to Docker + ROS 2 Jazzy. That
swap ends whatever Claude Code / shell session is running on the SD card, and
a fresh OS has no memory of any of this. This file is git-tracked so it
travels with the repo onto the new drive and survives the swap. Read it first
in any new session picking this work back up.

**Do not delete the SD card / old install.** It stays physically untouched as
a known-working fallback until the new setup is verified end-to-end.

## Where things stand

- SSD has already been formatted and had Raspberry Pi OS installed on a
  **separate machine** (not this Pi) — so Phase A/B of flashing-in-place is
  done differently than originally planned; the SSD arrives pre-imaged.
- Plan is: (1) move the repo/work over via USB, (2) physically switch the
  Pi's boot drive to the SSD, (3) confirm the new OS boots and is reachable,
  (4) move any remaining data back / reconcile, (5) test current
  functionality against the checklist below, (6) port everything to Docker +
  Jazzy on the new OS.

## Step-by-step

1. **Copy the repo onto USB.** From the current SD-card system (this one),
   copy `~/mServe-STACK` (the whole repo, including this file, `Dockerfile`,
   `docker-compose.yml`, and any uncommitted local changes) onto a USB drive.
   Prefer `git status`-clean state; if there are uncommitted changes worth
   keeping, commit or stash them first so nothing is silently lost in the
   copy.
2. **Physically swap the boot drive** to the SSD (already imaged with
   Raspberry Pi OS on the separate machine).
3. **First boot on the new OS** — verify before doing anything else:
   - `cat /etc/os-release` — confirm Raspberry Pi OS / Bookworm, not still
     Ubuntu.
   - `hostname`, network connectivity, SSH reachable.
   - Set up **Raspberry Pi Connect** for remote access instead of Tailscale
     (`sudo apt install rpi-connect && rpi-connect signin`).
4. **Bring the repo over** from USB into the new OS's home directory
   (`~/mServe-STACK`), or `git clone` fresh if it's pushed somewhere — either
   way, end up with a full working copy including this `transfer.md`.
5. **Enable the GPIO-header UART** for the DDSM Driver HAT via
   `sudo raspi-config` → Interface Options → Serial Port → hardware serial
   **on**, login shell over serial **off** (see
   `ws/src/mserve_drivechain/README.md`, "One-time Pi setup" section).
   Confirm `/dev/ttyAMA0` exists.
6. **Install Docker**: `curl -fsSL https://get.docker.com | sh`, then
   `sudo usermod -aG docker $USER` (recipe already documented in
   `ws/src/mserve_drivechain/README.md`'s Docker-fallback section).
7. **Test current functionality** against the checklist below, first via the
   existing native path if still feasible, then increasingly via
   `docker compose build && ./scripts/run_stack.sh` (already has a working
   Docker branch for drivechain/base/rosbridge/web UI — see "Known gaps"
   below for what still needs porting).

## What works today (native Ubuntu 26.04 Lyrical, SD card) — verify parity against this after porting

Snapshot as of 2026-07-18, from `CLAUDE.md` and recent commit history
(`d94e965 Fix SLAM map generation end-to-end`, `1a97c24 Fix launch
package.xml regression`, `a7fe587 added behavior tree for slam`,
`88973bb slam working`, `7bad277 Add compressed camera image, Foxglove
bridge`):

- [ ] **Drivechain**: `mserve_drivechain` talks to the ESP32 (DDSM Driver
      HAT) over `/dev/ttyAMA0` (Pi GPIO14/15, header pins 8/10), JSON over
      UART, drives 2x DDSM115 hub motors. Lifecycle-managed, BehaviorTree.CPP
      driven.
- [ ] **Base**: `mserve_base` — `/cmd_vel` → safety clamp → `/mserve/cmd_vel_safe`
      → drivechain. Also lifecycle + BT driven.
- [ ] **Camera**: compressed image topic + Foxglove bridge working
      (`mserve_camera`, wraps `v4l2_camera`'s `V4l2CameraDevice`).
- [ ] **Lidar**: `mserve_lidar` — custom driver against the SDK directly (not
      the `rplidar_ros` apt package).
- [ ] **SLAM**: mapping AND localization both confirmed working end-to-end as
      of the most recent commit (fixed a lidar scan-size bug, a launch params
      bug, and a `serialize_map` Boost bug). Run via
      `./scripts/run_stack.sh --slam-map` or `--slam-local`.
- [ ] **Web UI**: `http://<pi-ip>:6240/drivechain.html` and `base.html`,
      served natively via `python3 -m http.server` regardless of Docker/
      native mode.
- [ ] **rosbridge**: port 9090.
- [ ] **Foxglove Bridge**: `./scripts/run_stack.sh --foxglove` → `ws://<pi-ip>:8765`.
- [ ] **Zenoh remote-RViz**: native-only today, used for viewing RViz from
      another machine (e.g. the Thor) over the network.
- [ ] **Gazebo/RViz sim on the Thor**: `mserve_description` URDF, recently
      unblocked after a `package.xml` regression fix — not Pi-local, but
      confirm the Pi side of whatever it depends on still works after this
      migration.
- [ ] **`--sim` mode**: `./scripts/run_stack.sh --sim` (no hardware needed) —
      good first smoke test on any new setup before testing real hardware.

## Known gaps to port into Docker (not a regression — these were native-only even before the original Docker→native migration)

Per `scripts/run_stack.sh`, the existing Docker branch only covers
**drivechain + base + rosbridge + web_video_server + web UI**. These are
explicitly skipped in Docker mode today and need new work, not just
restoring:

- Camera (`mserve_camera`, needs `ros-jazzy-v4l2-camera` + `/dev/video*`
  device passthrough in `docker-compose.yml`)
- Lidar (`mserve_lidar`, needs whatever its SDK requires + device passthrough)
- SLAM Toolbox (`ros-jazzy-slam-toolbox`)
- Foxglove Bridge (`ros-jazzy-foxglove-bridge`)
- Zenoh remote-RViz (likely needs `network_mode: host` in Compose instead of
  the default bridge network, since DDS/Zenoh discovery/multicast doesn't
  traverse Docker's bridge networking cleanly — needs a real decision, not
  copy-paste)

Also worth fixing while touching `docker-compose.yml`: its `MSERVE_WEB_PORT`
default (`5002`) doesn't match the actual web UI port (`6240`, always served
natively) — stale/dead config.

## Still open / not yet solved

- **The ELEGOO 3.5" touchscreen driver** (the original motivating problem —
  `dtoverlay=tft35a`, `goodtft/LCD-show`) is more likely to work on Raspberry
  Pi OS than it was on Ubuntu Lyrical, but **not guaranteed** — Pi 5's RP1 I/O
  chip is different enough from earlier Pi SoCs that legacy fbtft overlays
  are hit-or-miss even on Raspberry Pi OS. Treat as a real test once the base
  system is up, not an assumed win.
- **No systemd unit is checked into git.** The current live one on the SD
  card (`/etc/systemd/system/mserve-drivechain.service`, currently disabled)
  sources native Lyrical and has no `Requires=docker.service`. A
  Docker-aware version needs to be created on the new OS and this time
  actually committed to the repo (e.g. under a `systemd/` directory) so it
  isn't lost again on the next platform change.
