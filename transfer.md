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
- The USB stick used for this transfer (label `7571-6018`) already had, from
  before this session:
  - `backup-2026-07-12/` — a full backup of `/home/ecm` taken during the
    *original* Docker→native migration, including a `MIGRATION-README.md`
    that documents exactly how the Docker/Jazzy setup used to be restored.
    This turned out to be extremely useful — see below.
  - `Robot/mServe-STACK` — an older, separate copy (Jul 12, ~13:38), not the
    same as `backup-2026-07-12/mServe-STACK` (Jun 10 sources). Left as-is,
    not touched.
  - This session's fresh copy of the current repo (post-SLAM-fix, as of
    2026-07-18) was placed in `New/mServe-STACK-2026-07-18.tar.gz` (tarball,
    not a raw copy — the drive is `vfat`/FAT32, which doesn't preserve Unix
    permissions or symlinks, so a raw copy would silently strip the
    executable bit off scripts like `run_stack.sh`).
- Plan is: (1) move the repo/work over via USB — **done**, (2) physically
  swap the Pi's boot drive to the SSD, (3) confirm the new OS boots and is
  reachable, (4) extract the tarball and restore `.env`/Grafana from the old
  backup (see below), (5) test current functionality against the checklist
  below, (6) port everything to Docker + Jazzy on the new OS.

## Step-by-step

1. ~~Copy the repo onto USB~~ — **done**: `New/mServe-STACK-2026-07-18.tar.gz`
   on the USB stick, built via `tar czf`, preserves permissions/symlinks
   regardless of the FAT32 intermediate filesystem. Includes the `.env` file
   (gitignored but present locally, and needed by `docker-compose.yml`'s
   `env_file: .env`) and this `transfer.md`. Excludes `ws/build`,
   `ws/install`, `ws/log` (gitignored colcon artifacts, ~872MB, rebuilt fresh
   on the new OS anyway).
2. **Physically swap the boot drive** to the SSD (already imaged with
   Raspberry Pi OS on the separate machine).
3. **First boot on the new OS** — verify before doing anything else:
   - `cat /etc/os-release` — confirm Raspberry Pi OS / Bookworm, not still
     Ubuntu.
   - `hostname`, network connectivity, SSH reachable.
   - Set up **Raspberry Pi Connect** for remote access instead of Tailscale
     (`sudo apt install rpi-connect && rpi-connect signin`).
4. **Extract the repo** from USB: `tar xzf New/mServe-STACK-2026-07-18.tar.gz
   -C ~/` → gives a full working copy at `~/mServe-STACK`, including
   `transfer.md`, `.env`, `Dockerfile`, `docker-compose.yml`.
5. **Enable the GPIO-header UART** for the DDSM Driver HAT. Per the recovered
   `MIGRATION-README.md`, on the old Pi OS install this needed, in
   `/boot/firmware/config.txt`:
   ```
   dtparam=uart0=on
   dtoverlay=disable-bt
   ```
   and `sudo systemctl disable --now serial-getty@ttyAMA0.service` (the
   serial console must not be holding the port). Reboot for the overlay to
   take effect, then confirm `ls -l /dev/ttyAMA0`. (`raspi-config` → Interface
   Options → Serial Port is the equivalent guided path, per
   `ws/src/mserve_drivechain/README.md`.)
6. **Install Docker** — use the exact proven recipe from
   `MIGRATION-README.md` rather than the generic `get.docker.com` script:
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install -y docker.io docker-compose-plugin
   sudo usermod -aG docker,dialout $USER
   # log out/in (or `newgrp docker`) for the group change to take effect
   ```
7. **Restore Grafana + Loki + Promtail** — this existed on the pre-migration
   setup and was never carried over to native (not mentioned anywhere in
   `CLAUDE.md` today). It's a separate `~/grafana` folder/compose stack, not
   part of `mServe-STACK` itself:
   ```bash
   cp -r /media/$USER/7571-6018/backup-2026-07-12/grafana ~/
   cd ~/grafana
   docker compose up -d
   docker compose -f docker-compose.loki.yml up -d
   ```
   Grafana: `http://<pi-ip>:3030` (admin/admin unless changed). Loki: `:3100`.
   Note per the old README: `grafana/data/` and `grafana/loki/data/`
   (dashboards/logs) were never committed to git, only exist in this backup,
   and there was one uncommitted local edit to
   `grafana/loki/promtail-config.yml` at backup time — check that survived
   the copy.
8. **Recreate the systemd unit** — recovered verbatim from
   `MIGRATION-README.md` (the pre-migration version, which the current native
   unit on the SD card had rewritten to drop `Requires=docker.service`).
   Adapted here to the current script name (`scripts/run_stack.sh`, renamed
   from the old `web/run_drivechain_hw.sh` per `docs/TODO.md`):
   ```bash
   sudo tee /etc/systemd/system/mserve-drivechain.service > /dev/null <<'EOF'
   [Unit]
   Description=mServe drivechain + base (hardware)
   Requires=docker.service
   After=docker.service network-online.target
   Wants=network-online.target

   [Service]
   Type=simple
   User=ecm
   Group=ecm
   WorkingDirectory=/home/ecm/mServe-STACK
   ExecStart=/home/ecm/mServe-STACK/scripts/run_stack.sh
   Restart=on-failure
   RestartSec=5
   TimeoutStartSec=120

   [Install]
   WantedBy=multi-user.target
   EOF
   sudo systemctl daemon-reload
   sudo systemctl enable --now mserve-drivechain.service
   ```
   This time, actually commit this unit file into the repo (e.g. under
   `systemd/mserve-drivechain.service`) so it isn't lost again on the next
   platform change.
9. **Test current functionality** against the checklist below, first via
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

## Other things found in the USB backup (reference, not required)

- `backup-2026-07-12/mServe-STACK/scripts/03_packages/docker_build_workspace.sh`,
  `scripts/06_utils/docker_launch_mserve.sh`, `scripts/06_utils/docker_webbridge.sh`
  — older, pre-consolidation helper scripts for the Docker path. The current
  `scripts/run_stack.sh` already reimplements this logic in one place (see
  "Known gaps" above) — these are just historical reference if something in
  the new consolidated script doesn't match old behavior.
- `backup-2026-07-12/` also has full backups of `mbot-web/`, `robot-stack/`,
  and `ros2-systems-operability/` — unrelated to this revert, not touched,
  left on the USB in case they're needed later.

## Resolved (was "Still open / not yet solved")

- **The ELEGOO 3.5" touchscreen driver — works.** The original motivating
  problem for this whole platform revert, confirmed working on Pi 5's RP1
  chip (not a given going in). `dtoverlay=tft35a` from `goodtft/LCD-show`
  doesn't exist as a package for this kernel; the mainline `fbtft` overlay
  (generic `ili9486` preset + correct pin mapping) loads the driver cleanly
  but the screen stays white — the panel needs a manufacturer-specific init
  sequence (gamma/power-control registers) that the generic driver's
  built-in default doesn't send, and the `fbtft` overlay mechanism can't
  carry a custom init string via config.txt at all. Fix: pulled ELEGOO's
  actual precompiled overlay (`usr/tft35a-overlay.dtb` from
  `github.com/goodtft/LCD-show` — ELEGOO's own manual points at this
  upstream, their bundled driver predates Pi 4+ compatibility), decompiled
  it to confirm the pin mapping (reset=GPIO25, dc=GPIO24, touch irq=GPIO17 —
  matched what was already tried) had been right all along, then installed
  the compiled overlay itself (`/boot/firmware/overlays/tft35a.dtbo`,
  `dtoverlay=tft35a,rotate=90` in config.txt) so the correct init ships as
  compiled data instead of a config.txt param. Verified end-to-end: a 4-band
  RGB565 test pattern written directly to `/dev/fb0` displayed with correct
  colors in the right order, and `evtest` on the ADS7846 touch input device
  showed clean, coherent X/Y tracking during an actual touch drag — both
  display and touch confirmed working, not just driver-loaded.
- **Systemd unit — committed and running.** `systemd/mserve-drivechain.service`
  is checked into the repo (`Requires=docker.service`, unlike the old
  native-Lyrical one that had none), enabled via `systemctl enable --now`,
  and verified actually reaching `active` with drivechain/base up.
