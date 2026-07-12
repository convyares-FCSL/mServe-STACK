ssh ecm@172.16.68.73


I'm working on mServe — a ROS 2 differential-drive robot (Raspberry Pi 5 +
DDSM115 hub motors via a Waveshare ESP32 driver HAT). Repo: mServe-STACK.
I'm on Thor now (dev machine + RViz), the robot's compute is the Pi 5.

Current confirmed state (as of 2026-07-12):
- Pi 5 runs ROS 2 Lyrical NATIVELY (no Docker — migrated off Docker this
  session). Static Wi-Fi IP 172.16.68.73, also on Tailscale at
  100.122.150.74. Repo lives at /home/ecm/mServe-STACK on the Pi.
- Real hardware confirmed working: mserve_base -> mserve_drivechain ->
  JSON/UART (/dev/ttyAMA0) -> ESP32 -> both DDSM115 motors. Drove it from
  the web UI (scripts/run_stack.sh, port 6240) today.
- mserve-drivechain.service (systemd) is currently DISABLED — I turned off
  auto-start to do active dev. Run ./scripts/run_stack.sh manually on
  the Pi when you need the drive stack up; `sudo systemctl enable --now
  mserve-drivechain` restores auto-start later once this phase is done.
- Known CMake gotcha on this distro: ament_target_dependencies() was
  REMOVED in Lyrical (not deprecated — gone). Any new C++ package must use
  target_link_libraries() with modern imported targets (rclcpp::rclcpp,
  <pkg>::<pkg> for message packages) — see ws/src/mserve_base/CMakeLists.txt
  for the pattern already fixed this session.
- ws/src/mserve_description/urdf/ already has stub files:
  mserve_camera.xacro, mserve_lidar.xacro, mserve_depth_camera.xacro,
  mserve_core.xacro — check what's actually wired into mserve.urdf.xacro
  vs. still a stub before assuming either way.
- Design philosophy (docs/architecture.md): avoid ros2_control and heavy
  Xacro for now — hand-written, readable control/description code, same
  style as the existing base/drivechain packages.
- Gazebo + RViz both run on Thor (not a PC/WSL2, not the Pi — Pi has no
  compute GPU). docs/remote-rviz-setup.md documents a Thor<->WSL hybrid DDS
  discovery-server setup that's now partly superseded (RViz moved fully to
  Thor) but the discovery-server/Tailscale networking pattern in it is
  still the right reference for getting Thor's RViz to see the Pi's ROS
  graph over the network.

Do this in three phases, in order — don't start phase N+1 until phase N is
verified working:

## Phase 1: Camera -> RViz

1. SSH to the Pi and identify the actual camera hardware (model, interface
   — USB/CSI/other). Don't assume; check what's physically connected.
2. Find/install the right native ROS 2 Lyrical driver package for it
   (e.g. usb_cam, v4l2_camera, libcamera, or a vendor SDK/driver if it's a
   depth camera) — same apt-search-then-install approach used this session
   for ros-lyrical-behaviortree-cpp/rosbridge-server.
3. Wire a camera link into the URDF (mserve_camera.xacro is already
   stubbed — check/finish it) with correct TF relative to base_link.
4. Launch robot_state_publisher + the camera driver node on the Pi.
5. On Thor: get RViz to see the Pi's ROS graph over the network (shared
   ROS_DOMAIN_ID, Tailscale or LAN — reference docs/remote-rviz-setup.md
   for the discovery-server pattern, adapt from WSL to Thor-as-client).
6. Verify: Image display shows the live feed in RViz, TF tree places the
   camera frame correctly relative to base_link/odom.

## Phase 2: Lidar

Same pattern as camera: identify hardware -> native Lyrical driver ->
wire mserve_lidar.xacro into the URDF with correct TF -> launch on Pi ->
verify LaserScan/PointCloud2 renders correctly in RViz on Thor, positioned
sensibly relative to the camera and base_link.

## Phase 3: Xbox controller (teleop)

1. Figure out where the controller physically connects (likely Thor, as
   the operator machine) and get it recognized (joy node).
2. Use teleop_twist_joy (or hand-written mapping, consistent with this
   project's "hand-write it first" philosophy) to publish /cmd_vel.
3. /cmd_vel needs to reach the Pi over the network — mserve_base already
   subscribes to /cmd_vel and clamps it (see interfaces/config/topics.yaml
   for the exact topic names/QoS mserve_base expects), so no new safety
   code should be needed there, just get the DDS traffic across reliably.
4. Add a deadman/safety check on the joy mapping before wiring it live to
   real motors.
5. Verify: moving the stick drives the real robot, releasing it stops it
   (mserve_base's command-timeout zeroing should already cover a
   controller disconnect, but confirm it does).

Start by SSHing to the Pi and confirming the current state matches what's
described above (service disabled, repo state, what's actually in
mserve.urdf.xacro) before changing anything.