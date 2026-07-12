# Simulation and Hardware-in-Loop Plan

## What we have today

**Update:** Gazebo and RViz both now run on the NVIDIA Thor (not a PC/WSL2). The
"move to Thor" step below is done; the plan section is kept for history.

- A ROS 2 robot description package in `ws/src/mserve_description` (originally
  built against Jazzy; the Pi itself now runs natively on Lyrical — Thor's ROS
  distro for simulation is tracked separately).
- Gazebo Harmonic integration using `ros_gz_sim` and `ros_gz_bridge`.
- A `params/mserve_gazebo_bridge.yaml` bridge that connects:
  - `cmd_vel` ROS → Gazebo
  - `odom` Gazebo → ROS
  - `tf` Gazebo → ROS
  - `joint_states` Gazebo → ROS
  - `clock` Gazebo → ROS
- A top-level `launch/mserve_gazebo.launch.py` that starts:
  - Gazebo Sim
  - `robot_state_publisher`
  - the bridge
  - RViz
- A lightweight simulation world in `worlds/empty.sdf`.
- A working `teleop_twist_keyboard` path for manual drive testing.

## Current limitations (historical — resolved by the Thor move)

- Running Gazebo in WSL was choppy because WSL GPU support is limited.
- The Pi5 was never the ideal platform for heavy Gazebo simulation, and stays hardware-only.

## What's next

1. ~~Move Gazebo + ROS to the Nvidia Thor machine.~~ **Done** — Gazebo and RViz both now run on Thor.
2. Use VS Code Remote SSH for editing the code on Thor.
3. Refine the URDF/xacro model for the robot and sensors.
4. Add more realistic sensor simulation, then compare to the Pi5 hardware interface.

## Planned Hardware-in-Loop workflow

### Short-term goal

- Use Thor as the simulation host.
- Keep the Pi5 as the hardware/robot side.
- Run control and hardware interface nodes on the Pi5 if possible.
- Publish/subscribe the same ROS topics on both Thor and Pi5.

### Target setup

- **Thor**: Gazebo, simulation world, sensor models, `ros_gz_bridge`, `robot_state_publisher`, and any simulation-only nodes.
- **Pi5**: real hardware nodes, controllers, and any onboard logic.
- **Laptop**: remote code editing, diagnostics, RViz/visualization, and monitoring.

### Networking

- Use ROS 2 DDS discovery across machines.
- For a stable multi-machine setup, a VPN like Tailscale is a good option.
- Ensure all hosts can resolve each other and share the same ROS_DOMAIN_ID if needed.

## Longer-term goal

- Move toward Isaac Sim or another advanced simulator once the ROS network and HIL flow are stable.
- Keep the Pi5 as the hardware target and run simulations on the desktop/GPU host.
- Use the simulation environment to validate Pi5 control logic before hardware deployment.

## Quick practical advice

- Develop and test simulation on Thor.
- Use the Pi5 for hardware-in-loop validation and real robot tests.
- Keep the simulation and hardware sides separated by clear topic boundaries.
- Use `ros2 topic echo`, `ros2 node list`, and RViz to verify that both worlds see the same topics.
