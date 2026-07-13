# Hybrid RViz and Teleop from WSL with Gazebo on NVIDIA Thor

> **Superseded — see [`remote-rviz-zenoh.md`](remote-rviz-zenoh.md) for the
> current procedure.** This guide's premise — Gazebo on Thor, RViz on a WSL
> laptop, Fast DDS discovery server — no longer matches the current setup:
> RViz now also runs directly on Thor alongside Gazebo, so there's no WSL leg
> for the DDS traffic to cross anymore, and the current stack uses a Zenoh
> router instead of a Fast DDS discovery server. Kept for reference only.

This guide covers the (now historical) hybrid setup:

- **Thor** runs Gazebo, ROS 2 nodes, `robot_state_publisher`, and `ros_gz_bridge`.
- **Windows laptop / WSL** runs RViz and keyboard teleop.
- ROS 2 traffic crosses the network through Fast DDS discovery.

This is a good target now that Gazebo is working on Thor. The main thing to get right is WSL networking: discovery can work while topic data still fails if Thor cannot route back to the address WSL advertises.

## Chance of Success

High, if WSL has a reachable network address.

- **Best path:** install / run Tailscale inside WSL, so WSL has its own `100.x.x.x` address.
- **Also good:** same LAN with WSL mirrored networking enabled.
- **Risky path:** Tailscale only on Windows host while ROS runs inside WSL. ROS nodes may appear, but topic data can fail because WSL advertises an internal NAT address.

## Network Targets

On Thor:

```bash
hostname -I
tailscale ip -4
```

Use Thor's Tailscale IP if WSL also has Tailscale. Current known Thor Tailscale IP:

```text
100.110.87.8
```

On WSL, check that WSL itself has a Tailscale IP:

```bash
tailscale ip -4
```

If that command does not return a `100.x.x.x` address, install / start Tailscale inside WSL or use same-LAN/mirrored networking instead.

## Step 1: Start Fast DDS Discovery on Thor

On Thor, terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

fastdds discovery -i 0 -l 0.0.0.0 -p 11888
```

Leave this running.

Why: Tailscale and WSL do not reliably carry DDS multicast discovery. The discovery server gives all ROS participants a known unicast meeting point.

## Step 2: Start Gazebo and ROS on Thor

On Thor, terminal 2:

```bash
cd /home/ecm/Workspace/projects/mServe-STACK
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=127.0.0.1:11888

scripts/sim/launch_mserve_description_gazebo.sh launch_rviz:=false
```

Use `launch_rviz:=false` because RViz will run on WSL.

The Gazebo helper script builds `mserve_description`, sources `ws/install_host/setup.bash`, starts Gazebo, starts `robot_state_publisher`, and starts the ROS/Gazebo bridge.

## Step 3: Configure WSL ROS Environment

On WSL:

```bash
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=100.110.87.8:11888

ros2 daemon stop
ros2 daemon start
```

If using Thor's LAN IP instead of Tailscale, replace `100.110.87.8` with Thor's LAN IP.

## Step 4: Verify WSL Can See Thor

On WSL:

```bash
ros2 node list
ros2 topic list
```

Expected nodes include things like:

```text
/robot_state_publisher
/mserve_gz_bridge
```

Expected topics include:

```text
/robot_description
/tf
/tf_static
/joint_states
/odom
/cmd_vel
/clock
```

Then test data, not just discovery:

```bash
ros2 topic echo /odom --once
ros2 topic echo /robot_description --once
```

If `ros2 node list` works but `ros2 topic echo` hangs, that is usually a WSL reachability problem. Prefer Tailscale inside WSL, not only on Windows.

## Step 5: Launch RViz on WSL

If this repo is also checked out in WSL:

```bash
cd /home/ecm/Workspace/projects/mServe-STACK
./scripts/launch_remote_rviz.sh 100.110.87.8 ws/src/mserve_description/rviz/mserve.rviz
```

If the repo is not checked out in WSL, just start RViz:

```bash
rviz2
```

In RViz:

- Fixed Frame: `base_link` or `odom`
- Add `RobotModel`
- Set RobotModel description source to `Topic`
- Description Topic: `/robot_description`
- Add `TF`
- Add `Odometry` on `/odom`

## Step 6: Run Teleop from WSL

Install teleop if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard
```

Run:

```bash
cd /home/ecm/Workspace/projects/mServe-STACK
./scripts/launch_remote_teleop.sh 100.110.87.8
```

Keep keyboard focus in the teleop terminal. Watch the robot move in Gazebo on Thor and update in RViz on WSL.

## Verify Teleop Reaches Thor

On Thor, in another terminal with the same ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ecm/Workspace/projects/mServe-STACK/ws/install_host/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=127.0.0.1:11888

ros2 topic echo /cmd_vel
```

While pressing teleop keys in WSL, Thor should print `Twist` messages.

You can also check the Gazebo side:

```bash
GZ_IP=127.0.0.1 gz topic -e -t /cmd_vel
```

## Troubleshooting

### WSL sees no ROS nodes

Check discovery server reachability:

```bash
nc -vz 100.110.87.8 11888
```

Also verify both sides use the same environment:

```bash
echo "$RMW_IMPLEMENTATION"
echo "$ROS_DOMAIN_ID"
echo "$ROS_LOCALHOST_ONLY"
echo "$ROS_DISCOVERY_SERVER"
```

### WSL sees nodes but no topic data

This is the classic WSL DDS data-plane issue.

Fix options:

1. Run Tailscale inside WSL and use WSL's own Tailscale address.
2. Use Windows/WSL mirrored networking and same-LAN IPs.
3. Avoid WSL networking for RViz and run RViz directly on a Linux machine.

### RViz starts but the robot does not appear

Check:

```bash
ros2 topic echo /robot_description --once
ros2 topic echo /tf_static --once
ros2 topic echo /joint_states --once
```

If `/robot_description` works but TF does not, restart RViz after Thor is already running.

### Teleop publishes but Gazebo does not move

Check ROS side on Thor:

```bash
ros2 topic echo /cmd_vel
```

Check bridge side on Thor:

```bash
GZ_IP=127.0.0.1 gz topic -e -t /cmd_vel
```

If ROS sees `/cmd_vel` but Gazebo does not, inspect the bridge node:

```bash
ros2 node list
ros2 topic list | grep cmd_vel
```

## Quick Commands

Thor terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
fastdds discovery -i 0 -l 0.0.0.0 -p 11888
```

Thor terminal 2:

```bash
cd /home/ecm/Workspace/projects/mServe-STACK
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=127.0.0.1:11888
scripts/sim/launch_mserve_description_gazebo.sh launch_rviz:=false
```

WSL RViz terminal:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=100.110.87.8:11888
rviz2
```

WSL teleop terminal:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=100.110.87.8:11888
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
