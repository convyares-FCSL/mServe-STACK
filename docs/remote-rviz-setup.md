# Running RViz on Laptop with Simulation on NVIDIA Thor

This guide explains how to visualize the mServe simulation and ROS data in RViz on your development laptop while the simulation and all ROS nodes run on the NVIDIA Thor machine.

## Prerequisites

- **NVIDIA Thor**: Running mServe from `/home/ecm/Workspace/projects/mServe-STACK`
- **Your Laptop**: ROS 2 Jazzy installed
- **Network**: Tailscale/Tailshare tunnel OR same local network

## Thor Connection Info

- **Tailscale IP** (recommended): `100.110.87.8`
- **Local network IP**: Run on Thor: `hostname -I`

## Step 1: Start Discovery Server on Thor

SSH into Thor and start a Fast DDS discovery server:

```bash
ssh ecm@100.110.87.8

# On Thor, terminal 1:
source /opt/ros/jazzy/setup.bash
fastdds discovery -i 0 -l 0.0.0.0 -p 11888

# Leave this running
```

This is required for Tailscale use because plain ROS 2 DDS discovery relies on
multicast, and Tailscale does not carry that multicast traffic.

## Step 2: Start the Simulation on Thor

SSH into Thor and launch the simulation:

```bash
# On Thor, terminal 2:
export ROS_DISCOVERY_SERVER=127.0.0.1:11888
cd /home/ecm/Workspace/projects/mServe-STACK
scripts/05_utils/launch_mserve_description_gazebo.sh

# Leave this running — don't close the terminal
```

This is the preferred launch path on Thor because it:

- uses the real project path on Thor
- builds into `ws/build_host`, `ws/install_host`, and `ws/log_host`
- avoids stale `install/setup.bash` assumptions
- keeps the ROS nodes registered with the local discovery server on Thor

The terminal will show Gazebo startup messages and the bridge connecting topics. It's normal to see it pause briefly while loading.

If you want the manual equivalent on Thor:

```bash
cd /home/ecm/Workspace/projects/mServe-STACK/ws
source /opt/ros/jazzy/setup.bash
export ROS_DISCOVERY_SERVER=127.0.0.1:11888
colcon build --symlink-install --packages-select mserve_description \
  --build-base build_host \
  --install-base install_host \
  --log-base log_host
source install_host/setup.bash
ros2 launch mserve_description mserve_gazebo.launch.py
```

## Step 3: On Your Laptop — Quick Start

**Option A: Using the helper script (easiest)**

```bash
cd /home/ecm/ai-workspace/projects/mServe-STACK
./scripts/launch_remote_rviz.sh 100.110.87.8
```

**Option B: Manual setup**

```bash
# Set network environment
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=100.110.87.8:11888

# Source ROS and launch RViz
source /opt/ros/jazzy/setup.bash
rviz2
```

## Step 3: Configure RViz Display
## Step 4: Configure RViz Display

RViz starts with an empty scene. Add displays:

1. **Set Fixed Frame** (top toolbar):
   - Click the dropdown showing "No fixed frame"
   - Select `odom`

2. **Add transforms** (bottom-left, Add button → TF)
   - Shows robot coordinate frame tree

3. **Add robot model** (Add → RobotModel)
   - Displays the mServe URDF from Thor's `/robot_description`

4. **Add odometry** (Add → Odometry)
   - Plots the robot's path from `/odom` topic

5. **Optional: Add laser scan** (Add → LaserScan, topic: `/scan`)
   - Shows LIDAR data if enabled in URDF

## Step 5: Verify Connection

Check that RViz sees Thor's topics:

```bash
# In a new laptop terminal
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash

# Should list Thor's nodes
ros2 node list

# Should show Thor's topics
ros2 topic list

# Should show transform data
ros2 topic echo /tf | head -20
```

## Troubleshooting

### RViz Starts But No Data Appears

1. **Check Thor is running:**
   ```bash
   ssh ecm@100.110.87.8 "source /opt/ros/jazzy/setup.bash && export ROS_DISCOVERY_SERVER=127.0.0.1:11888 && source /home/ecm/Workspace/projects/mServe-STACK/ws/install_host/setup.bash && ros2 node list"
   ```

2. **Verify network settings on laptop:**
   ```bash
   echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
   echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
   echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
   ```

3. **Restart ROS 2 discovery:**
   ```bash
   ros2 daemon stop && sleep 1 && ros2 daemon start
   ros2 topic list  # Should show Thor's topics now
   ```

4. **Check the discovery server is reachable from laptop:**
   ```bash
   nc -vz 100.110.87.8 11888
   ```

### Firewall Issues

If topics don't appear, check firewall and TCP reachability to the discovery server:

```bash
# On Thor and laptop (Linux), allow the discovery server port
sudo ufw allow 11888/tcp

# Optional for same-LAN multicast DDS use:
sudo ufw allow in 7400:7409/udp
```

### Still No Connection?

Use SSH tunnel as fallback:

```bash
# On laptop, create tunnel
ssh -L 11888:localhost:11888 ecm@100.110.87.8

# In a new terminal on laptop:
export ROS_DISCOVERY_SERVER=localhost:11888
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2
```

## Saving Your RViz Config

Once displays are set up correctly:

1. **File → Save Config As** → `mserve_remote.rviz`
2. Next time, use: `./scripts/launch_remote_rviz.sh 100.110.87.8 mserve_remote.rviz`

## Network Reference

| Connection | IP | Use Case |
|-----------|-----|----------|
| Tailscale | 100.110.87.8 | Recommended (works over internet) |
| Local network | See `hostname -I` on Thor | If on same WiFi/Ethernet |

## Next: Teleoperation from Laptop

Once RViz displays the robot:

```bash
# On laptop, in a new terminal
export ROS_LOCALHOST_ONLY=0
source /opt/ros/jazzy/setup.bash

# Launch keyboard teleop — sends /cmd_vel to Thor
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Press arrow keys to move the robot
# Watch RViz update in real-time
```
