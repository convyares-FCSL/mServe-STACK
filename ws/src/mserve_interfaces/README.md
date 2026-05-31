# mserve_interfaces

Shared ROS 2 interfaces and configuration for the mServe stack.

This package contains:

- ROS messages: `DriveStatus`, `WheelCommand`, `WheelFeedback`, `DisplayStatus`, `Esp32Status`
- ROS service: `SetDisplayMode`
- ROS action: `Dock`
- Shared parameter and topic configuration in `config/mserve_params.yaml`

Usage:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_interfaces --symlink-install
source install/setup.bash
ros2 interface show mserve_interfaces/msg/WheelCommand
```
