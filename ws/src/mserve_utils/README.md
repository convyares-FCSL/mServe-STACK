# mserve_utils

C++ helper utilities for the mServe ROS 2 stack.

This package is intended to hold shared support code for:

- configuration validation
- QoS profiles
- topic name helpers
- common parameter helpers
- unit tests for utility behavior

Current state:

- minimal CMake and package metadata
- placeholder C++ support files
- a basic unit test for configuration behavior

Usage:

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select mserve_utils --symlink-install
colcon test --packages-select mserve_utils --event-handlers console_direct+
```
