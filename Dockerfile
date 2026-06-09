FROM ros:jazzy-ros-base

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        # Build tools
        python3-colcon-common-extensions \
        # ROS packages needed by mserve_drivechain
        ros-jazzy-behaviortree-cpp \
        ros-jazzy-rclcpp-lifecycle \
        ros-jazzy-lifecycle-msgs \
        ros-jazzy-ament-index-cpp \
        ros-jazzy-geometry-msgs \
        # Web bridge
        ros-jazzy-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*
