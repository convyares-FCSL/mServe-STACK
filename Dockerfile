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
        # robot_description / TF (mserve_min.launch.py's robot_state_publisher,
        # needed even with camera/lidar disabled — see launch/mserve_min.launch.py)
        ros-jazzy-xacro \
        # lifecycle_manager (BT.CPP bringup/shutdown trees) + vendored
        # behaviortree_ros2 (ws/src/third_party/BehaviorTree.ROS2) build deps
        ros-jazzy-generate-parameter-library \
        libboost-dev \
    && rm -rf /var/lib/apt/lists/*
