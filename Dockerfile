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
        # mserve_camera (wraps v4l2_camera's V4l2CameraDevice directly) +
        # its hand-rolled JPEG compression (cv::cvtColor/cv::imencode).
        # mserve_lidar needs no extra apt package — its RPLIDAR SDK is
        # vendored source, built as part of the package itself.
        ros-jazzy-v4l2-camera \
        libopencv-dev \
        # MJPEG transcode for camera.html/base.html <img> tags
        ros-jazzy-web-video-server \
        # Foxglove Bridge, ws://<pi-ip>:8765 (--foxglove)
        ros-jazzy-foxglove-bridge \
        # slam_toolbox (vendored source, ws/src/third_party/slam_toolbox,
        # pre-patched — see that dir's mServe: comments) build/runtime deps.
        # --slam-map/--slam-local only; not needed for the base stack.
        libceres-dev \
        libsuitesparse-dev \
        libomp-dev \
        qtbase5-dev \
        ros-jazzy-rviz-common \
        ros-jazzy-rviz-default-plugins \
        ros-jazzy-rviz-ogre-vendor \
        ros-jazzy-rviz-rendering \
        ros-jazzy-bondcpp \
        ros-jazzy-bond \
        ros-jazzy-interactive-markers \
        ros-jazzy-tf2-sensor-msgs \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-message-filters \
        ros-jazzy-pluginlib \
        # slam_toolbox's package.xml declares nav2_map_server as an
        # exec_depend (map *serving*, not the SLAM/mapping this stack
        # actually uses — save_map/serialize_map are slam_toolbox's own
        # services, independent of it — see ws/src/third_party/README.md's
        # slam_toolbox section) but colcon's scoped --packages-select build
        # still needs it resolvable to build slam_toolbox at all. The apt
        # binary satisfies that without vendoring/building all 18 Nav2
        # packages from source just for this.
        ros-jazzy-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*
