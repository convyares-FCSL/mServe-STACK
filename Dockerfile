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
    && rm -rf /var/lib/apt/lists/*
