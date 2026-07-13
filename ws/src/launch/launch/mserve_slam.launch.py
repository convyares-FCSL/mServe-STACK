import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory('interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'slam_toolbox_params.yaml')

    # slam_toolbox's own node is already a lifecycle node (same pattern as
    # every mserve_* node). Launched as a plain Node (not launch_ros's
    # LifecycleNode wrapper) because configure/activate/shutdown are driven
    # externally via service calls below, not by launch itself.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file],
    )

    # Reuses the same BT-based lifecycle_manager that drives configure/
    # activate/graceful-shutdown for drivechain/base/camera/lidar in
    # mserve_min.launch.py — pointed at a separate slam_bringup.xml/
    # slam_shutdown.xml pair (see ws/src/lifecycle_manager/src/trees/) so it
    # only ever touches slam_toolbox, and given a distinct node name so it
    # doesn't collide with the always-running instance. This replaces the
    # previous plain launch_ros ChangeState event handlers, which had no
    # shutdown path at all — Ctrl+C just SIGTERM'd slam_toolbox with no
    # deactivate/cleanup transition. Kept as its own launch file (not folded
    # into mserve_min.launch.py) because mapping is an occasional, opt-in
    # session, not something you want running on every boot.
    slam_lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='slam_lifecycle_manager',
        output='screen',
        parameters=[{
            'bringup_tree_file': 'slam_bringup.xml',
            'shutdown_tree_file': 'slam_shutdown.xml',
        }],
    )

    return LaunchDescription([
        slam_toolbox_node,
        slam_lifecycle_manager,
    ])
