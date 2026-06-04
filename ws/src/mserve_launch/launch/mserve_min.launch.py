import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory('mserve_interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'mserve_params.yaml')

    base_node = Node(
        package='mserve_base',
        executable='base_node',
        name='mserve_base',
        output='screen',
        parameters=[params_file]
    )

    drivechain_node = Node(
        package='mserve_drivechain',
        executable='drivechain_node',
        name='mserve_drivechain',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='mserve_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen'
    )

    return LaunchDescription([
        base_node,
        drivechain_node,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])
