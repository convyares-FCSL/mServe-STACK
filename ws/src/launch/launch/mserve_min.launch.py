import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory('mserve_interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'mserve_params.yaml')

    drivechain = Node(
        package='mserve_drivechain',
        executable='mserve_drivechain',
        name='drivechain',
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
        drivechain,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])
