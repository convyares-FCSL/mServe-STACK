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


    low_booster_node = Node(
        package='hyfleet_booster',
        executable='booster_node',
        name='low_booster',
        output='screen',
        parameters=[params_file]
    )

    high_booster_node = Node(
        package='hyfleet_booster',
        executable='booster_node',
        name='high_booster',
        output='screen',
        parameters=[params_file]
    )

    compressor_node = Node(
        package='hyfleet_compressor',
        executable='compressor_node',
        name='hyfleet_compression',
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
        low_booster_node,
        high_booster_node,
        compressor_node,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])
