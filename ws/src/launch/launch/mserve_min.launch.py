import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory('interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'mserve_params.yaml')

    backend_arg = DeclareLaunchArgument(
        'backend', default_value='hardware',
        description="mserve_drivechain backend: 'hardware' or 'sim'"
    )
    uart_device_arg = DeclareLaunchArgument(
        'uart_device', default_value='/dev/ttyAMA0',
        description='UART device used by the hardware backend'
    )

    drivechain = Node(
        package='mserve_drivechain',
        executable='drivechain_node',
        name='mserve_drivechain',
        output='screen',
        parameters=[params_file, {
            'drive.backend': LaunchConfiguration('backend'),
            'hardware.uart_device': LaunchConfiguration('uart_device'),
        }]
    )

    base = Node(
        package='mserve_base',
        executable='base_node',
        name='mserve_base',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen'
    )

    return LaunchDescription([
        backend_arg,
        uart_device_arg,
        drivechain,
        base,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])
