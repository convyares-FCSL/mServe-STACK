import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interfaces_share = get_package_share_directory('interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'mserve_params.yaml')

    description_share = Path(get_package_share_directory('mserve_description'))
    xacro_file = description_share / 'urdf' / 'mserve.urdf.xacro'
    # launch_ros tries to parse a plain Command(...) result as YAML by
    # default, which breaks on URDF/XML content — ParameterValue forces it
    # to be treated as a plain string instead.
    robot_description = ParameterValue(Command(['xacro', ' ', str(xacro_file)]), value_type=str)

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

    camera = Node(
        package='mserve_camera',
        executable='camera_node',
        name='mserve_camera',
        output='screen',
        parameters=[params_file]
    )

    lidar = Node(
        package='mserve_lidar',
        executable='lidar_node',
        name='mserve_lidar',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen'
    )

    # Publishes /robot_description + /tf_static /tf from the URDF — needed so
    # a remote RViz (e.g. on Thor) can place camera_link_optical relative to
    # base_link. Not a lifecycle node — robot_state_publisher is a plain
    # topic publisher with nothing to configure/activate.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        backend_arg,
        uart_device_arg,
        drivechain,
        base,
        camera,
        lidar,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[lifecycle_manager])
    ])
