from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('mserve_description'))
    xacro_file = package_share / 'urdf' / 'mserve.urdf.xacro'
    rviz_config = package_share / 'rviz' / 'mserve.rviz'

    robot_description = Command(['xacro', ' ', str(xacro_file)])

    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_joint_state_gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_config)],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joint_state_gui',
            default_value='true',
            description='Launch joint_state_publisher_gui for movable wheel joints.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the saved mServe config.',
        ),
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz,
    ])
