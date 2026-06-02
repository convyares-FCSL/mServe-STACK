from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Get the path to the xacro file and prepare the robot description parameter.
    package_share = Path(get_package_share_directory('mserve_description'))
    xacro_file = package_share / 'urdf' / 'mserve.urdf.xacro'
    bridge_config = package_share / 'params' / 'mserve_gazebo_bridge.yaml'
    world = LaunchConfiguration('world')
    world_file = PathJoinSubstitution([
        FindPackageShare('mserve_description'),
        'worlds',
        world,
    ])

    # Declare launch arguments for Gazebo and the spawn service.
    gz_args = LaunchConfiguration('gz_args')
    world_name = LaunchConfiguration('world_name')
    entity_name = LaunchConfiguration('entity_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    use_bridge = LaunchConfiguration('use_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Use xacro to process the robot description and pass it as a parameter.
    robot_description = ParameterValue(
        Command(['xacro', ' ', str(xacro_file)]),
        value_type=str,
    )

    # Launch the Gazebo simulator with the specified arguments.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file, ' ', gz_args],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Launch the robot state publisher to publish the robot description and use Gazebo time.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # After a short delay to allow Gazebo to start, call the spawn service to insert the robot into the simulation.
    spawn_mserve = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_spawn_model.launch.py']
            )
        ),
        launch_arguments={
            'world': world_name,
            'topic': '/robot_description',
            'entity_name': entity_name,
            'x': x,
            'y': y,
            'z': z,
            'Y': yaw,
        }.items(),
    )

    # Optionally bridge Gazebo transport topics into ROS 2 using a YAML bridge config.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='mserve_gz_bridge',
        output='screen',
        parameters=[{'config_file': str(bridge_config)}],
        condition=IfCondition(use_bridge),
    )

    # Optionally launch RViz to visualize the robot and Gazebo data.
    rviz_config = package_share / 'rviz' / 'mserve.rviz'
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    # Construct the launch description with all the declared arguments and actions.
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args',
            default_value='',
            description='Additional arguments passed to ros_gz_sim/gz_sim.launch.py.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='basic.sdf',
            description='World file name under mserve_description/worlds.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='basic',
            description='Gazebo world name used by the spawn service.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='mserve',
            description='Name of the spawned Gazebo entity.',
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Spawn X position in meters.',
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Spawn Y position in meters.',
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.10',
            description='Spawn Z position in meters.',
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Spawn yaw in radians.',
        ),
        DeclareLaunchArgument(
            'use_bridge',
            default_value='true',
            description='Bridge Gazebo cmd_vel, odom, tf, joint_states, and clock topics into ROS 2.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo time for ROS nodes launched here.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz to visualize the robot and Gazebo topics.',
        ),
        gazebo,
        robot_state_publisher,
        bridge,
        rviz,
        TimerAction(period=2.0, actions=[spawn_mserve]),
    ])
