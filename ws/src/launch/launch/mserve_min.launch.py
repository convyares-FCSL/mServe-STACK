import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interfaces_share = get_package_share_directory('interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'mserve_params.yaml')
    # controller_profiles for every supported controller model live in one
    # file, always loaded — mserve_params.yaml's mserve_joystick.
    # controller_profile value (read by the node itself, not here) picks
    # which one to actually use. See joystick_params.yaml's header comment.
    joystick_params_file = os.path.join(interfaces_share, 'config', 'joystick_params.yaml')

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

    # Default true (unchanged native behavior). The Docker path doesn't have
    # the camera/lidar driver deps built into the image yet (see
    # transfer.md's "Known gaps" — real new work, not a restore), so
    # run_stack.sh passes these false there to keep drivechain+base+rosbridge
    # bringup working without them.
    with_camera_arg = DeclareLaunchArgument(
        'with_camera', default_value='true',
        description='Start mserve_camera'
    )
    with_lidar_arg = DeclareLaunchArgument(
        'with_lidar', default_value='true',
        description='Start mserve_lidar'
    )
    with_display_arg = DeclareLaunchArgument(
        'with_display', default_value='true',
        description='Start mserve_display'
    )
    with_joystick_arg = DeclareLaunchArgument(
        'with_joystick', default_value='true',
        description='Start joy_node + mserve_joystick'
    )
    with_sensehat_arg = DeclareLaunchArgument(
        'with_sensehat', default_value='true',
        description='Start mserve_sensehat (Pi Sense HAT: LED matrix, joystick, IMU)'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            # Keeps /joy publishing continuously even with the stick
            # centered — mserve_base zeroes the drive command if no
            # /cmd_vel arrives within cmd_vel_timeout_ms (500ms default), so
            # mserve_joystick needs a steady stream of /joy callbacks to
            # re-publish from, not just on-change.
            'autorepeat_rate': 20.0,
            'device_id': 0,
        }],
        condition=IfCondition(LaunchConfiguration('with_joystick')),
    )

    joystick = Node(
        package='mserve_joystick',
        executable='joystick_node',
        name='mserve_joystick',
        output='screen',
        parameters=[params_file, joystick_params_file],
        condition=IfCondition(LaunchConfiguration('with_joystick')),
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
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('with_camera')),
    )

    lidar = Node(
        package='mserve_lidar',
        executable='lidar_node',
        name='mserve_lidar',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('with_lidar')),
    )

    # camera/lidar disabled (the Docker path today) means the default
    # bringup.xml/shutdown.xml would still burn ~25s on dead-service retries
    # for mserve_camera/mserve_lidar's lifecycle services, which never come
    # up, before drivechain/base are even touched — the same tradeoff
    # lifecycle_manager/README.md's "Running more than one instance" section
    # already documents (it's why SLAM runs as an opt-in second instance
    # instead of living in the always-on tree). Point at the drivechain+base
    # -only bringup_min.xml/shutdown_min.xml pair instead when both are off.
    camera_or_lidar_enabled = PythonExpression([
        "'", LaunchConfiguration('with_camera'), "' == 'true' or '",
        LaunchConfiguration('with_lidar'), "' == 'true'",
    ])

    lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        condition=IfCondition(camera_or_lidar_enabled),
    )

    lifecycle_manager_min = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'bringup_tree_file': 'bringup_min.xml',
            'shutdown_tree_file': 'shutdown_min.xml',
        }],
        condition=UnlessCondition(camera_or_lidar_enabled),
    )

    # Not a lifecycle node — same reasoning as robot_state_publisher below,
    # no hardware "connect" step worth gating (just opens /dev/fb0 + finds
    # the touch input device at startup). See mserve_display/README.md.
    display = Node(
        package='mserve_display',
        executable='display_node',
        name='mserve_display',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('with_display')),
    )

    # Not a lifecycle node — same reasoning as mserve_display above (no
    # hardware "connect" step worth gating, just opens fbdev/evdev/i2c
    # devices at startup). See mserve_sensehat/README.md.
    sensehat = Node(
        package='mserve_sensehat',
        executable='sensehat_node',
        name='mserve_sensehat',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('with_sensehat')),
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
        with_camera_arg,
        with_lidar_arg,
        with_display_arg,
        with_joystick_arg,
        with_sensehat_arg,
        drivechain,
        base,
        camera,
        lidar,
        display,
        sensehat,
        robot_state_publisher,
        joy_node,
        joystick,
        TimerAction(period=2.0, actions=[lifecycle_manager, lifecycle_manager_min])
    ])
