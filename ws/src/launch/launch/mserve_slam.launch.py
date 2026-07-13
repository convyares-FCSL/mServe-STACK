import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    interfaces_share = get_package_share_directory('interfaces')

    # Resolved here (via context, not a substitution chain) to a plain
    # Python string — the substitution-list approach tried earlier
    # (PathJoinSubstitution with a nested list to concatenate
    # 'slam_params_' + mode + '.yaml') silently failed to produce a valid
    # path: launch_ros dropped --params-file entirely, so slam_toolbox fell
    # back to compiled-in defaults (base_frame: base_footprint, which
    # doesn't exist in this robot's TF tree) and could never compute an odom
    # pose. OpaqueFunction + context.perform() is the reliable way to build
    # a file path from a launch argument.
    mode = LaunchConfiguration('mode').perform(context)
    params_file = os.path.join(interfaces_share, 'config', f'slam_params_{mode}.yaml')

    # Fixed install location, same assumption the rest of this repo already
    # makes (e.g. /dev/ttyAMA0 for the UART). Setting this as slam_toolbox's
    # own working directory means a bare name like "my_map" in a save_map/
    # serialize_map service call — from the web UI or by hand — resolves
    # here with no path typing needed. See map/README.md.
    maps_dir = os.path.expanduser('~/mServe-STACK/map')

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
        cwd=maps_dir,
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

    return [slam_toolbox_node, slam_lifecycle_manager]


def generate_launch_description():
    # 'map' (slam_params_map.yaml) builds/extends a map while driving —
    # mode: mapping. 'local' (slam_params_local.yaml) localizes against a
    # previously saved map instead of building one — mode: localization, see
    # that file's header comment for how to actually produce a map it can
    # load (serialize_map, not save_map).
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='map',
        choices=['map', 'local'],
        description="'map' to build/extend a map, 'local' to localize against a saved one",
    )

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=launch_setup),
    ])
