import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# AMCL + map_server for localization (switched 2026-07-19 from slam_toolbox's
# own localization mode — see nav2_params.yaml's header comment for why:
# slam_toolbox's job here is building maps, not the click-to-correct
# /initialpose ergonomics AMCL provides for keeping the live scan aligned to
# a saved map). slam_toolbox (--slam-map) and this launch file are now fully
# independent — no --slam-local pairing needed, --nav2 works standalone.
#
# Costmaps aren't separate nodes here — nav2_costmap_2d runs embedded inside
# controller_server (local_costmap) and planner_server (global_costmap),
# standard Nav2 architecture, just parameter namespacing in nav2_params.yaml.
#
# Managed by Nav2's own nav2_lifecycle_manager, not this project's custom
# BT.CPP-driven lifecycle_manager (which only manages mserve's own packages
# — drivechain/base/camera/lidar). Nav2 ships a purpose-built lifecycle
# manager for exactly its own node set; no reason to reimplement that.
def generate_launch_description():
    interfaces_share = get_package_share_directory('interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'nav2_params.yaml')

    # map_server + amcl first — controller/planner/bt_navigator all depend on
    # a valid map + localized pose being available.
    lifecycle_nodes = [
        'map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file],
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
    )
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='nav2_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': lifecycle_nodes,
            'bond_timeout': 4.0,
        }],
    )

    return LaunchDescription([
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        nav2_lifecycle_manager,
    ])
