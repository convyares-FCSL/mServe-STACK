"""
hyfleet_sim.launch.py
=====================
Brings up the compression test simulator alongside the booster and coordinator
nodes.  The sim replaces the ADS bridge + PLC + plant; all other nodes run
completely unchanged.

Bringup order mirrors mserve_min.launch.py:
  sim  →  low_booster  →  high_booster  →  hyfleet_compression  →  lifecycle_manager

The lifecycle_manager is delayed by 2 s (same as mserve_min) to ensure all
lifecycle nodes are registered before the manager starts transitioning them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory('mserve_interfaces')
    sim_share = get_package_share_directory('hyfleet_sim')
    params_file = os.path.join(interfaces_share, 'config', 'mserve_params.yaml')
    hyfleet_params = os.path.join(sim_share, 'config', 'hyfleet_params.yaml')

    # ------------------------------------------------------------------
    # Simulator  — plain (non-lifecycle) node; starts first so services
    # are available before the lifecycle nodes configure.
    # ------------------------------------------------------------------
    sim_node = Node(
        package='hyfleet_sim',
        executable='sim_node',
        name='compressor_sim',
        output='screen',
        parameters=[hyfleet_params],
    )

    # ------------------------------------------------------------------
    # Booster nodes  (lifecycle, unchanged from production)
    # ------------------------------------------------------------------
    low_booster_node = Node(
        package='hyfleet_booster',
        executable='booster_node',
        name='low_booster',
        output='screen',
        parameters=[params_file, hyfleet_params],
    )

    high_booster_node = Node(
        package='hyfleet_booster',
        executable='booster_node',
        name='high_booster',
        output='screen',
        parameters=[params_file, hyfleet_params],
    )

    # ------------------------------------------------------------------
    # Coordinator  (lifecycle, unchanged from production)
    # ------------------------------------------------------------------
    compressor_node = Node(
        package='hyfleet_compressor',
        executable='compressor_node',
        name='hyfleet_compression',
        output='screen',
        parameters=[params_file, hyfleet_params],
    )

    # ------------------------------------------------------------------
    # Lifecycle manager  — delayed so all lifecycle nodes are registered
    # ------------------------------------------------------------------
    lifecycle_manager = Node(
        package='mserve_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
    )

    return LaunchDescription([
        sim_node,
        low_booster_node,
        high_booster_node,
        compressor_node,
        TimerAction(period=2.0, actions=[lifecycle_manager]),
    ])
