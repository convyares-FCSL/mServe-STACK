"""Smoke test for mserve_min.launch.py.

Doesn't actually launch anything (no hardware/ROS graph needed) — just
verifies generate_launch_description() builds without error and wires up
the package/executable pairs the rest of the stack depends on. A missing
node here means a bringup that silently launches an incomplete stack.

Deliberately does NOT import mserve_min_launch as
`from launch.mserve_min_launch import generate_launch_description`: this
package's own name ("launch") shadows the installed module search path in
a way that makes that import ambiguous. Loading the file directly by path
sidesteps it — see package.xml for the same "launch" name collision
affecting declared dependencies.
"""
import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node


def _load_launch_module():
    launch_file = Path(__file__).resolve().parent.parent / 'launch' / 'mserve_min.launch.py'
    spec = importlib.util.spec_from_file_location('mserve_min_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_generate_launch_description_builds():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    assert ld.entities


def test_bringup_declares_backend_and_uart_args():
    module = _load_launch_module()
    ld = module.generate_launch_description()

    arg_names = {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert {'backend', 'uart_device'}.issubset(arg_names)


def test_bringup_includes_expected_nodes():
    module = _load_launch_module()
    ld = module.generate_launch_description()

    top_level_nodes = [e for e in ld.entities if isinstance(e, Node)]
    timer_actions = [e for e in ld.entities if isinstance(e, TimerAction)]

    # lifecycle_manager is deliberately delayed (TimerAction) so its bringup
    # tree doesn't race the other nodes coming up — it must not just be lost.
    assert len(timer_actions) == 1
    delayed_nodes = [a for a in timer_actions[0].actions if isinstance(a, Node)]

    pkg_exec_pairs = {(n.node_package, n.node_executable) for n in top_level_nodes + delayed_nodes}

    assert pkg_exec_pairs == {
        ('mserve_drivechain', 'drivechain_node'),
        ('mserve_base', 'base_node'),
        ('mserve_camera', 'camera_node'),
        ('mserve_lidar', 'lidar_node'),
        ('robot_state_publisher', 'robot_state_publisher'),
        ('lifecycle_manager', 'lifecycle_manager'),
    }
