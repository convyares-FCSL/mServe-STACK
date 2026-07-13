import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    interfaces_share = get_package_share_directory('interfaces')
    params_file = os.path.join(interfaces_share, 'config', 'slam_toolbox_params.yaml')

    # slam_toolbox's own node is already a lifecycle node (same pattern as
    # every mserve_* node) — no need to wrap it, just drive it through
    # configure/activate ourselves. This is a single node with no bringup
    # sequencing/retries needed, so plain launch_ros event handlers are
    # enough; doesn't need the BT-based lifecycle_manager that manages the
    # always-on drivechain/base/camera/lidar stack in mserve_min.launch.py.
    # Kept as its own launch file (not folded into mserve_min.launch.py)
    # because mapping is an occasional, opt-in session, not something you
    # want running on every boot.
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params_file],
    )

    configure_on_start = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_after_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        slam_toolbox_node,
        activate_after_configure,
        configure_on_start,
    ])
