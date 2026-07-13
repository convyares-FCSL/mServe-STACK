"""TF smoke test — launches robot_state_publisher alone (no hardware, no
other mServe node) with the real xacro-expanded URDF, and checks it actually
broadcasts /tf_static for every fixed joint.

test_urdf_load.py already checks the xacro expands to the expected
links/joints/sensors structurally; this test covers the other half — that
robot_state_publisher can actually turn that URDF into TF at runtime.
Wheels (continuous joints) are deliberately not checked here: they only
appear on /tf (not /tf_static) and only once something is publishing
/joint_states, which no node in this test provides.
"""
from pathlib import Path
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

# robot_state_publisher publishes /tf_static once, latched (transient_local).
# A plain volatile subscription can start listening after that single publish
# already happened and then wait forever — durability must match to get the
# late-joiner replay.
TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# left_wheel/right_wheel are deliberately excluded — their joints are
# continuous, so they only ever appear on /tf (not /tf_static), and only
# once something is publishing /joint_states.
EXPECTED_FIXED_FRAMES = {
    'chassis',
    'caster_wheel',
    'camera_link',
    'camera_link_optical',
    'lidar_riser_link',
    'lidar_link',
    'display_link',
    'arm_mount_link',
}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    xacro_file = Path(__file__).resolve().parent.parent / 'urdf' / 'mserve.urdf.xacro'
    robot_description = ParameterValue(Command(['xacro', ' ', str(xacro_file)]), value_type=str)

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return launch.LaunchDescription([
        robot_state_publisher,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestTfSmoke(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_tf_smoke')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_tf_static_publishes_expected_fixed_frames(self):
        received = set()

        def on_tf(msg):
            received.update(t.child_frame_id for t in msg.transforms)

        sub = self.node.create_subscription(TFMessage, '/tf_static', on_tf, TF_STATIC_QOS)
        try:
            deadline = time.time() + 10
            while time.time() < deadline and not EXPECTED_FIXED_FRAMES.issubset(received):
                rclpy.spin_once(self.node, timeout_sec=0.2)
        finally:
            self.node.destroy_subscription(sub)

        missing = EXPECTED_FIXED_FRAMES - received
        self.assertFalse(missing, f"/tf_static never carried these frames: {missing}")
