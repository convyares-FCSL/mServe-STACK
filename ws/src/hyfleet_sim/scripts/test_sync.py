#!/usr/bin/env python3
"""Test: SYNC — low holds interstage band while high compresses to target.

Pass: coordinator goal SUCCEEDS after high booster reaches target.
Fail: goal rejected, timeout, or either booster reports failure.
"""
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from mserve_interfaces.action import ControlCompressor

CC = ControlCompressor
TARGET_BAR = 900.0


class TestSync(Node):
    def __init__(self):
        super().__init__('test_sync')
        self._client = ActionClient(self, CC, '/hyfleet_compression/control_compressor')
        self._reset  = self.create_client(Trigger, '/compressor_sim/reset')

    def run(self) -> bool:
        self.get_logger().info(f'--- test_sync: SYNC START → {TARGET_BAR} bar ---')

        if self._reset.wait_for_service(timeout_sec=5.0):
            f = self._reset.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available')
            return False

        goal = CC.Goal()
        goal.command         = CC.Goal.START
        goal.target          = CC.Goal.SYNC_BOOSTERS
        goal.target_pressure = TARGET_BAR
        goal.mode            = CC.Goal.PERFORMANCE

        send_f = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_f, timeout_sec=10.0)
        handle = send_f.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_f = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_f, timeout_sec=120.0)
        if not result_f.done():
            self.get_logger().error('Timed out waiting for result')
            return False

        r = result_f.result().result
        if r.accepted:
            self.get_logger().info(f'PASSED — {r.message}')
            return True
        self.get_logger().error(f'FAILED — {r.message}')
        return False

    def _feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'  high {fb.pressure:.1f} bar  {fb.percent_complete:.0f}%')


def main():
    rclpy.init()
    node = TestSync()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()