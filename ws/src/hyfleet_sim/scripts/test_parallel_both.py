#!/usr/bin/env python3
"""Test: LOW and HIGH run simultaneously in parallel slots.

Sends both goals without waiting for the first to finish.
Pass: both goals complete successfully, independent of each other.
Fail: either goal rejected, aborted, or timed out.
"""
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from mserve_interfaces.action import ControlCompressor

CC = ControlCompressor
LOW_TARGET  = 380.0
HIGH_TARGET = 700.0


class TestParallelBoth(Node):
    def __init__(self):
        super().__init__('test_parallel_both')
        self._client = ActionClient(self, CC, '/hyfleet_compression/control_compressor')
        self._reset  = self.create_client(Trigger, '/compressor_sim/reset')

    def run(self) -> bool:
        self.get_logger().info(
            f'--- test_parallel_both: LOW {LOW_TARGET} bar + HIGH {HIGH_TARGET} bar simultaneously ---'
        )

        if self._reset.wait_for_service(timeout_sec=5.0):
            f = self._reset.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available')
            return False

        low_goal = CC.Goal()
        low_goal.command         = CC.Goal.START
        low_goal.target          = CC.Goal.LOW_BOOSTER
        low_goal.target_pressure = LOW_TARGET

        high_goal = CC.Goal()
        high_goal.command         = CC.Goal.START
        high_goal.target          = CC.Goal.HIGH_BOOSTER
        high_goal.target_pressure = HIGH_TARGET

        # Send both goals without waiting for either to finish
        low_send_f  = self._client.send_goal_async(low_goal,  feedback_callback=self._low_fb)
        high_send_f = self._client.send_goal_async(high_goal, feedback_callback=self._high_fb)

        rclpy.spin_until_future_complete(self, low_send_f,  timeout_sec=10.0)
        rclpy.spin_until_future_complete(self, high_send_f, timeout_sec=10.0)

        low_handle  = low_send_f.result()
        high_handle = high_send_f.result()

        if low_handle is None or not low_handle.accepted:
            self.get_logger().error('LOW goal rejected')
            return False
        if high_handle is None or not high_handle.accepted:
            self.get_logger().error('HIGH goal rejected')
            return False

        low_result_f  = low_handle.get_result_async()
        high_result_f = high_handle.get_result_async()

        # Spin until both results are ready
        deadline = self.get_clock().now().nanoseconds + 120 * 10**9
        while not (low_result_f.done() and high_result_f.done()):
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().error('Timed out waiting for both results')
                return False
            rclpy.spin_once(self, timeout_sec=0.5)

        low_ok  = low_result_f.result().result.accepted
        high_ok = high_result_f.result().result.accepted
        low_msg  = low_result_f.result().result.message
        high_msg = high_result_f.result().result.message

        if low_ok and high_ok:
            self.get_logger().info(f'PASSED — low: {low_msg}  high: {high_msg}')
            return True
        if not low_ok:
            self.get_logger().error(f'FAILED (LOW) — {low_msg}')
        if not high_ok:
            self.get_logger().error(f'FAILED (HIGH) — {high_msg}')
        return False

    def _low_fb(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'  [LOW]  {fb.pressure:.1f} bar  {fb.percent_complete:.0f}%')

    def _high_fb(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'  [HIGH] {fb.pressure:.1f} bar  {fb.percent_complete:.0f}%')


def main():
    rclpy.init()
    node = TestParallelBoth()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
