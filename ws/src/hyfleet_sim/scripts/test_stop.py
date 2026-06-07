#!/usr/bin/env python3
"""Test: STOP a running HIGH boost.

Starts HIGH at 700 bar, waits 3 seconds (booster running), then sends HIGH STOP.
The START goal is preempted (ABORTED); the STOP goal should succeed, leaving
the booster in a clean idle state.

Pass: START goal ABORTED, STOP goal SUCCEEDS.
"""
import sys
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from mserve_interfaces.action import ControlCompressor

CC = ControlCompressor
START_TARGET   = 700.0
WAIT_BEFORE_STOP = 3.0


class TestStop(Node):
    def __init__(self):
        super().__init__('test_stop')
        self._client = ActionClient(self, CC, '/hyfleet_compression/control_compressor')
        self._reset  = self.create_client(Trigger, '/compressor_sim/reset')

    def run(self) -> bool:
        self.get_logger().info(f'--- test_stop: HIGH START {START_TARGET} bar → STOP ---')

        if self._reset.wait_for_service(timeout_sec=5.0):
            f = self._reset.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available')
            return False

        # Goal 1 — start compression
        g_start = CC.Goal()
        g_start.command         = CC.Goal.START
        g_start.target          = CC.Goal.HIGH_BOOSTER
        g_start.target_pressure = START_TARGET

        send1 = self._client.send_goal_async(g_start, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send1, timeout_sec=10.0)
        h1 = send1.result()
        if h1 is None or not h1.accepted:
            self.get_logger().error('START goal rejected')
            return False
        self.get_logger().info(f'START accepted — waiting {WAIT_BEFORE_STOP}s before stop')

        result1_f = h1.get_result_async()

        deadline = time.monotonic() + WAIT_BEFORE_STOP
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Goal 2 — stop
        g_stop = CC.Goal()
        g_stop.command         = CC.Goal.STOP
        g_stop.target          = CC.Goal.HIGH_BOOSTER
        g_stop.target_pressure = 0.0

        send2 = self._client.send_goal_async(g_stop)
        rclpy.spin_until_future_complete(self, send2, timeout_sec=10.0)
        h2 = send2.result()
        if h2 is None or not h2.accepted:
            self.get_logger().error('STOP goal rejected')
            return False
        self.get_logger().info('STOP accepted')

        result2_f = h2.get_result_async()

        deadline_ns = self.get_clock().now().nanoseconds + 60 * 10**9
        while not (result1_f.done() and result2_f.done()):
            if self.get_clock().now().nanoseconds > deadline_ns:
                self.get_logger().error('Timed out')
                return False
            rclpy.spin_once(self, timeout_sec=0.5)

        status1 = result1_f.result().status
        aborted = (status1 == GoalStatus.STATUS_ABORTED)
        r2 = result2_f.result().result

        self.get_logger().info(
            f'START status={status1} ({"ABORTED ✓" if aborted else "unexpected"})  '
            f'STOP accepted={r2.accepted}'
        )

        if r2.accepted and aborted:
            self.get_logger().info('PASSED')
            return True
        self.get_logger().error('FAILED')
        return False

    def _feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'  high  {fb.pressure:.1f} bar  {fb.percent_complete:.0f}%')


def main():
    rclpy.init()
    node = TestStop()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
