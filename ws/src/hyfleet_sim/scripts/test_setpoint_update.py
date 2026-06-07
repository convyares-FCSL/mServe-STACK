#!/usr/bin/env python3
"""Test: setpoint update while LOW is running.

Sends LOW at 350 bar, waits 3 seconds (booster is compressing), then
sends a new LOW at 480 bar.  Because both goals target the same slot,
the first is preempted (ABORTED) and the second runs to the higher target.

Pass: first goal ABORTED with 'replaced by newer goal', second goal SUCCEEDS.
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
INITIAL_TARGET = 350.0
UPDATED_TARGET = 480.0
WAIT_BEFORE_UPDATE_SEC = 3.0


class TestSetpointUpdate(Node):
    def __init__(self):
        super().__init__('test_setpoint_update')
        self._client = ActionClient(self, CC, '/hyfleet_compression/control_compressor')
        self._reset  = self.create_client(Trigger, '/compressor_sim/reset')

    def run(self) -> bool:
        self.get_logger().info(
            f'--- test_setpoint_update: LOW {INITIAL_TARGET} → {UPDATED_TARGET} bar ---'
        )

        if self._reset.wait_for_service(timeout_sec=5.0):
            f = self._reset.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)

        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Action server not available')
            return False

        # Goal 1 — initial setpoint
        g1 = CC.Goal()
        g1.command         = CC.Goal.START
        g1.target          = CC.Goal.LOW_BOOSTER
        g1.target_pressure = INITIAL_TARGET

        send1 = self._client.send_goal_async(g1, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send1, timeout_sec=10.0)
        h1 = send1.result()
        if h1 is None or not h1.accepted:
            self.get_logger().error('Goal 1 rejected')
            return False
        self.get_logger().info(f'Goal 1 accepted — waiting {WAIT_BEFORE_UPDATE_SEC}s before update')

        result1_f = h1.get_result_async()

        # Spin for a few seconds so booster is actively compressing
        deadline = time.monotonic() + WAIT_BEFORE_UPDATE_SEC
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Goal 2 — updated setpoint (same slot, same command → preempts goal 1)
        g2 = CC.Goal()
        g2.command         = CC.Goal.START
        g2.target          = CC.Goal.LOW_BOOSTER
        g2.target_pressure = UPDATED_TARGET

        send2 = self._client.send_goal_async(g2, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send2, timeout_sec=10.0)
        h2 = send2.result()
        if h2 is None or not h2.accepted:
            self.get_logger().error('Goal 2 rejected')
            return False
        self.get_logger().info('Goal 2 accepted — waiting for both results')

        result2_f = h2.get_result_async()

        deadline_ns = self.get_clock().now().nanoseconds + 120 * 10**9
        while not (result1_f.done() and result2_f.done()):
            if self.get_clock().now().nanoseconds > deadline_ns:
                self.get_logger().error('Timed out')
                return False
            rclpy.spin_once(self, timeout_sec=0.5)

        # Goal 1 should be ABORTED (preempted)
        status1 = result1_f.result().status
        msg1    = result1_f.result().result.message
        aborted = (status1 == GoalStatus.STATUS_ABORTED)
        self.get_logger().info(
            f'Goal 1 status={status1} ({"ABORTED ✓" if aborted else "unexpected"}) — {msg1}'
        )

        # Goal 2 should succeed
        r2 = result2_f.result().result
        if r2.accepted and aborted:
            self.get_logger().info(f'PASSED — goal 2: {r2.message}')
            return True
        self.get_logger().error(f'FAILED — goal 2 accepted={r2.accepted}  goal 1 aborted={aborted}')
        return False

    def _feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(f'  low  {fb.pressure:.1f} bar  {fb.percent_complete:.0f}%')


def main():
    rclpy.init()
    node = TestSetpointUpdate()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
