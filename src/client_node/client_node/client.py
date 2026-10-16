#!/usr/bin/env python3
"""Interactive / scriptable client for the GoToBlock action.

Sends a goal to the `goto_block` action server, prints every feedback message, and
cancels cleanly on Ctrl-C or after --cancel-after seconds.

Examples
--------
    ros2 run client_node client --color yellow
    ros2 run client_node client --color blue --approach 0.8 --timeout 90
    ros2 run client_node client --color red --cancel-after 5
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from slam_interfaces.action import GoToBlock


class GoToBlockClient(Node):
    def __init__(self):
        super().__init__('goto_block_client')
        self._client = ActionClient(self, GoToBlock, 'goto_block')
        self.feedback_count = 0

    def wait_for_server(self, timeout_sec):
        self.get_logger().info('waiting for /goto_block action server ...')
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'/goto_block not available after {timeout_sec:.0f} s - '
                'is goal_selector running?')
            return False
        self.get_logger().info('/goto_block ready')
        return True

    def on_feedback(self, msg):
        fb = msg.feedback
        self.feedback_count += 1
        self.get_logger().info(
            f'[fb {self.feedback_count:3d}] {fb.state:<12s} '
            f'target_known={fb.target_known} '
            f'dist={fb.distance_to_target:6.2f} m '
            f'elapsed={fb.elapsed_time:6.2f} s')

    def send(self, color, timeout_sec, approach_distance):
        goal = GoToBlock.Goal()
        goal.target_color = color
        goal.timeout_sec = float(timeout_sec)
        goal.approach_distance = float(approach_distance)
        self.get_logger().info(
            f'sending goal: color={color} timeout={timeout_sec:.1f}s '
            f'approach={approach_distance:.2f}m')
        return self._client.send_goal_async(goal, feedback_callback=self.on_feedback)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--color', default='yellow',
                        choices=['red', 'green', 'blue', 'yellow'],
                        help='target landmark colour (default: yellow)')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='server-side timeout in seconds (default: 60)')
    parser.add_argument('--approach', type=float, default=0.6,
                        help='stop distance from the landmark in metres (default: 0.6)')
    parser.add_argument('--server-wait', type=float, default=30.0,
                        help='seconds to wait for the action server (default: 30)')
    parser.add_argument('--cancel-after', type=float, default=0.0,
                        help='cancel the goal after N seconds (0 = never); '
                             'used to test abort/replan handling')
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    # Let Ctrl-C reach our handler instead of rclpy's default hard kill.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = GoToBlockClient()
    rc = 0
    goal_handle = None
    try:
        if not node.wait_for_server(args.server_wait):
            return 2

        send_future = node.send(args.color, args.timeout, args.approach)
        rclpy.spin_until_future_complete(node, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().error('goal REJECTED by the action server')
            return 3
        node.get_logger().info('goal accepted')

        result_future = goal_handle.get_result_async()
        cancelled_at = None
        if args.cancel_after > 0.0:
            cancelled_at = time.monotonic() + args.cancel_after

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if cancelled_at is not None and time.monotonic() >= cancelled_at:
                node.get_logger().warn(
                    f'--cancel-after {args.cancel_after:.1f}s elapsed, cancelling goal')
                goal_handle.cancel_goal_async()
                cancelled_at = None          # only cancel once

        if not rclpy.ok():
            return 0

        wrapped = result_future.result()
        if wrapped is None:
            node.get_logger().error('no result returned')
            return 4
        res = wrapped.result
        node.get_logger().info('-' * 60)
        node.get_logger().info(
            f'success={res.success}  final_distance={res.final_distance:.3f} m')
        node.get_logger().info(f'message: {res.message}')
        node.get_logger().info(
            f'feedback messages received: {node.feedback_count}')
        node.get_logger().info('-' * 60)
        rc = 0 if res.success else 1

    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl-C received, cancelling goal')
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
                time.sleep(0.3)
            except Exception:               # noqa: BLE001 - best effort on teardown
                pass
        rc = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
