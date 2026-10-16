"""Drive a LifecycleNode into the active state automatically.

Lighter than nav2_lifecycle_manager: the node does not have to implement a bond.

The order must be configure then activate; a direct activate is rejected as an invalid
transition.
"""
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node


class AutoStart(Node):
    def __init__(self):
        super().__init__('lifecycle_autostart')
        self.declare_parameter('target', '/landmark_detector')
        self.declare_parameter('delay_sec', 6.0)
        self.target = self.get_parameter('target').value
        self.delay = float(self.get_parameter('delay_sec').value)

    def run(self):
        time.sleep(self.delay)                     # let Gazebo, /clock and camera_info come up
        cli = self.create_client(ChangeState, f'{self.target}/change_state')
        if not cli.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(f'{self.target}/change_state 不可用，放弃自动激活')
            return
        for tid, name in ((Transition.TRANSITION_CONFIGURE, 'configure'),
                          (Transition.TRANSITION_ACTIVATE, 'activate')):
            req = ChangeState.Request()
            req.transition.id = tid
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
            ok = fut.done() and fut.result() is not None and fut.result().success
            self.get_logger().info(f'{self.target} {name}: {"OK" if ok else "FAILED"}')
            if not ok:
                return


def main():
    rclpy.init()
    node = AutoStart()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
