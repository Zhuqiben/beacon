"""Colour calibration tool: subscribe to /camera/image_raw, click a colour patch and print
the mean HSV of the surrounding window.

Usage (needs an X11 display):
    ros2 run landmark_detector sample_color
    # Sample each colour several times at different distances and angles, then write the
    # HSV ranges into params.yaml. Thresholds are flat parameters: each segment is a
    # length-6 array [Hlo, Slo, Vlo, Hhi, Shi, Vhi].
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class Sampler(Node):
    def __init__(self):
        super().__init__('color_sampler')
        self.bridge = CvBridge()
        self.img = None
        # The Gazebo camera plugin uses SENSOR_DATA QoS (BEST_EFFORT); a default RELIABLE
        # subscription never matches it and receives no frames at all.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, '/camera/image_raw', self.cb, qos)
        cv2.namedWindow('img')
        cv2.setMouseCallback('img', self.on_click)

    def cb(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('img', self.img)
        cv2.waitKey(1)

    def on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.img is None:
            return
        patch = self.img[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        print(f'click=({x},{y}) meanHSV={hsv.reshape(-1, 3).mean(axis=0)}')


def main():
    rclpy.init()
    node = Sampler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
