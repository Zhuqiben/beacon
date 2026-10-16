"""Colour landmark detection: LifecycleNode, synchronised RGB-D, flat HSV thresholds.

Every module-level function is pure: no ROS graph involved, so tests can call them directly
(see test/test_color.py).
"""
import math

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from slam_interfaces.msg import Landmark, LandmarkArray

LANDS = ('red', 'green', 'blue', 'yellow')

# Threshold segment names (flat parameter names) to colour names; red straddles 0/180 and
# therefore needs two segments.
SEGMENTS = ('red_a', 'red_b', 'green', 'blue', 'yellow', 'white', 'black')
COLOR_OF = {'red_a': 'red', 'red_b': 'red', 'green': 'green', 'blue': 'blue',
            'yellow': 'yellow', 'white': 'white', 'black': 'black'}
DEFAULT_SEG = [0, 0, 0, 255, 255, 255]


# ---------------- pure helpers (unit-testable, no ROS node needed) ----------------
def add_segment(thresholds, name, vals):
    """Merge one flat threshold segment into ``thresholds``.

    ``vals`` is a 6-element array [Hlo, Slo, Vlo, Hhi, Shi, Vhi]. ROS 2 parameters have no map
    type, so each segment is its own parameter; red uses red_a and red_b and is merged here
    into one colour carrying several segments (see COLOR_OF). Returns False if the length is
    wrong, in which case the segment is skipped.
    """
    if len(vals) != 6:
        return False
    low = np.array(vals[0:3], dtype=np.uint8)
    high = np.array(vals[3:6], dtype=np.uint8)
    thresholds.setdefault(COLOR_OF[name], []).append((low, high))
    return True


def split_colors(thresholds):
    """Split the active colours into (landmarks, obstacles) name lists for the start-up log."""
    lm = [c for c in thresholds if c in LANDS]
    ob = [c for c in thresholds if c not in LANDS]
    return lm, ob


def color_mask(hsv, ranges):
    """Union mask of one or more threshold segments (red needs two)."""
    mask = None
    for low, high in ranges:
        m = cv2.inRange(hsv, low, high)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def bearing_from_pixel(u, K):
    """Optical-frame u (positive to the right) to body-frame bearing (positive to the left,
    i.e. counter-clockwise).

    phi_body = -atan2(u - cx, fx). Getting the sign wrong mirrors the whole map left to
    right, which conflicts with the odometry prediction and always makes the EKF diverge.
    K = [fx, 0, cx, ...].
    """
    return -math.atan2(u - K[2], K[0])


def planar_range(depth, bearing, cos_floor=0.35):
    """Depth along the optical axis to the planar range the EKF expects.

    The measurement model is z = [r, phi] with z_hat = [sqrt(dx^2 + dy^2), atan2(dy, dx) - theta],
    so r is the planar Euclidean distance to the landmark, while the depth camera reports the
    distance along the optical axis. The same line of sight hits the same surface point at
    bearing phi, hence planar range = depth / cos(phi). This step cannot be dropped: at
    phi = 46 deg the two differ by 1 / cos(46 deg) = 1.44, so using the depth directly shrinks
    the map towards the observer (a landmark at 2.47 m lands at 1.73 m).

    The correction grows with the obliquity of the line of sight, so cos is floored to keep
    grazing angles from amplifying depth noise without bound.
    """
    return depth / max(math.cos(bearing), cos_floor)


def percentile_depth(depth_img, x, y, w, h, margin, pct, min_range, max_range, is_16u):
    """Percentile ``pct`` of the valid depths inside the bbox, in metres, after shrinking the
    bbox by ``margin``.

    The centre value of the bbox is not used: the landmark has height and the camera looks
    down on it, so the centre often falls on the lower edge of the block or on the floor. The
    floor is inside the valid range as well and is not filtered out, which systematically
    overestimates the range. ``is_16u`` selects the millimetre conversion, since the Gazebo
    depth plugin publishes either 32FC1 or 16UC1.
    """
    mx, my = int(w * margin), int(h * margin)
    x0, y0 = max(0, x + mx), max(0, y + my)
    roi = depth_img[y0:y + h - my, x0:x + w - mx]
    if roi.size == 0:
        return None
    roi = roi.astype(np.float32)
    if is_16u:
        roi = roi * 0.001                      # mm to m
    roi = roi[np.isfinite(roi)]
    roi = roi[(roi > min_range * 0.5) & (roi < max_range * 2.0)]
    if roi.size == 0:
        return None
    return float(np.percentile(roi, pct))


class DetectorNode(LifecycleNode):
    def __init__(self):
        super().__init__('landmark_detector')
        self.bridge = CvBridge()
        self.K = None
        self.thresholds = {}          # color -> [(low, high), ...]
        self.sync = None
        self.pub = None
        self.sub_rgb = self.sub_dep = self.sub_info = None
        self.depth_is_16u = None      # None = not probed yet

    # ---------------- lifecycle ----------------
    def on_configure(self, state):
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('min_area', 200.0)
        self.declare_parameter('depth_percentile', 30.0)
        self.declare_parameter('bbox_margin', 0.25)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.depth_pct = float(self.get_parameter('depth_percentile').value)
        self.bbox_margin = float(self.get_parameter('bbox_margin').value)

        # One flat parameter per segment, each a 6-element array.
        self.thresholds = {}
        for name in SEGMENTS:
            try:
                self.declare_parameter(name, DEFAULT_SEG)
                vals = list(self.get_parameter(name).value)
            except Exception as exc:
                # Typical cause: params.yaml holds floats ([0.0, 120.0, ...]), which does not
                # match the INTEGER_ARRAY default. This has to be reported loudly.
                self.get_logger().error(
                    f'阈值参数 "{name}" 读取失败：{exc} —— params.yaml 里必须是'
                    ' 长度 6 的整数数组，如 [0, 120, 70, 10, 255, 255]')
                continue
            if not add_segment(self.thresholds, name, vals):
                self.get_logger().warn(f'阈值 "{name}" 长度不是 6，已跳过')

        lm, ob = split_colors(self.thresholds)
        self.get_logger().info(f'thresholds: landmarks={lm} obstacles={ob}')
        if not self.thresholds:
            self.get_logger().error(
                '没有任何阈值参数生效 —— 检查 params.yaml 是否为扁平参数')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # The camera publisher depth_cam is RELIABLE and this subscriber has to match it. A
        # BEST_EFFORT subscriber on a RELIABLE publisher still matches, but delivers only
        # about 0.40 Hz (90% of the frames dropped), so while spinning in place a block
        # behind the robot (red/blue) often never produces a single frame.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub_rgb = message_filters.Subscriber(self, Image, '/camera/image_raw',
                                                  qos_profile=qos)
        self.sub_dep = message_filters.Subscriber(self, Image, '/camera/depth/image_raw',
                                                  qos_profile=qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_dep], 10, 0.05)
        self.sync.registerCallback(self.on_pair)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera_info',
                                                 self.on_info, 10)
        if self.pub is None:
            self.pub = self.create_lifecycle_publisher(LandmarkArray, '/observations', 10)
        # A publisher created by create_lifecycle_publisher starts out disabled, and
        # publish() is silently dropped by the when_enabled decorator. This node implements
        # on_activate/on_deactivate without calling super(), so rclpy does not activate the
        # managed entity for us: without the explicit activation below, /observations stays
        # an empty array forever.
        self.pub.on_activate(state)
        self.get_logger().info('landmark_detector ACTIVE')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        # The synchroniser is not a Subscription, clearing it is enough; the real
        # subscriptions have to be torn down one by one.
        self.sync = None
        if self.sub_rgb is not None:
            # rclpy's Subscription only has destroy(), no unregister() (the __getattr__ of
            # message_filters.Subscriber proxies unregister through and raises
            # AttributeError, which makes deactivate return ERROR), so use the node API.
            self.destroy_subscription(self.sub_rgb.sub)
            self.sub_rgb = None
        if self.sub_dep is not None:
            self.destroy_subscription(self.sub_dep.sub)
            self.sub_dep = None
        if self.sub_info is not None:
            self.destroy_subscription(self.sub_info)
            self.sub_info = None
        if self.pub is not None:
            self.pub.on_deactivate(state)
        self.get_logger().info('landmark_detector INACTIVE')
        return TransitionCallbackReturn.SUCCESS

    # ---------------- callbacks ----------------
    def on_info(self, msg):
        self.K = msg.k

    def depth_of_bbox(self, depth_img, contour):
        x, y, w, h = cv2.boundingRect(contour)
        return percentile_depth(depth_img, x, y, w, h, self.bbox_margin,
                                self.depth_pct, self.min_range, self.max_range,
                                self.depth_is_16u)

    def on_pair(self, rgb_msg, dep_msg):
        if self.K is None or self.pub is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(dep_msg, desired_encoding='passthrough')
        if depth is None or depth.size == 0:
            self.get_logger().warn('深度图为空，跳过本帧', throttle_duration_sec=5.0)
            return
        if self.depth_is_16u is None:               # probe the encoding once, first frame
            self.depth_is_16u = (depth.dtype == np.uint16)
            self.get_logger().info(
                f'[depth] dtype={depth.dtype}, treat_as_16u={self.depth_is_16u}')

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        out = LandmarkArray()
        out.header = rgb_msg.header

        for color, ranges in self.thresholds.items():
            mask = color_mask(hsv, ranges)           # red: union of both segments
            if mask is None:
                continue
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in cnts:
                if cv2.contourArea(c) < self.min_area:
                    continue
                r = self.depth_of_bbox(depth, c)
                if r is None or r > self.max_range or r < self.min_range:
                    continue
                (cx, cy), _, _ = cv2.minAreaRect(c)
                # Optical u is positive to the right, body y is positive to the left:
                #   phi_body = -atan2(u - cx, fx)
                # Skipping this mirrors the map left to right, which conflicts with the
                # odometry prediction and makes the EKF diverge.
                phi = float(bearing_from_pixel(cx, self.K))
                r = planar_range(r, phi)             # optical-axis depth to planar range
                if r > self.max_range:
                    continue
                lm = Landmark()
                lm.color = color
                lm.role = 'landmark' if color in LANDS else 'obstacle'
                lm.range = float(r)
                lm.bearing = phi
                lm.confidence = 1.0
                out.landmarks.append(lm)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
