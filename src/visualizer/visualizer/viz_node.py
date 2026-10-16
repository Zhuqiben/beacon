#!/usr/bin/env python3
"""Visualiser: annotate the camera stream and publish RViz markers.

Publishes
---------
/observations/annotated  sensor_msgs/Image                 camera frame with detection boxes
/landmark_markers        visualization_msgs/MarkerArray    landmarks plus uncertainty spheres

Subscribes
----------
/camera/image_raw   sensor_msgs/Image              frames to annotate
/camera/camera_info sensor_msgs/CameraInfo         intrinsics, for the bearing to pixel projection
/observations       slam_interfaces/LandmarkArray  detections to draw
/landmark_map       slam_interfaces/LandmarkMap    EKF estimates to draw (latched)

Landmark markers persist once detected
--------------------------------------
``/landmark_map`` keeps every landmark it has ever initialised: ``slam_node`` sets ``valid``
once in ``init_landmark`` and never clears it. That matches the EKF-SLAM semantics - leaving
the field of view must not drop a landmark from the state vector, or the map could never
support loop closure - and ``/map``, which feeds the ``StaticLayer`` of ``global_costmap``,
relies on it.

Drawing a landmark is therefore not premature: ``valid`` is driven by the EKF's ``inited``
flag, which is only set after a genuine detection. A landmark is drawn for as long as it stays
``valid``, which is the default (``obs_ttl_sec <= 0``).

Setting ``obs_ttl_sec > 0`` enables the stricter gate of "seen within the last N seconds,
otherwise ``Marker.DELETE``", for anyone who wants the markers to follow the camera.

The estimator, ``/landmark_map``, ``/map`` and therefore Nav2 are unaffected.

Run alongside RViz:
    rviz2 -d $(ros2 pkg prefix sim_bringup)/share/sim_bringup/config/slam.rviz
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from slam_interfaces.msg import LandmarkArray, LandmarkMap


# BGR, chosen to stay legible against the Gazebo arena floor (a mid grey).
ROLE_BGR = {
    'landmark': (0, 255, 0),
    'obstacle': (0, 0, 255),
}
ROLE_RGBA = {
    'landmark': (0.1, 0.9, 0.1, 0.9),
    'obstacle': (0.9, 0.2, 0.2, 0.9),
}


def qos_latched():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.declare_parameter('draw_labels', True)
        self.declare_parameter('marker_z', 0.25)
        # Time window for the "seen recently" gate on the landmark markers. The default 0.0
        # disables the gate, so a landmark is drawn while it is valid, and valid is only ever
        # set after a real detection (see the module docstring).
        self.declare_parameter('obs_ttl_sec', 0.0)
        self.draw_labels = bool(self.get_parameter('draw_labels').value)
        self.marker_z = float(self.get_parameter('marker_z').value)
        self.obs_ttl_sec = float(self.get_parameter('obs_ttl_sec').value)

        self.bridge = CvBridge()
        self.fx = None
        self.cx = None
        self.last_obs = None
        self.latest_map = None
        self.image_count = 0
        self.last_seen = {}          # colour -> time it was last reported
        self.drawn = set()           # colours drawn last round, so the log only fires on change

        self.pub_img = self.create_publisher(Image, '/observations/annotated', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/landmark_markers', 10)

        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(LandmarkArray, '/observations', self.on_obs, 10)
        self.create_subscription(LandmarkMap, '/landmark_map', self.on_map, qos_latched())
        self.create_timer(0.2, self.publish_markers)

        self.get_logger().info('visualizer up; publishing /observations/annotated and '
                               '/landmark_markers')

    # ------------------------------------------------------------------ inputs

    def on_info(self, msg):
        # Pinhole model: u = cx + fx * tan(bearing). fx from CameraInfo.K[0].
        self.fx = float(msg.k[0]) or None
        self.cx = float(msg.k[2]) if self.fx else None

    def on_obs(self, msg):
        self.last_obs = msg
        # Input to the marker TTL gate. The node clock is used instead of the message stamp:
        # under use_sim_time it shares the time base of the observation header, and it does
        # not depend on how the detector writes its stamps.
        now = self.get_clock().now()
        for lm in msg.landmarks:
            self.last_seen[str(lm.color).strip().lower()] = now

    def on_map(self, msg):
        self.latest_map = msg

    # ------------------------------------------------------------------ image

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                    # noqa: BLE001 - CvBridge raises broadly
            self.get_logger().warn(f'imgmsg_to_cv2 failed: {exc}', throttle_duration_sec=5.0)
            return

        self.image_count += 1
        h, w = frame.shape[:2]
        obs = self.last_obs
        if obs is not None and self.fx and abs(msg.header.stamp.sec - obs.header.stamp.sec) <= 2:
            self._draw(frame, w, h, obs)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub_img.publish(out)

    def _draw(self, frame, w, h, obs):
        for lm in obs.landmarks:
            bearing = float(lm.bearing)
            # Bearing is a camera-frame azimuth; refuse to project anything behind the lens.
            if abs(bearing) >= (math.pi / 2.0 - 1e-3):
                continue
            u = int(round(self.cx + self.fx * math.tan(bearing)))
            if not (0 <= u < w):
                continue
            colour = ROLE_BGR.get(lm.role, (255, 255, 0))
            # Box width is the projected size of a 0.4 m cube at this range.
            rng = max(0.1, float(lm.range))
            half = int(round(0.5 * self.fx * 0.4 / rng))
            half = max(4, min(half, w // 2))
            v0 = int(0.35 * h)
            v1 = int(0.75 * h)
            self._rect(frame, u - half, v0, u + half, v1, colour)

            if self.draw_labels:
                text = f'{lm.color} {rng:.2f}m {lm.confidence:.2f}'
                self._text(frame, u - half, max(12, v0 - 4), text, colour)

    @staticmethod
    def _rect(frame, x0, y0, x1, y1, colour, thickness=2):
        h, w = frame.shape[:2]
        x0, x1 = max(0, x0), min(w - 1, x1)
        y0, y1 = max(0, y0), min(h - 1, y1)
        if x1 <= x0 or y1 <= y0:
            return
        frame[y0:y0 + thickness, x0:x1] = colour
        frame[y1 - thickness + 1:y1 + 1, x0:x1] = colour
        frame[y0:y1, x0:x0 + thickness] = colour
        frame[y0:y1, x1 - thickness + 1:x1 + 1] = colour

    @staticmethod
    def _text(frame, x, y, text, colour):
        try:
            import cv2
        except ImportError:
            return
        cv2.putText(frame, text, (max(0, x), y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (max(0, x), y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)

    # ---------------------------------------------------------------- markers

    def publish_markers(self):
        lm = self.latest_map
        arr = MarkerArray()
        if lm is None or not len(lm.colors):
            # Publish a DELETEALL so stale markers disappear when the node restarts.
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            self.pub_markers.publish(arr)
            return

        stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now()
        drawn = set()
        for idx, (colour, x, y, sigma, valid) in enumerate(
                zip(lm.colors, lm.x, lm.y, lm.sigma, lm.valid)):
            # valid comes from the EKF's inited flag, which is only set after a real detection,
            # so on its own it never draws a position the camera has not seen; the marker then
            # stays. obs_ttl_sec > 0 additionally requires a recent sighting.
            draw = bool(valid) and self._recently_seen(colour, now)
            if draw:
                drawn.add(str(colour))
            marker = Marker()
            marker.header.frame_id = lm.header.frame_id or 'map'
            marker.header.stamp = stamp
            marker.ns = 'landmarks'
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD if draw else Marker.DELETE
            marker.pose.position = Point(x=float(x), y=float(y), z=self.marker_z)
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=0.4, y=0.4, z=0.4)     # every landmark is a 0.4 m cube
            rgba = ROLE_RGBA['landmark'] if draw else (0.4, 0.4, 0.4, 0.4)
            marker.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            marker.lifetime.sec = 0                          # 0 = forever, latched by RViz
            arr.markers.append(marker)

            # Uncertainty sphere grows with the EKF sigma, so map quality is visible at a glance.
            sphere = Marker()
            sphere.header = marker.header
            sphere.ns = 'sigma'
            sphere.id = idx + 1000
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD if draw else Marker.DELETE
            sphere.pose = marker.pose
            r = max(0.05, 3.0 * float(sigma))
            sphere.scale = Vector3(x=2.0 * r, y=2.0 * r, z=2.0 * r)
            sphere.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.18)
            sphere.lifetime.sec = 0
            arr.markers.append(sphere)

            # The label is resent every round as well: a marker with lifetime 0 stays in RViz
            # forever once added, so the gate has to actively DELETE it when the cube is gone.
            label = Marker()
            label.header = marker.header
            label.ns = 'labels'
            label.id = idx + 2000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD if (self.draw_labels and draw) else Marker.DELETE
            label.pose.position = Point(x=float(x), y=float(y), z=self.marker_z + 0.5)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{colour}  sigma={float(sigma):.3f}'
            label.lifetime.sec = 0
            arr.markers.append(label)

        # Log a single line only when the set changes, so whether the visualiser runs ahead of
        # the estimator can be checked from the log instead of by watching RViz.
        if drawn != self.drawn:
            self.drawn = drawn
            self.get_logger().info(
                'RViz 地标（%s）：%s'
                % ('最近 %.1fs 内被观测到' % self.obs_ttl_sec if self.obs_ttl_sec > 0.0
                   else '已检测到，常驻显示',
                   ', '.join(sorted(drawn)) if drawn else '无'))

        self.pub_markers.publish(arr)

    def _recently_seen(self, colour, now):
        """Whether the colour was reported on /observations within the last ``obs_ttl_sec``.

        Always true when ``obs_ttl_sec <= 0`` (the default), i.e. drawing is decided by the
        ``valid`` flag of ``/landmark_map`` alone.
        """
        if self.obs_ttl_sec <= 0.0:
            return True
        seen = self.last_seen.get(str(colour).strip().lower())
        if seen is None:
            return False
        return (now - seen).nanoseconds * 1e-9 <= self.obs_ttl_sec


def main(argv=None):
    rclpy.init(args=argv)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
