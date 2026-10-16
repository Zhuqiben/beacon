#!/usr/bin/env python3
"""Pipeline latency and localisation-accuracy monitor.

Two jobs:

1. Latency - for each pipeline stage, the age of the newest message,
   (now - header.stamp). Requires use_sim_time so that "now" is the same clock the
   sensors stamp with; otherwise the numbers are meaningless.
2. Accuracy - planar distance between /robot_pose (EKF) and /gazebo/model_states
   (ground truth). Ground truth is for scoring only and never enters the control loop;
   it is read here and in benchmark.py and nowhere else.

Publishes
---------
/performance_monitor   std_msgs/Float32MultiArray  flattened [stage_id, ms, ...]
/performance_report    std_msgs/String             human-readable summary, 1 Hz

Stage ids (kept stable so benchmark.py can label them):
    0 detection   /observations
    1 slam        /landmark_map
    2 planning    /plan
    3 pose        /robot_pose
"""

import math
import statistics
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, String

from slam_interfaces.msg import LandmarkArray, LandmarkMap


STAGE_DETECTION = 0
STAGE_SLAM = 1
STAGE_PLANNING = 2
STAGE_POSE = 3
STAGE_NAMES = {
    STAGE_DETECTION: 'detection',
    STAGE_SLAM: 'slam',
    STAGE_PLANNING: 'planning',
    STAGE_POSE: 'pose',
}


def qos_latched():
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.declare_parameter('model_name', 'waffle')
        self.declare_parameter('report_period_sec', 1.0)
        self.declare_parameter('window', 200)
        self.model_name = str(self.get_parameter('model_name').value)
        self.window = int(self.get_parameter('window').value)
        period = float(self.get_parameter('report_period_sec').value)

        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is false: latency figures will be garbage. '
                'Launch with use_sim_time:=true.')

        self.latency = {sid: [] for sid in STAGE_NAMES}
        self.errs = []
        self.gt = None
        self.est = None
        self.received = {sid: 0 for sid in STAGE_NAMES}

        self.pub_latency = self.create_publisher(Float32MultiArray, '/performance_monitor', 10)
        self.pub_report = self.create_publisher(String, '/performance_report', 10)

        self.create_subscription(LandmarkArray, '/observations', self.on_detection, 10)
        self.create_subscription(LandmarkMap, '/landmark_map', self.on_slam, qos_latched())
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/robot_pose', self.on_pose, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_gt, 10)
        self.create_timer(period, self.report)
        self.create_timer(0.1, self.sample_error)

        self.get_logger().info(
            f'performance_monitor up (model={self.model_name}, window={self.window})')

    # ----------------------------------------------------------------- stages

    def _stamp_ms(self, header):
        # Sim time can start at 0, and a stamp of 0 means "unset" - ignore those.
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            header.stamp.sec + header.stamp.nanosec * 1e-9)
        return age * 1000.0

    def _push(self, stage, ms):
        if ms is None:
            return
        self.received[stage] += 1
        # Negative ages mean a clock-domain mismatch, not a fast pipeline.
        if ms < -1.0:
            return
        buf = self.latency[stage]
        buf.append(max(0.0, ms))
        if len(buf) > self.window:
            del buf[:len(buf) - self.window]

    def on_detection(self, msg):
        self._push(STAGE_DETECTION, self._stamp_ms(msg.header))

    def on_slam(self, msg):
        self._push(STAGE_SLAM, self._stamp_ms(msg.header))

    def on_plan(self, msg):
        self._push(STAGE_PLANNING, self._stamp_ms(msg.header))

    def on_pose(self, msg):
        self._push(STAGE_POSE, self._stamp_ms(msg.header))
        p = msg.pose.pose.position
        self.est = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    # ----------------------------------------------------------- ground truth

    def on_gt(self, msg):
        if self.model_name not in msg.name:
            return
        i = msg.name.index(self.model_name)
        p = msg.pose[i].position
        self.gt = (p.x, p.y, yaw_of(msg.pose[i].orientation))

    def sample_error(self):
        if self.gt is None or self.est is None:
            return
        self.errs.append(math.hypot(self.gt[0] - self.est[0], self.gt[1] - self.est[1]))
        if len(self.errs) > 10000:
            del self.errs[:5000]

    # ---------------------------------------------------------------- output

    def report(self):
        flat = []
        parts = []
        for sid in sorted(self.latency):
            buf = self.latency[sid]
            if not buf:
                parts.append(f'{STAGE_NAMES[sid]}=n/a')
                continue
            mean = statistics.fmean(buf)
            p95 = sorted(buf)[min(len(buf) - 1, int(0.95 * len(buf)))]
            flat.extend([float(sid), mean])
            parts.append(f'{STAGE_NAMES[sid]}={mean:.1f}ms(p95 {p95:.1f})')

        if flat:
            out = Float32MultiArray()
            out.data = flat
            self.pub_latency.publish(out)

        if self.errs:
            e = self.errs
            parts.append(f'loc_err mean={statistics.fmean(e):.3f}m '
                         f'rms={math.sqrt(sum(v * v for v in e) / len(e)):.3f}m '
                         f'max={max(e):.3f}m n={len(e)}')
        else:
            parts.append('loc_err=n/a')

        text = '  '.join(parts)
        msg = String()
        msg.data = text
        self.pub_report.publish(msg)
        self.get_logger().info(text)


def main(argv=None):
    rclpy.init(args=argv)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
