#!/usr/bin/env python3
"""Freshness relay for /tf.

The `turtlebot3_diff_drive` plugin (model.sdf) delivers the odom->base_footprint edge to
a RELIABLE reader in bursts: nothing for about five seconds, then the whole backlog at
once. A reliable tf2 Buffer therefore sees a stamp that is seconds old, while a
BEST_EFFORT reader on the same topic stays fresh. The frame counts are identical on both
paths, so nothing is lost and tf2 itself is not at fault; raising the reader depth does
not help either, which is why the fix belongs on the sender side. Forwarding the edge
through this node brings the reliable lag down to the best-effort lag.

This node subscribes to /tf with BEST_EFFORT and re-publishes the affected edges back onto
/tf with RELIABLE, so every existing consumer is cured without a single remapping. It
re-publishes onto /tf itself rather than a new topic because /tf consumers are already
RELIABLE and mostly upstream (Nav2's C++ tf2 Buffer, the tf2_ros Python
TransformListener default QoS) and cannot be switched to BEST_EFFORT from a launch file
-- `--qos-reliability` is not an rcl argument (UnknownROSArgsError) -- and because
ekf_slam/slam_node.py also consumes /tf, so pointing only Nav2 at a new topic would still
leave map->odom computed from stale odometry.

Only the frames listed in `relay_frames` are forwarded, and only when their stamp is newer
than the last one forwarded. The default is the one edge whose publisher is unique, so no
healthy edge is duplicated and /tf traffic grows by about 15 Hz instead of about 64 Hz.
/tf_static is deliberately untouched: static transforms are TRANSIENT_LOCAL at about 0 Hz
and are not affected.

    ros2 run sim_bringup tf_relay.py
    ros2 run sim_bringup tf_relay.py --ros-args -p relay_frames:="['odom:base_footprint']"
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage

DEFAULT_FRAMES = ['odom:base_footprint']


def _qos(depth, reliability, durability=DurabilityPolicy.VOLATILE):
    return QoSProfile(
        depth=int(depth),
        history=HistoryPolicy.KEEP_LAST,
        reliability=reliability,
        durability=durability,
    )


class TfRelay(Node):
    def __init__(self):
        super().__init__(
            'tf_relay',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.declare_parameter('relay_frames', DEFAULT_FRAMES)
        self.declare_parameter('in_depth', 200)
        self.declare_parameter('out_depth', 1000)
        self.declare_parameter('report_period_s', 10.0)

        frames = self.get_parameter('relay_frames').value or []
        self.want = set()
        for item in frames:
            parent, _, child = str(item).partition(':')
            if parent and child:
                self.want.add((parent, child))
        if not self.want:
            self.get_logger().error('relay_frames is empty -- nothing to relay')

        self.in_depth = int(self.get_parameter('in_depth').value)
        self.out_depth = int(self.get_parameter('out_depth').value)

        self.newest = {edge: None for edge in self.want}
        self.stats = {edge: {'in': 0, 'out': 0, 'dup': 0} for edge in self.want}
        self.n_msgs_in = 0
        self.n_msgs_out = 0

        self.pub = self.create_publisher(
            TFMessage, '/tf',
            _qos(self.out_depth, ReliabilityPolicy.RELIABLE))
        # NOTE: rclpy (Humble) has no ignore_local_publications, so the relay also
        # receives its own re-publication.  on_tf drops it because the stamp is not
        # newer than the one just forwarded, so there is no feedback loop; the only
        # cost is that `in` counts roughly 2x the plugin's publish rate and `dup`
        # mirrors it.
        self.create_subscription(
            TFMessage, '/tf', self.on_tf,
            _qos(self.in_depth, ReliabilityPolicy.BEST_EFFORT))

        period = float(self.get_parameter('report_period_s').value)
        if period > 0.0:
            self.create_timer(period, self.report)

        self.get_logger().info(
            'tf_relay up: frames=%s | in BEST_EFFORT depth=%d -> out RELIABLE depth=%d'
            % (sorted('%s->%s' % e for e in self.want), self.in_depth, self.out_depth))

    @staticmethod
    def _stamp(tf):
        return tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9

    def on_tf(self, msg):
        self.n_msgs_in += 1
        out = TFMessage()
        for tf in msg.transforms:
            edge = (tf.header.frame_id, tf.child_frame_id)
            if edge not in self.want:
                continue
            self.stats[edge]['in'] += 1
            stamp = self._stamp(tf)
            last = self.newest[edge]
            if last is not None and stamp <= last:
                self.stats[edge]['dup'] += 1
                continue
            self.newest[edge] = stamp
            self.stats[edge]['out'] += 1
            out.transforms.append(tf)
        if not out.transforms:
            return
        self.pub.publish(out)
        self.n_msgs_out += 1

    def report(self):
        now = self.get_clock().now()
        stamp_now = now.nanoseconds / 1e9 if now.nanoseconds else None
        parts = []
        for edge in sorted(self.want):
            st = self.stats[edge]
            age = 'n/a'
            if stamp_now is not None and self.newest[edge] is not None:
                age = '%.3fs' % (stamp_now - self.newest[edge])
            parts.append('%s->%s in=%d out=%d dup=%d age=%s'
                         % (edge[0], edge[1], st['in'], st['out'], st['dup'], age))
        self.get_logger().info(
            'tf messages in=%d out=%d | %s' % (self.n_msgs_in, self.n_msgs_out, ' | '.join(parts)))


def main(args=None):
    rclpy.init(args=args)
    node = TfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
