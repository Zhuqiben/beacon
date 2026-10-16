"""EKF-SLAM node: /odom prediction plus /observations update -> /robot_pose, /landmark_map,
and the map->odom TF.
"""
import math

import rclpy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from slam_interfaces.msg import LandmarkArray, LandmarkMap
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .ekf_core import (EKFSLAM, map_to_odom, quat_to_yaw, select_observations,
                       stamp_dt, yaw_to_quat)

# Normal odom -> base_link lag is one odom period (1/15 s) plus one TF timer period
# (1/20 s), about 0.12 s. Beyond this threshold odom counts as stale and must not be used
# to recompute the map->odom correction.
STALE_TF_S = 0.5


def qos_latched():
    """The landmark map is a "latest snapshot": a subscriber that starts later has to get
    the current map immediately.

    visualizer and performance_monitor both subscribe to ``/landmark_map`` with this
    profile. A publisher on the default VOLATILE would be **incompatible** with them (the
    DURABILITY policies differ) and they would receive nothing at all -- with only a WARN,
    no error, which makes it the hardest kind of silent failure to find. The other
    direction is compatible: a TRANSIENT_LOCAL publisher delivers normally to a VOLATILE
    subscriber such as landmark_map.
    """
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class SlamNode(Node):
    def __init__(self):
        super().__init__('ekf_slam')
        self.declare_parameter('select_distance_max', 5.0)
        self.declare_parameter('min_landmarks', 2)
        # Measurement convention (ekf_core.geometry_bias): the detector's range is camera
        # -> near face while the filter's z_hat is origin -> centre. The camera extrinsics
        # come from model.sdf: camera_joint (0.064, -0.065, 0.094) plus camera_rgb_joint
        # (0.005, 0.018, 0.013).
        self.declare_parameter('camera_dx', 0.069)
        self.declare_parameter('camera_dy', -0.047)
        self.declare_parameter('landmark_half_size', 0.2)
        self.declare_parameter('range_bias_fix', True)
        self.sel_max = float(self.get_parameter('select_distance_max').value)
        self.min_landmarks = int(self.get_parameter('min_landmarks').value)

        self.ekf = EKFSLAM(
            camera_dx=float(self.get_parameter('camera_dx').value),
            camera_dy=float(self.get_parameter('camera_dy').value),
            landmark_half_size=float(self.get_parameter('landmark_half_size').value),
            range_bias_fix=bool(self.get_parameter('range_bias_fix').value))
        self.last_stamp_ns = None
        self.n_frames = 0
        self.predict_only_frames = 0
        self.bad_sigma = set()          # landmark colours already warned about for sigma
        self.n_seen = 0                 # last observation count, for _report_health
        self.last_map_to_odom = None    # last valid map->odom correction (3-tuple)
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(LandmarkArray, '/observations', self.on_obs, 20)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/robot_pose', 10)
        self.map_pub = self.create_publisher(LandmarkMap, '/landmark_map', qos_latched())
        self.create_service(Empty, '/reset_slam', self.on_reset)
        self.create_timer(0.05, self.broadcast)      # 20 Hz

    # ---------------- odometry prediction ----------------
    def on_odom(self, msg):
        # dt comes from msg.header.stamp, not from self.get_clock().now(): under
        # use_sim_time the node clock is quantised to /clock, which gave dt <= 0 for a
        # large share of the 15 Hz /odom messages so they were dropped, and the dropped
        # intervals were lost forever because last was advanced unconditionally.
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        dt = stamp_dt(self.last_stamp_ns, ns)
        if dt is not None and 0.0 < dt < 0.5:
            self.ekf.predict(msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z, dt)
        self.last_stamp_ns = ns
        self._report_health()
        self.publish_pose(msg.header.stamp)
        self.publish_map(msg.header.stamp)

    def _report_health(self):
        """Log the filter's health indicators, throttled.

        Two indicators together answer the question that matters -- why the landmarks are
        not pinned down precisely:

        * ``gate`` rejection rate: a healthy filter should reject about 5 % at the 95 %
          chi-square gate. A rate near 58 % (the "valid updates < 2" case accumulating to
          hundreds of frames) points at an inconsistent measurement model or covariance
          convention rather than at "large noise";
        * ``psd`` clamp count and smallest eigenvalue: 0 means P stayed positive
          semidefinite throughout.
        """
        if self.n_seen != self.ekf.obs_seen:
            self.n_seen = self.ekf.obs_seen
            rate = (self.ekf.gated / self.n_seen) if self.n_seen else 0.0
            self.get_logger().info(
                f'滤波器健康：门限拒绝 {self.ekf.gated}/{self.n_seen} = {rate:.1%}，'
                f'整帧回滚 {self.predict_only_frames}/{self.n_frames} 帧，'
                f'正半定夹紧 {self.ekf.psd_fixes} 次'
                + (f'（最近 {self.ekf.last_psd_fix[0]} λmin={self.ekf.last_psd_fix[1]:.3e}）'
                   if self.ekf.last_psd_fix else ''),
                throttle_duration_sec=20.0)

    # ---------------- observation update ----------------
    def on_obs(self, msg):
        obs = select_observations(msg.landmarks, max_dist=self.sel_max)
        # 1) New landmarks are only initialised in this frame, never updated by the same
        #    observation.
        fresh = set()
        for o in obs:
            if o.color not in self.ekf.idx:
                continue
            if not self.ekf.inited[o.color]:
                self.ekf.init_landmark(o.color, o.range, o.bearing)
                fresh.add(o.color)
        # 2) Initialised landmarks: whole-frame update.
        #    Fewer than min_landmarks (2 by default) accepted updates means the frame only
        #    predicts and does not touch the pose.
        #    The backup point is taken after init_landmark: initialisation is not part of
        #    the pose update and must not be rolled back.
        used, applied = self.ekf.update_frame(
            [(o.color, o.range, o.bearing) for o in obs if o.color not in fresh],
            min_landmarks=self.min_landmarks)
        self.n_frames += 1
        if not applied:
            self.predict_only_frames += 1
            self.get_logger().warn(
                f'本帧有效更新 {used} < min_landmarks({self.min_landmarks})，'
                f'位姿暂时只靠里程计推算（本帧更新已整体丢弃，'
                f'累计 {self.predict_only_frames}/{self.n_frames} 帧）',
                throttle_duration_sec=10.0)

    # ---------------- reset ----------------
    def on_reset(self, _request, response):
        """``/reset_slam`` (``std_srvs/srv/Empty``): clear the landmark map and move the
        pose back to the origin.

        **The call order is a contract**: ``/reset_world`` first (the simulation side),
        then ``/reset_slam``. The other way round, /odom frames produced *before* the reset
        are still in the pipeline afterwards and push the just-zeroed filter away from the
        origin at the old velocity.

        Every item cleared here is cross-frame memory with one property in common: it does
        **not** heal with time.

        * ``ekf.reset()`` -- pose and landmarks. Without it map->odom keeps the old
          correction and the whole map is offset.
        * ``last_stamp_ns`` -- without it the first frame after the reset gets a dt equal to
          the gap from the last pre-reset frame, which is normally dropped by
          ``0 < dt < 0.5``; but a reset gap below 0.5 s would make that frame predict with
          a span that crosses the reset.
        * ``last_map_to_odom`` -- without it broadcast keeps using the pre-reset correction
          while odom is stale.
        * ``n_frames`` / ``predict_only_frames`` / ``n_seen`` -- the counters behind the
          health rate; without them the pre- and post-reset runs are mixed in one log line.
        * ``bad_sigma`` -- the alerted set; without it a colour that went bad before the
          reset would never warn again.
        """
        self.ekf.reset()
        self.last_stamp_ns = None
        self.n_frames = 0
        self.predict_only_frames = 0
        self.bad_sigma = set()
        self.n_seen = 0
        self.last_map_to_odom = None
        self.get_logger().info('EKF-SLAM 已重置：地标图清空、位姿归零、map→odom 将按新位姿重算')
        return response

    # ---------------- publishing ----------------
    def publish_pose(self, stamp):
        x, y, th = self.ekf.pose()
        p = PoseWithCovarianceStamped()
        p.header.stamp, p.header.frame_id = stamp, 'map'
        p.pose.pose.position.x, p.pose.pose.position.y = x, y
        qx, qy, qz, qw = yaw_to_quat(th)
        p.pose.pose.orientation.x, p.pose.pose.orientation.y = qx, qy
        p.pose.pose.orientation.z, p.pose.pose.orientation.w = qz, qw
        # PoseWithCovarianceStamped.covariance is a fixed-length float64[36] (a planar pose
        # only fills x/y/yaw on the diagonal, yaw at index 5). reshape(-1) over a 3x3 block
        # yields 9 elements and rclpy raises AssertionError on the length, so every frame
        # would fail here.
        cov3 = self.ekf.pose_cov()
        cov6 = np.zeros((6, 6))
        cov6[0:2, 0:2] = cov3[0:2, 0:2]
        cov6[5, 5] = cov3[2, 2]
        p.pose.covariance = cov6.reshape(-1).tolist()
        self.pose_pub.publish(p)

    def publish_map(self, stamp):
        m = LandmarkMap()
        m.header.stamp, m.header.frame_id = stamp, 'map'
        for c in self.ekf.colors:
            m.colors.append(c)
            if self.ekf.inited[c]:
                lx, ly = self.ekf.landmark(c)
                sigma = self.ekf.landmark_sigma(c)
                # A non-finite sigma means the covariance is already broken. This log line
                # is the only direct evidence: without it the symptom shows up only as a
                # crashed map_node or a planner reporting no valid path.
                if not math.isfinite(sigma) and c not in self.bad_sigma:
                    self.bad_sigma.add(c)
                    self.get_logger().error(
                        f'地标 {c} 的 sigma 非有限值（协方差失去正半定性？）'
                        f'P=[[{self.ekf.P[self.ekf.idx[c], self.ekf.idx[c]]:.3e}, '
                        f'{self.ekf.P[self.ekf.idx[c], self.ekf.idx[c] + 1]:.3e}], '
                        f'[{self.ekf.P[self.ekf.idx[c] + 1, self.ekf.idx[c]]:.3e}, '
                        f'{self.ekf.P[self.ekf.idx[c] + 1, self.ekf.idx[c] + 1]:.3e}]]')
                m.x.append(float(lx))
                m.y.append(float(ly))
                m.sigma.append(sigma)
                m.valid.append(True)
            else:
                m.x.append(0.0)
                m.y.append(0.0)
                m.sigma.append(0.0)
                m.valid.append(False)
        self.map_pub.publish(m)

    # ---------------- TF: map->odom = (map->base) composed with (odom->base)^-1 ---------
    def broadcast(self):
        try:
            t_ob = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception as e:
            # Warn on a throttle rather than once: during startup the odom frame does not
            # exist yet, so the single warning looks like noise, and a permanent runtime
            # failure would produce no further message at all while map->odom silently
            # stopped updating.
            self.get_logger().warn(
                f'lookup odom→base_link failed: {e}', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        age_s = (now - rclpy.time.Time.from_msg(t_ob.header.stamp)).nanoseconds / 1e9
        if age_s <= STALE_TF_S:
            x, y, th = self.ekf.pose()
            ob_x = t_ob.transform.translation.x
            ob_y = t_ob.transform.translation.y
            ob_th = quat_to_yaw(t_ob.transform.rotation)
            self.last_map_to_odom = map_to_odom(x, y, th, ob_x, ob_y, ob_th)
        elif self.last_map_to_odom is None:
            self.get_logger().warn(
                f'odom→base_link 数据陈旧 {age_s:.2f}s 且尚无可用修正量，暂不广播 map→odom',
                throttle_duration_sec=5.0)
            return
        else:
            # A stale odom pose must **not** be used to recompute the correction: an odom
            # sample tens of seconds old moves the whole map elsewhere. Reuse the previous
            # correction instead, which changes slowly anyway.
            self.get_logger().warn(
                f'odom→base_link 数据陈旧 {age_s:.2f}s，沿用上一次 map→odom 修正量',
                throttle_duration_sec=5.0)

        mo_x, mo_y, mo_th = self.last_map_to_odom

        t = TransformStamped()
        # The stamp is the **publish time**, not ``t_ob.header.stamp``: the odom ->
        # base_link sample is itself 0.07-0.12 s older than its publication, so carrying the
        # sample stamp over to map->odom makes this transform late by construction. Nav2's
        # controller_server.isGoalReached() uses the goal pose stamp to transform the goal
        # from map to odom (local_costmap.global_frame=odom); nav_2d_utils::transformPose
        # **returns false without writing the output** on the timeout branch, and the caller
        # ignores the return value, so the goal becomes the default (0, 0, 0) and the robot
        # declaring "Reached the goal!" while sitting at the odom origin looks like a
        # success. Stamping the current time removes that artificial lateness.
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x = mo_x
        t.transform.translation.y = mo_y
        qx, qy, qz, qw = yaw_to_quat(mo_th)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = SlamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
