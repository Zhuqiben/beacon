#!/usr/bin/env python3
"""GoToBlock action server: SEARCHING -> NAVIGATING -> APPROACHING -> DONE / ABORTED.

Navigation drives the Nav2 ``navigate_to_pose`` **action**, not the ``/goal_pose`` topic:
the topic form returns no result code and cannot support "failure -> clear costmaps ->
retry". A ``MultiThreadedExecutor`` is required because ``execute()`` is long running.

Collision counting keeps one tally per goal, based on the Gazebo contact sensor on
``/bumper_states``. All three exits (success, failure, cancel) pass through
``_tally_finish()``, which feeds ``GoToBlock.Result.message``, ``/collision_stats`` and
the log.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String
from slam_interfaces.action import GoToBlock
from slam_interfaces.msg import LandmarkMap

from goal_selector.collision_tally import (
    CollisionTally, blocks_in_contact, describe_summary)
from goal_selector.fsm import StateMachine

# State strings returned to the client; must match the values in fsm.FEEDBACK_STATES.
SEARCHING, NAVIGATING, APPROACHING = 'SEARCHING', 'NAVIGATING', 'APPROACHING'

# Extra setback applied to the goal along the landmark->robot direction on every retry, to
# climb back out of the inflation layer (see _stand_off_point).
RETRY_STANDOFF_STEP = 0.1

# A goal must clear the landmark inflation ring *and* the robot radius: InflationLayer marks
# cells within robot_radius outside the lethal ring as inscribed(253) and SmacPlanner2D treats
# cost >= 253 as an obstacle, so a goal inside that band is reported as "Goal in collision" or
# silently shifted. (0.22 robot_radius + 0.04 discretisation margin.)
ROBOT_CLEARANCE = 0.26

# Lower bound on the goal distance from the landmark centre: circumscribed radius of the
# 0.4 m cube (0.2*sqrt(2) ~ 0.283 m) plus robot_radius 0.22 plus a margin.
MIN_STANDOFF = 0.51

# robot_radius as configured in nav2_params.yaml; see _parking_floor.
ROBOT_RADIUS = 0.22

# Circumscribed radius of the 0.4 m landmark cube.
LANDMARK_HALF_DIAG = 0.283

# Front edge of collision_monitor's StopPolygon (0-0.3 m ahead of base_link). Laser points
# inside it make the monitor publish zero velocity on /cmd_vel, the channel used to spin.
STOP_POLYGON_REACH = 0.30

# Docking window for the final standoff, robot centre to landmark centre:
#   0.583 m = LANDMARK_HALF_DIAG + STOP_POLYGON_REACH -- any closer and a cube corner enters
#             the StopPolygon, which throttles the in-place search that follows;
#   0.750 m = approach_distance 0.6 plus a 0.15 m margin. The midpoint is the target, leaving
# about 0.083 m per side for the EKF distance bias.
DOCK_STANDOFF = 0.665

# Consecutive samples that must satisfy the arrival test (10 Hz loop), filtering EKF jitter.
# 3 samples = 0.3 s.
ARRIVAL_CONFIRM = 3

# Nav2's SUCCEEDED is not used as the arrival decision: SimpleGoalChecker (xy_goal_tolerance
# 0.25 m) reports SUCCEEDED the moment the robot enters the goal ball, so the stopping point
# lands at "commanded standoff + 0.25 m" whatever sigma is. This node closes the loop itself.
# Maximum slack: xy_goal_tolerance 0.25 m plus 0.1 m of EKF pose drift; beyond it the
# parking-ring semantics are broken and the goal is reported as failed.
ARRIVAL_SLACK = 0.35

# When Nav2 sits on its own goal ring without reporting success, this node takes over instead
# of betting the goal on SUCCEEDED -- a stalled in-place rotation is a known physical failure
# mode here. The takeover performs what Nav2 was supposed to do: creep to stop_at.
# The window covers the outer edge of the tolerance ball (xy_goal_tolerance 0.25 m plus the
# EKF distance bias); below it the fallback would miss the deadlock in which the robot parks
# just outside the ball and the final yaw never converges.
STANDOFF_SELF_SLACK = 0.45     # m: treated as "standing on the ring" (must exceed 0.25)
STANDOFF_SELF_N = 60           # samples: 10 Hz x 6 s (progress checker runs every 10 s)

# Minimum gain required to re-visit a landmark and recompute the standoff from a fresh sigma;
# below it sigma counts as converged and the extra leg is not worth the budget.
REFINE_MIN_GAIN = 0.05

# Time kept in reserve for the last navigation leg and for convergence.
REFINE_TIME_RESERVE = 8.0

# The arrival tests above cannot catch tolerance-ball overshoot: Nav2 stops the robot the
# instant it enters the ball, so the 10 Hz loop only ever sees a stopped robot at about
# setpoint + 0.25 m. The last 0.25 m is driven by this node with a Twist on /cmd_vel,
# closed-loop on the estimated distance and targeting _dock_target(). Publishing there is
# safe because navigation_launch.py remaps cmd_vel -> cmd_vel_nav for controller_server only,
# so behavior_server spins on /cmd_vel directly and both users share that channel in turn.
CREEP_SPEED = 0.08         # m/s: slowest approach speed, below a quarter of max_vel_x
CREEP_PERIOD = 0.05        # s: 20 Hz servo
CREEP_BUDGET = 0.80        # m: lower bound of the travel budget, see CREEP_BUDGET_GAIN
CREEP_BUDGET_GAIN = 1.5    # x: budget scales with the remaining distance to dock
CREEP_BUDGET_MAX = 1.60    # m: hard cap on the budget (1.60 m / 0.08 m/s = 20 s)
CREEP_TIMEOUT = 25.0       # s: hard timeout (20 s of travel plus a 25 % margin)
CREEP_ALIGN_MAX = 0.35     # rad: straighten up in place beyond this heading error
CREEP_WZ_GAIN = 1.5        # P gain for heading hold (hold only, no large turns)
CREEP_WZ_MAX = 0.5         # rad/s
CREEP_STALL_TIME = 1.5     # s: give up once the estimated distance stops dropping for
                           #    this long (timed during translation only)  # noqa: E114,E116
CREEP_SETTLE = 0.3         # s: settle before the final distance reading

# After docking the robot must reverse itself out of the rejection circle, otherwise the next
# leg cannot be planned: DOCK_STANDOFF lies inside _parking_floor(sigma) of the landmark it
# just docked on, so its own cell becomes lethal and the next plan fails with "Starting point
# in lethal space". Nav2 recovery cannot fix that (behavior_server's backup finds the
# footprint in a lethal cell every cycle and never moves); the travel is
# _parking_floor(sigma) + GUARD_MARGIN - current distance, capped at BACKOFF_DIST_MAX.
BACKOFF_ROUNDS = 2          # at most two rounds (the second one tops up by distance)
# Shared margin of the trigger test and the retreat travel: it must exceed the planner's
# rejection band _parking_floor(sigma) + eps (0.05 m costmap discretisation plus ~0.1 m of
# EKF lateral error), otherwise a blind band reports "safe start" while planning still fails.
GUARD_MARGIN = 0.20        # m: margin for the rejection circle and for the retreat travel
BACKOFF_DIST_MAX = 0.60    # m: single retreat cap (covers inflation up to sigma 0.22)
BACKOFF_SPEED = 0.08       # m/s: same slowest speed as CREEP_SPEED, in reverse
BACKOFF_PERIOD = 0.05      # s: 20 Hz servo
BACKOFF_TIMEOUT = 10.0     # s: hard timeout (0.60 m / 0.08 m/s = 7.5 s plus margin)
BACKOFF_STALL_TIME = 1.5   # s: give up once the distance to the landmark stops growing
BACKOFF_WZ_GAIN = 1.5      # heading-hold P gain while reversing (same sign as forward)
BACKOFF_WZ_MAX = 0.5       # rad/s
BACKOFF_SETTLE = 0.3       # s: settle before the final distance reading

# SEARCHING rotates by itself, otherwise a block that is invisible at the start is never
# acquired: the camera covers about 110 deg, so blocks near +/-135 deg from the spawn heading
# stay outside the field of view of a pure busy wait. After lost_threshold_sec without an
# observation the node starts a slow in-place sweep, until the target appears, a full turn is
# done or the budget runs out. 0.5 rad/s keeps a block about 2.2 s inside the field of view,
# roughly 13 frames at the detector's 158 ms p95, enough for slam_node to pass min_landmarks.
SEARCH_SPIN_WZ = 0.5        # rad/s: in-place sweep rate (positive = counter-clockwise)
SEARCH_SPIN_PERIOD = 0.05   # s: 20 Hz servo (same as CREEP_PERIOD / BACKOFF_PERIOD)
SEARCH_SWEEP_MAX = 7.0      # rad ~ 401 deg: stop after sweeping this much
SEARCH_SPIN_STALL = 4.0     # s: declare "cannot turn" once the swept angle stops growing
SEARCH_SPIN_EPS = 0.005     # rad: minimum progress per sample (yaw noise is about 0.001)
# Budget kept for NAVIGATING/APPROACHING: the accumulated sweep stops at timeout - reserve.
SEARCH_TIME_RESERVE = 20.0  # s: absolute cap on the reserved budget
SEARCH_RESERVE_FRAC = 0.40  # reserved fraction of a short timeout, whichever is smaller
# The reserve grows with a long timeout, because an active search finds the target later:
# 60 s goals keep 20 s, 120 s goals get 30 s.
SEARCH_RESERVE_GROW_FRAC = 0.25  # lower bound of the reserve: max(..., timeout * this)
SEARCH_FB_PERIOD = 0.2      # s: feedback still goes out at 5 Hz

# If a single in-place turn does not find the target, go and look somewhere else: the blocks
# wander over the map, so a given spot may simply not see them. Moving to a known block A is
# better, because A proves the area contains visible things; the target is searched for on the
# way and A is swept once reached.
# Relocation uses Nav2 (navigate_to_pose) instead of /cmd_vel, because the way there contains
# other blocks and their inflation rings; this node stays off /cmd_vel during relocation, so
# the two never compete for the velocity channel. "Looking on the way" is free: as soon as
# the target enters the map the goal is cancelled and normal navigation resumes.
SEARCH_RELOC_ENABLE = True      # master switch for active relocation search
SEARCH_RELOC_MAX = 2            # known blocks visited per goal at most
SEARCH_RELOC_MIN_BUDGET = 8.0   # s: do not relocate with less search budget than this
SEARCH_RELOC_LEG_MAX = 15.0     # s: hard cap on one relocation leg (3 to 4 m at 0.26 m/s)
SEARCH_RELOC_MIN_GAP = 0.4      # m: minimum travel to A for the detour to be worthwhile
SEARCH_RELOC_LOG_PERIOD = 5.0   # s: progress log cadence during relocation
# Viewpoints are limited to these colours: the landmark map also holds black and white entries
# from detector noise at unreliable positions; the target and visited blocks are excluded.
SEARCH_RELOC_ALLOW = ('red', 'green', 'blue', 'yellow')
# At 0.35 rad/s a full turn takes 20 s, half the budget of a 60 s goal, so the sweep is capped
# when a destination is available; the remaining angles are covered after arriving at A.
SEARCH_SPIN_SEC_MAX = 12.0      # s: cap on one in-place sweep when relocation is possible


class GoalSelector(Node):
    """SEARCHING -> NAVIGATING -> APPROACHING -> DONE / ABORTED."""

    def __init__(self):
        super().__init__('goal_selector')
        # rclpy already declares use_sim_time from the parameter override; keep a fallback
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('approach_distance', 0.6)
        self.declare_parameter('lost_threshold_sec', 5.0)
        self.declare_parameter('max_retries', 2)
        # Runtime switches for the relocation search, defaulting to the SEARCH_RELOC_*
        # constants so that behaviour can be compared without recompiling:
        #   ros2 param set /goal_selector search_reloc false
        #   ros2 param set /goal_selector search_reloc_max 3
        # Both are read inside execute(), so a change applies to the next goal.
        self.declare_parameter('search_reloc', SEARCH_RELOC_ENABLE)
        self.declare_parameter('search_reloc_max', SEARCH_RELOC_MAX)
        # Cap on how often the standoff is recomputed. sigma is derived from the current
        # observation distance and is necessarily too pessimistic from far away, so the
        # standoff must be allowed to be recomputed once the robot is close.
        self.declare_parameter('max_approach_refinements', 2)
        # The same three inflation parameters that landmark_map.map_node uses; they must
        # match. map_node sizes the lethal circle on /map with them, this node decides how
        # far outside that circle the goal has to be. Changing one side alone puts the goal
        # inside the lethal circle.
        self.declare_parameter('inflation_base', 0.30)
        self.declare_parameter('sigma_k', 0.0)
        self.declare_parameter('max_inflation', 1.0)
        self.approach = float(self.get_parameter('approach_distance').value)
        self.lost_sec = float(self.get_parameter('lost_threshold_sec').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.max_refine = int(self.get_parameter('max_approach_refinements').value)
        self.inflation_base = float(self.get_parameter('inflation_base').value)
        self.sigma_k = float(self.get_parameter('sigma_k').value)
        self.max_inflation = float(self.get_parameter('max_inflation').value)

        self.map = None
        self.pose = None
        self._nav_distance = -1.0
        self._nav_status = None
        self._nav_done = threading.Event()
        # Incremented on every Nav2 goal; results arriving late from an older goal are
        # discarded through it, otherwise the cancel result of the previous goal would wake
        # the next goal up early.
        self._nav_gen = 0

        # ---- Collision tally ----
        # The ground truth is the Gazebo contact sensor rather than a distance threshold;
        # see the collision_tally module docstring.
        self.declare_parameter('collision_topic', '/bumper_states')
        self.declare_parameter('collision_stats_topic', '/collision_stats')
        self.declare_parameter('collision_rearm_sec', 0.35)
        self._collision_topic = str(self.get_parameter('collision_topic').value)
        self._tally = CollisionTally(
            rearm_sec=float(self.get_parameter('collision_rearm_sec').value))
        self._tally_lock = threading.Lock()
        self._contacts_seen = 0

        self.create_subscription(LandmarkMap, '/landmark_map', self._on_map, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/robot_pose', self._on_pose, 10)
        # The sensor update rate is 50 Hz; the queue depth is forced to 1 because a contact
        # state is an instantaneous quantity and a backlog is meaningless.
        self.create_subscription(ContactsState, self._collision_topic, self._on_contacts, 1)
        self._stats_pub = self.create_publisher(
            String, str(self.get_parameter('collision_stats_topic').value), 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Direct velocity channel for the final creep, see the CREEP_* constants
        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # Costmap-clearing clients for the retry path, created once here so that a retry
        # does not rediscover the service.
        # The service type must be nav2_msgs/ClearEntireCostmap, not std_srvs/Empty: two
        # clients with the same name but different types do not raise in rclpy, they only
        # fail silently in the graph, so wait_for_service would keep timing out and the
        # costmap would never actually be cleared.
        self._clear_clients = [
            self.create_client(ClearEntireCostmap,
                               '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap,
                               '/local_costmap/clear_entirely_local_costmap')]
        self.srv = ActionServer(self, GoToBlock, 'goto_block',
                                execute_callback=self.execute,
                                goal_callback=self._on_goal,
                                cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info(
            f'goal_selector: /goto_block 就绪，approach={self.approach:.2f} m，'
            f'lost_threshold={self.lost_sec:.1f}s，max_retries={self.max_retries}，'
            f'主动搜索='
            f'{"开启" if self.get_parameter("search_reloc").value else "关闭"}'
            f'（原地扫不到就搬去已知色块，最多 '
            f'{int(self.get_parameter("search_reloc_max").value)} 段 × '
            f'≤{SEARCH_RELOC_LEG_MAX:.0f}s）')

    # ---------------- Subscriptions and callbacks ----------------
    def _on_goal(self, req):
        """Reject until a map has arrived, otherwise SEARCHING just waits for the timeout."""
        if self.map is None:
            self.get_logger().warn('尚未收到 /landmark_map，拒绝目标')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_map(self, msg):
        self.map = msg

    def _on_pose(self, msg):
        self.pose = msg.pose.pose

    # ---------------- Collision tally (robot vs block, one per goal) ----------------
    def _on_contacts(self, msg):
        """``/bumper_states`` (Gazebo contact sensor) -> collision tally.

        Fed on every frame, even with no goal running: ``CollisionTally.reset()`` relies on
        ``touching`` to tell what was already pressed at goal start, and without this feed a
        contact left over from the previous goal would be counted into the new one.

        Only rising edges count; continuous contact arrives on every frame and is
        deduplicated inside the tally.
        """
        names = []
        for state in msg.states:
            names.append(state.collision1_name)
            names.append(state.collision2_name)
        touched = blocks_in_contact(names)
        now = time.monotonic()
        with self._tally_lock:
            self._contacts_seen += 1
            before = self._tally.total
            self._tally.observe(touched, now)
            # Publish as soon as a new hit is counted, so that the running total can be
            # watched live with ros2 topic echo.
            live = (self._tally.summary(now) if self._tally.total > before else None)
        if live is not None:
            live['phase'] = 'during'
            self._stats_pub.publish(String(data=json.dumps(live, ensure_ascii=False)))

    def _tally_begin(self, color):
        """Start of a new goal: clear the running tally. Once per /goto_block goal."""
        with self._tally_lock:
            previous = self._tally.stop(time.monotonic()) if self._tally.active else None
            trip = self._tally.reset(color, time.monotonic())
            seen = self._contacts_seen
        if previous is not None:
            # execute() can be interrupted before it closes the tally, so close it
            # here as well; otherwise that leg would vanish without a trace.
            self.get_logger().warn(
                f"碰撞统计：第 {previous['trip']} 段没有正常封账就被新目标打断，"
                f"按已累计的 {previous['collisions']} 次记"
                f"（{describe_summary(previous)}）")
        self.get_logger().info(
            f'碰撞统计：第 {trip} 段开始（目标 {color or "未指定"}），计数已清零')
        if seen == 0:
            self.get_logger().warn(
                f'碰撞统计：还没收到 {self._collision_topic}，'
                f'若本轮计数一直为 0 请查 waffle 的接触传感器是否已加载')

    def _tally_finish(self, ok):
        """End of a goal (success, failure or cancel): close, log, publish, return a
        summary string.
        """
        with self._tally_lock:
            summary = self._tally.stop(time.monotonic())
            seen = self._contacts_seen
        summary['success'] = bool(ok)
        if seen == 0:
            # With no data source, "0 collisions" must not be reported as a conclusion.
            text = f'碰撞统计不可用（未收到 {self._collision_topic}）'
            self.get_logger().warn(text)
        else:
            text = describe_summary(summary)
            self.get_logger().info(
                f"碰撞统计：第 {summary['trip']} 段结束"
                f"（目标 {summary['target_color'] or '未指定'}，"
                f"{'成功' if ok else '未成功'}）→ {text}；"
                f"本段耗时 {summary['elapsed_sec']:.1f}s")
        summary['phase'] = 'end'
        summary['available'] = bool(seen)
        self._stats_pub.publish(String(data=json.dumps(summary, ensure_ascii=False)))
        return text

    def _on_nav_feedback(self, fb_msg, gen):
        """Nav2 feedback; as with the result callback, late feedback from an older
        goal is dropped.
        """
        if gen != self._nav_gen:
            return
        self._nav_distance = float(fb_msg.feedback.distance_remaining)

    def _on_nav_result(self, fut, gen):
        """Nav2 result; a gen mismatch means a late result from the previous goal."""
        if gen != self._nav_gen:
            return
        try:
            self._nav_status = fut.result().status
        except Exception as e:                 # result() raises when cancelled
            self.get_logger().warn(f'Nav2 结果异常: {e}')
            self._nav_status = None
        self._nav_done.set()

    def target_xy(self, color):
        """Look a target colour up by index in the parallel colors/x/y/sigma/valid
        arrays.

        Returns ``(x, y, sigma)``; None when unobserved or invalid. sigma comes along
        because it sets the inflation radius, and hence how far the goal stands off.
        """
        if self.map is None:
            return None
        for c, x, y, s, v in zip(self.map.colors, self.map.x, self.map.y,
                                 self.map.sigma, self.map.valid):
            if c == color and v:
                return float(x), float(y), float(s)
        return None

    def _distance_to(self, x, y):
        """Distance from the current pose to (x, y); None until /robot_pose arrives."""
        if self.pose is None:
            return None
        return math.hypot(x - self.pose.position.x, y - self.pose.position.y)

    def _landmark_radius(self, sigma):
        """Lethal inflation radius of a landmark; same formula as
        ``landmark_map.map_node``.

        The two nodes must agree: ``map_node`` uses it to decide how large a lethal
        disc to paint on ``/map``, this node uses it to decide how far outside that
        disc the goal has to stand off. Both declare the same parameters
        (inflation_base / sigma_k / max_inflation).

        With ``sigma_k`` = 0 the radius is simply ``inflation_base`` = 0.30 m, the
        circumscribed circle of the 0.4 m cube plus margin. The sigma term is kept in
        the formula: raising ``sigma_k`` makes the disc track the estimate again,
        without touching either node.

        A non-finite ``sigma`` is treated as 0. Letting the goal fall inside the disc
        and having Nav2 handle it is preferable to passing NaN into a goal pose,
        which produces an unexplainable Nav2 goal.
        """
        if not math.isfinite(sigma):
            return self.inflation_base
        return min(self.inflation_base + self.sigma_k * max(0.0, sigma),
                   self.max_inflation)

    def _parking_floor(self, sigma):
        """Closest distance to a landmark the planner lets the robot centre reach:
        inflation radius + ``ROBOT_RADIUS``.

        This is the boundary of the cells SmacPlanner2D / DWB still accept; any closer
        is ``inscribed(253)`` and the planner either reports "Goal in collision" or
        silently shifts the goal. It therefore only answers "can one more approach be
        planned", it is **not** the docking target:

            _parking_floor(sigma) = inflation_base + sigma_k * sigma + 0.22

        With a fixed disc it is the constant 0.30 + 0.22 = 0.52 m, comfortably inside
        the 0.75 m ceiling. The sigma term only covers map uncertainty for the
        **planner**; it does not mean the robot physically cannot get closer. The
        docking target is `_dock_target`.
        """
        return self._landmark_radius(sigma) + ROBOT_RADIUS

    def _dock_target(self, used_off):
        """Docking target for the final leg: actually honour ``approach_distance``.

        ``approach_distance`` means "the requested approach distance". The planning
        standoff ``used_off`` is ``inflate_r + ROBOT_CLEARANCE``, which at
        sigma = 0.14 m reaches 0.88 m, **farther than the requested approach**, so the
        final leg has to keep closing in. It cannot just drive as close as possible
        either: the usable docking distance is a narrow window (derivation in
        ``DOCK_STANDOFF``):

            0.583 m (cube corner just leaves the StopPolygon) <= d <= 0.75 m

        Take the middle of that window, ``DOCK_STANDOFF``, and let a larger requested
        ``approach`` win. ``min(used_off, ...)`` is only a floor for the case where the
        planning standoff is already closer than the docking target.
        """
        return min(used_off, max(DOCK_STANDOFF, self.approach))

    def _yaw_now(self):
        """Current heading in rad; None until /robot_pose arrives."""
        if self.pose is None:
            return None
        q = self.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _publish_twist(self, vx, wz):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self._cmd_vel.publish(msg)

    def _halt(self, times=4):
        """Publish zero velocity a few times: /cmd_vel has no smoothing that would
        absorb a single dropped message.
        """
        for _ in range(times):
            self._publish_twist(0.0, 0.0)
            time.sleep(CREEP_PERIOD)

    def _final_creep(self, x, y, stop_at, t0, timeout):
        """Final autonomous creep: walk the last stretch that the Nav2 tolerance
        sphere leaves behind.

        Returns ``(entered the docking ring, estimated distance at the end)``. The
        reason is in the comment next to ``CREEP_SPEED``:
        ``SimpleGoalChecker.xy_goal_tolerance = 0.25`` stops the robot at "standoff +
        0.25 m", while the tolerance only allows 0.15 m, so this node has to cover the
        difference.

        ``stop_at`` comes from ``_dock_target``. Only the EKF pose and landmark
        estimate are used; the speed is the lowest notch, and travel
        (``CREEP_BUDGET`` .. ``CREEP_BUDGET_MAX``), time (``CREEP_TIMEOUT``) and "the
        estimate stops shrinking while translating" are three hard caps. Any of them
        triggers a stop with an honest report instead of a pretended arrival. The
        stall timer only runs while translating: turning in place does not reduce the
        radial distance, see ``CREEP_STALL_TIME``.
        """
        d = self._distance_to(x, y)
        if d is None or d <= stop_at:
            return (d is not None), d
        t_end = min(time.monotonic() + CREEP_TIMEOUT, t0 + timeout - 1.0)
        budget = min(CREEP_BUDGET_MAX, max(CREEP_BUDGET, (d - stop_at) * CREEP_BUDGET_GAIN))
        self.get_logger().info(
            f'末段自主贴近：估距 {d:.2f} m → 停靠目标 {stop_at:.2f} m，'
            f'用 /cmd_vel 走完 Nav2 容差球留下的 {d - stop_at:.2f} m'
            f'（cmd {CREEP_SPEED:.2f} m/s，行程上限 {budget:.2f} m）')
        travel, last, stall_t = 0.0, d, None
        try:
            while time.monotonic() < t_end:
                d = self._distance_to(x, y)
                yaw = self._yaw_now()
                if d is None or yaw is None:
                    break
                if d <= stop_at:
                    break
                if travel >= budget:
                    self.get_logger().warn(
                        f'末段贴近行程达上限 {budget:.2f} m（估距仍有 {d:.2f} m），停止')
                    break
                bearing = math.atan2(y - self.pose.position.y, x - self.pose.position.x)
                err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
                aligning = abs(err) > CREEP_ALIGN_MAX
                if aligning:
                    # Turn in place first: driving at an angle shrinks the radial
                    # distance more slowly and can graze the inflated disc.
                    self._publish_twist(0.0, math.copysign(CREEP_WZ_MAX, err))
                else:
                    self._publish_twist(
                        CREEP_SPEED,
                        max(-CREEP_WZ_MAX, min(CREEP_WZ_MAX, CREEP_WZ_GAIN * err)))
                    travel += CREEP_SPEED * CREEP_PERIOD
                if aligning:
                    # Turning in place is **not** a stall: the radial distance does
                    # not shrink while the heading is corrected. Counting it would cap
                    # the tolerable entry heading error at CREEP_ALIGN_MAX +
                    # CREEP_WZ_MAX * CREEP_STALL_TIME = 1.10 rad = 63 deg, beyond which
                    # the creep always aborts. CREEP_TIMEOUT and the CREEP_BUDGET
                    # bounds hold the risk.
                    last, stall_t = d, None
                elif last - d < 1e-3:
                    stall_t = stall_t or time.monotonic()
                    if time.monotonic() - stall_t > CREEP_STALL_TIME:
                        self.get_logger().warn(
                            f'末段贴近 {CREEP_STALL_TIME:.1f}s 内估距没下降（仍有 {d:.2f} m），停止')
                        break
                else:
                    last, stall_t = d, None
                time.sleep(CREEP_PERIOD)
        finally:
            self._halt()
            time.sleep(CREEP_SETTLE)
        d_end = self._distance_to(x, y)
        return (d_end is not None and d_end <= stop_at), (d if d_end is None else d_end)

    def _forbidden_start(self):
        """Is the current pose inside a landmark's planner rejection ring?

        The rejection radius is ``_parking_floor(sigma)`` = landmark lethal disc +
        ``ROBOT_RADIUS``: the same ring that makes SmacPlanner2D report ``Starting
        point in lethal space``, because a robot centre inside it means its own cell is
        already inscribed(253). Returns the ``(color, x, y, sigma, d)`` with the
        largest escape distance, or None when outside every ring.

        The test radius is ``_parking_floor(sigma) + GUARD_MARGIN``: the planner's
        actual rejection band is ~0.15 m wider than ``_parking_floor(sigma)`` (0.05 m
        costmap discretisation plus ~0.1 m EKF lateral error), and a test without
        margin misses that ``[floor, floor + eps)`` blind band.
        """
        if self.pose is None or self.map is None:
            return None
        worst, worst_pen = None, 0.0
        for c, x, y, s, v in zip(self.map.colors, self.map.x, self.map.y,
                                 self.map.sigma, self.map.valid):
            if not v:
                continue
            # Trigger radius; see the GUARD_MARGIN note for why it is not
            # _parking_floor alone.
            reach = self._parking_floor(float(s)) + GUARD_MARGIN
            d = self._distance_to(float(x), float(y))
            if d is None or d >= reach:
                continue
            pen = reach - d
            if worst is None or pen > worst_pen:
                worst, worst_pen = (c, float(x), float(y), float(s), d), pen
        return worst

    def _escape_lethal_start(self, t0, timeout):
        """Back the robot out of the rejection ring left behind by the previous dock.

        Why this cannot be left to Nav2's backup recovery is explained next to the
        ``BACKOFF_*`` constants: Nav2's backup is blocked by its own
        ``isCollisionFree`` and does not move a single step, so the only way out is to
        publish /cmd_vel from this node. Only the EKF pose and landmark estimate are
        used; travel, time and "the distance stops growing" are three hard caps, and
        hitting any of them stops the motion with an honest report instead of
        pretending the ring was left.

        Returns ``(left the rejection ring, tag)``; ``(True, None)`` when not in one.
        """
        zone = self._forbidden_start()
        if zone is None:
            return True, None
        color, x, y, sigma, d0 = zone
        floor = self._parking_floor(sigma)
        # Stop at floor + GUARD_MARGIN, i.e. at the trigger radius of
        # _forbidden_start, so the robot is really out even though sigma -- and with
        # it the ring boundary -- moves whenever /landmark_map is republished.
        need = floor + GUARD_MARGIN
        step = min(need - d0, BACKOFF_DIST_MAX)
        t_end = min(time.monotonic() + BACKOFF_TIMEOUT, t0 + timeout - 2.0)
        self.get_logger().warn(
            f'起点落在 {color} 地标的规划层拒收圈内（{d0:.2f} m，'
            f'边界 {floor:.2f} m + 余量 {GUARD_MARGIN:.2f} m）：'
            f'Nav2 会报 "Starting point in lethal space" 且 backup 被碰撞检查挡住'
            f'（一步不动），先用 /cmd_vel 倒车 {step:.2f} m 出圈再规划')
        travel, last, stall_t = 0.0, d0, None
        try:
            while time.monotonic() < t_end:
                d = self._distance_to(x, y)
                yaw = self._yaw_now()
                if d is None or yaw is None or d >= need or travel >= step:
                    break
                # Escape direction = landmark -> robot; while docked the nose points at
                # the landmark, so the landmark is straight behind.
                heading = math.atan2(self.pose.position.y - y, self.pose.position.x - x)
                err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
                if abs(err) > math.pi * 0.5:
                    # Reversing: the travel direction is body -x (map yaw + pi), so the
                    # error is shifted by pi. The angular sign matches the forward case:
                    # +wz turns body +x, and turns the travel direction by +wz too.
                    vx = -BACKOFF_SPEED
                    e = math.atan2(math.sin(err - math.pi), math.cos(err - math.pi))
                else:
                    vx = BACKOFF_SPEED
                    e = err
                self._publish_twist(
                    vx, max(-BACKOFF_WZ_MAX, min(BACKOFF_WZ_MAX, BACKOFF_WZ_GAIN * e)))
                travel += BACKOFF_SPEED * BACKOFF_PERIOD
                if d - last < 1e-3:
                    stall_t = stall_t or time.monotonic()
                    if time.monotonic() - stall_t > BACKOFF_STALL_TIME:
                        self.get_logger().warn(
                            f'退避 {BACKOFF_STALL_TIME:.1f}s 内离 {color} 的距离没涨'
                            f'（仍有 {d:.2f} m），停止')
                        break
                else:
                    last, stall_t = d, None
                time.sleep(BACKOFF_PERIOD)
        finally:
            self._halt()
            time.sleep(BACKOFF_SETTLE)
        d_end = self._distance_to(x, y)
        out = d_end is not None and d_end >= need
        self.get_logger().info(
            f'退避结束：距 {color} {d_end:.2f} m（规划层边界 {floor:.2f} m，'
            f'倒车目标 {need:.2f} m）{"，已带余量出圈" if out else "，仍未到余量目标，交给 Nav2"}')
        return out, color

    def _stand_off_offset(self, sigma):
        """How far the goal should stand off from a landmark, measured along the
        landmark -> robot direction.

        Not a constant, but the larger of "reachable" and "clear of obstacles":

        1. ``MIN_STANDOFF`` -- the space the cube itself occupies, plus robot_radius.
        2. The landmark's **lethal disc** radius on /map plus ``ROBOT_CLEARANCE``.
           ``map_node`` paints every cell within
           ``min(inflation_base + sigma_k*sigma, max_inflation)`` of the landmark as
           lethal, and Nav2's inflation layer pushes a further robot_radius of
           inscribed cells outward. A goal inside those layers makes ``SmacPlanner2D``
           fail outright with "Goal in collision" / "no valid path"; worse, the
           previous goal leaves the robot standing inside the disc, so the next round
           fails with "Starting point in lethal space".

        The result is deliberately **not** clamped to ``approach``. The acceptance
        criterion is one-sided -- a goal closer than ``approach_distance + 0.15 m``
        passes, a farther one fails -- so the goal gets as close as it can while
        staying outside the disc. Forcing it out to exactly ``approach`` buys nothing
        and pushes the goal into lethal space.

        **The standoff trades reachability for safety**: the true distance is roughly
        standoff + map landmark error, while only ``approach + 0.15 = 0.75 m`` is
        allowed. Combining both bounds:

            disc + 0.26 <= 0.75  =>  0.20 + 3*sigma <= 0.49  =>  sigma <= 0.097

        i.e. only with ``sigma`` below ~0.10 m can the goal be plannable and
        reachable at once. With a larger ``sigma`` this function still retreats
        correctly on geometry, and the true distance then necessarily exceeds the
        tolerance: the right reading is "the landmark estimate is not accurate
        enough", not "the navigation was off". What has to go down is ``sigma``
        (EKF / observation accuracy); the standoff is only its mirror.

        With inflation_base = 0.30 and sigma_k = 0 in params.yaml the standoff is
        ``max(0.30 + 0.26, 0.51) = 0.56 m``, independent of sigma and always inside
        0.75 m. The trade-off is that the /map disc no longer expresses uncertainty;
        sigma is shown instead by the sigma spheres (radius 3*sigma) in RViz.
        Raising ``sigma_k`` back to a non-zero value restores the reasoning above.
        """
        need = self._landmark_radius(sigma) + ROBOT_CLEARANCE
        # Lower bound MIN_STANDOFF: the landmark is a 0.4 m cube (half edge 0.2) plus
        # robot_radius 0.22; a closer goal would land in the cube's occupied cells and
        # Nav2 would report "Goal in collision".
        return max(need, MIN_STANDOFF)

    def _stand_off_point(self, x, y, off):
        """Goal = the point on the landmark -> robot line at distance ``off`` from the
        landmark.

        Uses the landmark positions and poses from the map (EKF); ground truth is never
        touched. The goal must not be placed ``approach`` metres from the *robot*
        instead: for d = 2.84 m and approach = 0.6 m that would drop the goal 0.6 m in
        front of the nose, Nav2 would report SUCCEEDED after a nudge, and the robot
        would never actually approach the target.
        """
        if self.pose is None:
            return x, y
        dx, dy = x - self.pose.position.x, y - self.pose.position.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return x, y
        # The 1.0 cap keeps the goal from landing behind the robot: when d <= off the
        # caller has already taken the parking-ring branch and reported arrival, so no
        # goal is sent from here.
        k = min(1.0, max(0.0, off / d))
        return x - k * dx, y - k * dy

    @staticmethod
    def _yaw_to(px, py, tx, ty):
        """Heading from (px, py) toward (tx, ty); returns ``(yaw, Quaternion)``.

        Called with the goal and the landmark, so the robot faces the landmark on
        arrival.

        A pose goal is checked by ``SimpleGoalChecker`` (stateful: True, xy 0.25 m /
        yaw 0.25 rad), so a fixed orientation forces the robot to spin in place after
        reaching xy. While spinning, translation is ~0 -- and ``SimpleProgressChecker``
        (required_movement_radius 0.5 m / movement_time_allowance 10 s, see
        nav2_params.yaml) watches translation only -- so it reports
        ``Failed to make progress`` every 10 s and the behaviour tree falls into a Spin
        recovery loop. Aiming at the landmark makes xy and yaw converge together, and
        the heading stays meaningful: the landmark sits straight ahead.
        """
        yaw = math.atan2(ty - py, tx - px)
        q = Quaternion()
        q.z, q.w = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
        return yaw, q

    def _clear_costmaps(self):
        """Clear the costmaps before a retry, so the footprint left by the previous
        attempt does not make the next goal unreachable.
        """
        for cli in self._clear_clients:
            name = cli.srv_name
            if cli.wait_for_service(timeout_sec=2.0):
                cli.call_async(ClearEntireCostmap.Request())
                self.get_logger().info(f'已清代价地图：{name}')
            else:
                self.get_logger().warn(f'{name} 不可用，跳过')

    # ---------------- Relocation search (look elsewhere when the spin finds
    # nothing) ----------------
    def _reloc_candidates(self, color, visited):
        """Known landmarks worth relocating to, sorted by distance from the current
        pose.

        A candidate is a landmark that is ``valid`` in ``/landmark_map``, is **not**
        the target colour, has not been visited yet and is **actually worth the trip**
        (skipped when the robot is already closer to it than its arrival point).
        Returns ``[(d, color, x, y, sigma, off)]``.

        The arrival point sits ``off = max(_stand_off_offset(sigma),
        _parking_floor(sigma) + GUARD_MARGIN)`` from the landmark:

        * ``_stand_off_offset`` keeps the goal out of the cube's occupied cells, as
          for a normal dock;
        * ``_parking_floor + GUARD_MARGIN`` keeps the goal **outside the planner
          rejection ring**. A normal dock stops at 0.56 m, but that is a mission
          endpoint; a relocation only needs a vantage point with a clear view, and
          ending up inside the rejection ring would make the following Nav2 plan fail
          with "Starting point in lethal space". Hence the extra ``GUARD_MARGIN``.
        """
        out = []
        if self.map is None or self.pose is None:
            return out
        for c, x, y, s, v in zip(self.map.colors, self.map.x, self.map.y,
                                 self.map.sigma, self.map.valid):
            c, x, y, s = str(c), float(x), float(y), float(s)
            if not v or c == color or c in visited or c not in SEARCH_RELOC_ALLOW:
                continue
            d = self._distance_to(x, y)
            if d is None:
                continue
            off = max(self._stand_off_offset(s), self._parking_floor(s) + GUARD_MARGIN)
            if d <= off + SEARCH_RELOC_MIN_GAP:
                continue        # walking there is no better than spinning here
                # once more
            out.append((d, c, x, y, s, off))
        out.sort(key=lambda r: r[0])
        return out

    def _start_reloc_search(self, color, left, total, visited, search_deadline, now, t0):
        """Pick a known landmark A, send a Nav2 goal, return ``(Nav2 handle, colour of
        A)``.

        Returns ``(None, None)`` when no A is worth visiting (quota used up / budget
        too small / Nav2 unavailable / every candidate rejected), and the caller falls
        back to waiting in place. **Only the search budget is spent**: the leg ends at
        ``search_deadline``, so the share reserved for navigation is untouched.
        """
        if left <= 0:
            return None, None
        budget = search_deadline - now
        if budget < SEARCH_RELOC_MIN_BUDGET:
            self.get_logger().info(
                f'剩余搜索预算 {budget:.1f}s < {SEARCH_RELOC_MIN_BUDGET:.0f}s，'
                f'不再搬迁（留给目标出现后的导航）')
            return None, None
        for d, c, x, y, _s, off in self._reloc_candidates(color, visited):
            if not self.nav.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn('navigate_to_pose 不可用，无法搬迁搜索')
                return None, None
            gx, gy = self._stand_off_point(x, y, off)
            goal_yaw, goal_quat = self._yaw_to(gx, gy, x, y)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
            goal.pose.pose.orientation = goal_quat
            self._nav_gen += 1
            gen = self._nav_gen
            self._nav_done.clear()
            self._nav_status = None
            self._nav_distance = -1.0
            self.get_logger().warn(
                f'{color}：原地扫一圈没看到 ⇒ 主动搬迁搜索，去已知的 {c} 附近'
                f'（({x:.2f},{y:.2f}) 退 {off:.2f} m ⇒ 终点 ({gx:.2f},{gy:.2f})，'
                f'当前距它 {d:.2f} m，终朝向 {math.degrees(goal_yaw):.1f}°）；'
                f'沿途继续找 {color}，到了再扫一圈'
                f'（第 {total - left + 1}/{total} 段，剩余搜索预算 {budget:.0f}s）')
            send_fut = self.nav.send_goal_async(
                goal, feedback_callback=lambda fb, g=gen: self._on_nav_feedback(fb, g))
            t_end = min(time.monotonic() + 5.0, search_deadline)
            while not send_fut.done() and time.monotonic() < t_end:
                time.sleep(SEARCH_SPIN_PERIOD)
            handle = send_fut.result() if send_fut.done() else None
            if handle is None or not handle.accepted:
                self.get_logger().warn(f'去 {c} 附近的搬迁目标被拒绝，换下一个候选')
                continue
            handle.get_result_async().add_done_callback(
                lambda fut, g=gen: self._on_nav_result(fut, g))
            visited.append(c)
            return handle, c
        return None, None

    # ---------------- Main execution loop ----------------
    def execute(self, goal_handle):
        """Full life cycle of one GoToBlock goal; each goal runs its own FSM."""
        req = goal_handle.request
        color, timeout = req.target_color, float(req.timeout_sec) or 60.0
        approach = float(req.approach_distance) or self.approach
        t0 = time.monotonic()
        fb = GoToBlock.Feedback()
        res = GoToBlock.Result()
        fsm = StateMachine('IDLE')
        fsm.on('start')                        # IDLE -> SEARCHING
        # One independent collision tally per goal: reset at the start, closed on any
        # exit path (success / failure / cancel).
        self._tally_begin(color)

        def abort_with(msg, dist=-1.0):
            res.success, res.final_distance = False, dist
            res.message = f'{msg}；到达过程{self._tally_finish(False)}'
            self.get_logger().error(res.message)
            goal_handle.abort()
            return res

        def cancel_with(msg, dist=-1.0):
            # canceled() and abort() are mutually exclusive terminal transitions; only
            # one of them may be called (rclpy rejects an illegal state transition).
            res.success, res.final_distance = False, dist
            res.message = f'{msg}；到达过程{self._tally_finish(False)}'
            self.get_logger().warn(res.message)
            goal_handle.canceled()
            return res

        # The rejection ring has to be left **before** the node starts observing.
        #
        # The previous docking point lies inside that landmark's own planner rejection
        # ring, and the next target has often not been observed yet. Escaping only
        # right before a Nav2 goal deadlocks: the target has to be visible to reach
        # the escape, the escape has to happen before spinning in place (inside the
        # ring behavior_server aborts every spin with "Collision Ahead - Exiting
        # Spin"), and the spin is needed to see the target.
        for _ in range(BACKOFF_ROUNDS):
            if self._forbidden_start() is None:
                break
            self._escape_lethal_start(t0, timeout)

        # ---- SEARCHING: wait for the target colour to enter the map, and spin in
        # place to find it when it does not ----
        fb.state, fb.target_known, fb.distance_to_target = SEARCHING, False, -1.0
        goal_handle.publish_feedback(fb)
        reserve = max(min(SEARCH_TIME_RESERVE, SEARCH_RESERVE_FRAC * timeout),
                      SEARCH_RESERVE_GROW_FRAC * timeout)
        search_deadline = t0 + timeout - reserve   # absolute search deadline (with relocation)
        spin_start = None            # instant the spin started (None = not yet)
        spin_yaw = None              # last sampled yaw, used to accumulate the swept angle
        spin_swept = 0.0             # accumulated swept angle (rad), |d_yaw| only
        spin_prog_t = 0.0            # instant of the last increase in swept angle (stall test)
        spin_active = False          # whether an angular command is being sent (decides _halt)
        spin_done = False            # whether this spin round finished (relocation comes after)
        spin_round = 0               # spin round counter (1 at the start pose, then at each A)
        # ---- Relocation-search state (design in the SEARCH_RELOC_* constants) ----
        reloc_max = (int(self.get_parameter('search_reloc_max').value)
                     if bool(self.get_parameter('search_reloc').value) else 0)
        reloc_left = reloc_max       # relocation legs still available
        reloc_visited = []           # landmarks already visited (avoids A / B ping-pong)
        reloc_active = False         # a relocation is running (Nav2 drives, this node watches)
        reloc_handle = None          # relocation Nav2 goal; closed before leaving SEARCHING
        reloc_color = None           # colour of the landmark being approached (log only)
        reloc_leg_end = 0.0          # deadline of the current relocation leg
        reloc_log_t = 0.0            # relocation progress log pacing
        fb_t = time.monotonic()
        try:
            while self.target_xy(color) is None or self.pose is None:
                now = time.monotonic()
                if goal_handle.is_cancel_requested:
                    return cancel_with(f'已取消（等待 {color} 期间）')
                if now - t0 > timeout:
                    # The relocation goal is closed by the finally block below (return
                    # paths run it too)
                    return abort_with(
                        f'SEARCHING 超时 {timeout:.0f}s：始终未观测到 {color}'
                        + (f'（原地扫了 {spin_round} 轮，并主动搬到 '
                           f'{"、".join(reloc_visited)} 附近找过，仍未观测到）'
                           if reloc_visited else ''))

                if reloc_active:
                    # ---- Relocation leg: Nav2 drives, this node only observes ----
                    # Seeing the target on the way counts as success (the while
                    # condition then fails); the finally block cancels this Nav2 goal.
                    if self._nav_done.is_set() or now >= reloc_leg_end:
                        arrived = self._nav_done.is_set()
                        if not arrived and reloc_handle is not None:
                            reloc_handle.cancel_goal_async()
                        d_a = self._nav_distance
                        self.get_logger().info(
                            f'搬迁到 {reloc_color} 附近这一段结束（'
                            f'{"Nav2 已到点" if arrived else "本段预算用尽"}，'
                            f'距该地标剩 {d_a:.2f} m，已用 {now - t0:.0f}s）：'
                            f'在新位置再扫一圈找 {color}')
                        reloc_active, reloc_handle, reloc_color = False, None, None
                        time.sleep(BACKOFF_SETTLE)   # let Nav2 settle before sending speeds
                        spin_start, spin_yaw = None, None
                        spin_swept, spin_prog_t = 0.0, 0.0
                        spin_done = False
                    else:
                        if now - reloc_log_t >= SEARCH_RELOC_LOG_PERIOD:
                            self.get_logger().info(
                                f'搬迁中 → {reloc_color} 附近（距该地标剩 '
                                f'{self._nav_distance:.2f} m，已用 {now - t0:.0f}s）：'
                                f'沿途继续找 {color}')
                            reloc_log_t = now
                        if now - fb_t >= SEARCH_FB_PERIOD:
                            fb.elapsed_time = now - t0
                            goal_handle.publish_feedback(fb)
                            fb_t = now
                        time.sleep(SEARCH_SPIN_PERIOD)
                        continue

                yaw = self._yaw_now()
                if yaw is None:
                    # No /robot_pose yet: not even the heading is known, so the swept
                    # angle cannot accumulate; stop and wait for the pose.
                    if spin_active:
                        self._halt()
                        spin_active = False
                elif spin_start is None:
                    if now - t0 >= self.lost_sec:
                        spin_start, spin_yaw, spin_prog_t = now, yaw, now
                        spin_round += 1
                        self.get_logger().warn(
                            f'{self.lost_sec:.0f}s 内没观测到 {color}：朝向不对，'
                            f'开始第 {spin_round} 轮原地旋转搜索（{SEARCH_SPIN_WZ:.2f} rad/s，上限 '
                            f'{SEARCH_SWEEP_MAX:.2f} rad / 卡死 {SEARCH_SPIN_STALL:.0f}s / '
                            f'为导航保留 {reserve:.0f}s'
                            f'{"，有搬迁去处时单轮限 12s" if reloc_left > 0 else ""}）')
                else:
                    # |d_yaw| normalised via atan2: a wrap at +/- pi never reads as "2*pi short"
                    d_yaw = abs(math.atan2(math.sin(yaw - spin_yaw),
                                           math.cos(yaw - spin_yaw)))
                    spin_yaw = yaw
                    if d_yaw >= SEARCH_SPIN_EPS:
                        spin_swept += d_yaw
                        spin_prog_t = now
                    exhausted = (spin_swept >= SEARCH_SWEEP_MAX
                                 or now - spin_prog_t >= SEARCH_SPIN_STALL
                                 or now >= search_deadline
                                 # keep one round short while relocation targets remain
                                 or (reloc_left > 0
                                     and now - spin_start >= SEARCH_SPIN_SEC_MAX
                                     and bool(self._reloc_candidates(
                                         color, reloc_visited))))
                    if not exhausted:
                        self._publish_twist(0.0, SEARCH_SPIN_WZ)
                        spin_active = True
                    elif not spin_done:
                        # Nothing seen after the sweep: try elsewhere rather than
                        # waiting, which adds no information and only burns the
                        # navigation budget. Waiting stays the fallback once
                        # relocation is exhausted.
                        self._halt()
                        spin_active = False
                        spin_done = True
                        self.get_logger().warn(
                            f'原地旋转搜索扫过 {spin_swept:.2f} rad'
                            f'（{math.degrees(spin_swept):.0f}°）仍未观测到 {color}，'
                            f'停止旋转（已用 {now - t0:.0f}s，剩余 '
                            f'{max(0.0, timeout - (now - t0)):.0f}s）')
                        reloc_handle, reloc_color = self._start_reloc_search(
                            color, reloc_left, reloc_max, reloc_visited,
                            search_deadline, now, t0)
                        if reloc_handle is not None:
                            reloc_left -= 1
                            reloc_active = True
                            reloc_leg_end = min(search_deadline,
                                                now + SEARCH_RELOC_LEG_MAX)
                            reloc_log_t = now
                        else:
                            self.get_logger().warn(
                                f'{color}：主动搜索无法继续（搬迁额度用尽，或地图里没有'
                                f'值得去的其他已知色块），退回原地等待直到超时 '
                                f'{timeout:.0f}s（动态色块仍可能自己走进视野）')

                if now - fb_t >= SEARCH_FB_PERIOD:
                    fb.elapsed_time = now - t0
                    goal_handle.publish_feedback(fb)
                    fb_t = now
                time.sleep(SEARCH_SPIN_PERIOD)
        finally:
            # Target found, cancelled, timed out, raised -- every path out of SEARCHING
            # must stop the motion; a leftover angular command would carry into
            # NAVIGATING and fight Nav2 over /cmd_vel (see the CREEP_* comment).
            if spin_active:
                self._halt()
            # Close the relocation goal for the same reason: the target appeared, the
            # task was cancelled or it timed out. navigate_to_pose honours one goal at
            # a time, so leaving it alive would displace the NAVIGATING goal below.
            if reloc_handle is not None and not self._nav_done.is_set():
                reloc_handle.cancel_goal_async()
                self.get_logger().info(
                    f'{color} 的搜索结束：取消搬迁到 {reloc_color} 附近的导航目标')
                t_end = time.monotonic() + BACKOFF_SETTLE * 4.0
                while not self._nav_done.is_set() and time.monotonic() < t_end:
                    time.sleep(SEARCH_SPIN_PERIOD)
            reloc_active, reloc_handle, reloc_color = False, None, None

        x, y, sigma = self.target_xy(color)
        fb.target_known = True
        if reloc_visited:
            self.get_logger().warn(
                f'主动搜索奏效：原地扫了 {spin_round} 轮没看到 {color}，改去 '
                f'{"、".join(reloc_visited)} 附近的路上观测到了'
                f'（搜索阶段共 {time.monotonic() - t0:.1f}s）')
        self.get_logger().info(
            f'目标 {color} @ ({x:.2f}, {y:.2f})，σ={sigma:.3f} m '
            f'→ 膨胀圈 {self._landmark_radius(sigma):.2f} m，'
            f'终点退距 {self._stand_off_offset(sigma):.2f} m（approach={approach:.2f} m）')
        fsm.on('known')                        # SEARCHING -> LOCALIZING
        fsm.on('ok')                           # LOCALIZING -> NAVIGATING

        # ---- NAVIGATING: send the Nav2 goal, retry with a costmap clear on failure ----
        # Attempt 0 hugs the outside of the lethal disc; later attempts step outward
        # along the landmark -> robot direction. The retry ceiling sits at the top of
        # the ladder (`off0 + STEP * max_retries`) so it can actually climb out of the
        # inflation layer.
        off0 = self._stand_off_offset(sigma)
        off_cap = off0 + RETRY_STANDOFF_STEP * self.max_retries
        if off0 > approach + 1e-9:
            self.get_logger().warn(
                f'地标膨胀圈 {self._landmark_radius(sigma):.2f} m + 余量 '
                f'{ROBOT_CLEARANCE:.2f} m > approach {approach:.2f} m：'
                f'终点将退到 {off0:.2f} m（超出 approach+0.15 m 容差）；'
                f'根因是 σ={sigma:.3f} m 太大')
        used_off = off0
        # ---- Outer loop: refine the approach ----
        # sigma is estimated at the current range and gets more pessimistic with
        # distance, so a goal derived from a far away sigma can land outside the
        # tolerance. Driving onto the circle sigma implies shrinks sigma, which
        # shrinks the standoff in turn: iterate "far first, close later".
        stages = []
        for stage in range(self.max_refine + 1):
            for attempt in range(self.max_retries + 1):
                if goal_handle.is_cancel_requested:
                    return cancel_with('已取消')
                if not self.nav.wait_for_server(timeout_sec=10.0):
                    return abort_with('navigate_to_pose Action 不可用（Nav2 没起来？）')

                used_off = min(off0 + RETRY_STANDOFF_STEP * attempt, off_cap)
                gx, gy = self._stand_off_point(x, y, used_off)
                d_now = self._distance_to(x, y)

                self._nav_gen += 1
                gen = self._nav_gen
                self._nav_done.clear()
                self._nav_status = None
                self._nav_distance = -1.0

                if d_now is not None and d_now <= used_off:
                    # Already inside the parking ring: a goal at the robot's feet gives a
                    # degenerate path, so declare arrival directly
                    self._nav_distance = d_now
                    self.get_logger().info(f'距目标 {d_now:.2f} m <= {used_off:.2f} m，无需导航')
                    break

                # When the previous dock pushed the start into the rejection ring,
                # back out first, otherwise this plan fails with "Starting point in
                # lethal space" and Nav2's backup recovery never moves. See the
                # BACKOFF_* notes.
                for _ in range(BACKOFF_ROUNDS):
                    if self._forbidden_start() is None:
                        break
                    self._escape_lethal_start(t0, timeout)
                    d_now = self._distance_to(x, y)

                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
                # Goal orientation: **facing the landmark** (not an identity quaternion).
                goal_yaw, goal_quat = self._yaw_to(gx, gy, x, y)
                goal.pose.pose.orientation = goal_quat
                self.get_logger().info(
                    f'Nav2 终点 ({gx:.2f},{gy:.2f}) = 地标 ({x:.2f},{y:.2f}) 退 {used_off:.2f} m，'
                    f'终朝向 {math.degrees(goal_yaw):.1f}°（面向地标），'
                    f'当前距地标 {d_now if d_now is not None else float("nan"):.2f} m '
                    f'（第 {stage + 1} 段·第 {attempt + 1}/{self.max_retries + 1} 次）')

                send_fut = self.nav.send_goal_async(
                    goal, feedback_callback=lambda fb, g=gen: self._on_nav_feedback(fb, g))
                while not send_fut.done():
                    if time.monotonic() - t0 > timeout:
                        return abort_with('发送 Nav2 目标超时')
                    time.sleep(0.05)
                handle = send_fut.result()
                if handle is None or not handle.accepted:
                    self.get_logger().warn(f'Nav2 拒绝目标（第 {attempt + 1} 次）')
                    continue

                res_fut = handle.get_result_async()
                res_fut.add_done_callback(lambda fut, g=gen: self._on_nav_result(fut, g))

                # Docking target; not _parking_floor, which is the planner inflation
                # boundary and lies beyond the docking window (see _dock_target).
                stop_at = self._dock_target(used_off)
                closed_n = 0
                at_standoff_n = 0
                arrived_here = False
                at_standoff = False
                fb.state = NAVIGATING
                while not self._nav_done.is_set():
                    if goal_handle.is_cancel_requested:
                        handle.cancel_goal_async()
                        return cancel_with('已取消（导航中）', self._nav_distance)
                    if time.monotonic() - t0 > timeout:
                        handle.cancel_goal_async()
                        return abort_with(f'NAVIGATING 超时 {timeout:.0f}s', self._nav_distance)
                    # Close the loop in this node instead of waiting for Nav2's
                    # SUCCEEDED (reason next to ARRIVAL_CONFIRM).
                    d_here = self._distance_to(x, y)
                    if d_here is not None and d_here <= stop_at:
                        closed_n += 1
                        if closed_n >= ARRIVAL_CONFIRM:
                            arrived_here, at_standoff = True, False
                            break
                    else:
                        closed_n = 0
                        # On Nav2's standoff circle without ever getting SUCCEEDED
                        # (the final orientation never settles): take over after
                        # STANDOFF_SELF_N samples.
                        if d_here is not None and d_here <= used_off + STANDOFF_SELF_SLACK:
                            at_standoff_n += 1
                            if at_standoff_n >= STANDOFF_SELF_N:
                                arrived_here, at_standoff = True, True
                                break
                        else:
                            at_standoff_n = 0
                    fb.distance_to_target = self._nav_distance
                    fb.elapsed_time = time.monotonic() - t0
                    goal_handle.publish_feedback(fb)
                    time.sleep(0.1)

                if arrived_here:
                    if at_standoff:
                        self.get_logger().warn(
                            f'已站在 Nav2 退距圈上（估距 {d_here:.2f} m ≤ '
                            f'{used_off + STANDOFF_SELF_SLACK:.2f} m）但 '
                            f'{STANDOFF_SELF_N / 10.0:.0f}s 内没等到 SUCCEEDED：'
                            f'放弃终朝向要求，取消 Nav2 目标，交末段贴近收尾'
                            f'（域 216 的"终朝向收不了尾"兜底）')
                    else:
                        self.get_logger().info(
                            f'自主判定抵达：估距 {d_here:.2f} m <= 停靠目标 {stop_at:.2f} m'
                            f'（approach {self.approach:.2f} / 停靠窗口下界 '
                            f'{LANDMARK_HALF_DIAG + STOP_POLYGON_REACH:.2f}），'
                            f'取消 Nav2 目标（不等它的 SUCCEEDED）')
                elif self._nav_status != 4:     # not ACTION_STATUS_SUCCEEDED
                    self.get_logger().warn(f'Nav2 失败 status={self._nav_status}，'
                                           f'清代价地图后重试 {attempt + 1}/{self.max_retries}')
                    self._clear_costmaps()
                    fsm.on('blocked')          # NAVIGATING -> NAVIGATING (self-loop)
                    continue
                # Fake-success gate: Nav2's SUCCEEDED only means "reached the point
                # Nav2 was given". A map->odom TF lagging behind makes
                # controller_server report SUCCEEDED without a live pose, failing the
                # goal while the robot has not moved at all, which has to be retried
                # as a navigation failure.
                # Mechanism: isGoalReached() transforms the goal to odom with
                # nav_2d_utils::transformPose, whose timeout branch **returns false
                # without writing its output** while the caller ignores the return
                # value, so the goal degenerates to (0, 0, 0) and an unmoved robot
                # counts as arrived.
                if not arrived_here:
                    d_gate = self._distance_to(x, y)
                    if d_gate is not None and d_gate > used_off + ARRIVAL_SLACK:
                        self.get_logger().warn(
                            f'Nav2 报成功但仍在距目标 {d_gate:.2f} m 处 '
                            f'(> {used_off + ARRIVAL_SLACK:.2f} m)，判为假成功'
                            f'（map→odom 变换失败时终点被丢弃成 (0,0,0)，'
                            f'清代价地图救不回这种成因），清代价地图重试 '
                            f'{attempt + 1}/{self.max_retries + 1}')
                        self._clear_costmaps()
                        fsm.on('blocked')      # NAVIGATING -> NAVIGATING (self-loop)
                        continue
                # Arrival (self-judged or reported by Nav2) always closes the Nav2 goal:
                # the final creep publishes /cmd_vel itself, and controller_server must not
                # still hold the same goal competing for the channel.
                handle.cancel_goal_async()
                break
            else:
                return abort_with(f'Nav2 重试 {self.max_retries} 次仍失败', self._nav_distance)

            stages.append(used_off)
            # Arrived at this stop: sigma should have shrunk because the robot got
            # closer, so the standoff shrinks too. Refine once more until sigma
            # converges (gain below REFINE_MIN_GAIN) or the stage / budget limit is hit.
            if stage >= self.max_refine or goal_handle.is_cancel_requested:
                break
            if time.monotonic() - t0 > timeout - REFINE_TIME_RESERVE:
                self.get_logger().info('剩余预算不足以再贴近一趟，就停在这里')
                break
            fresh = self.target_xy(color)
            if fresh is None:
                break
            nx, ny, nsigma = fresh
            need = max(approach, self._stand_off_offset(nsigma))
            d_after = self._distance_to(nx, ny)
            if need > used_off - REFINE_MIN_GAIN:
                break         # sigma did not shrink: another leg would reuse the same circle
            if d_after is not None and d_after <= need:
                break         # already inside the new parking ring, no need to move
            self.get_logger().info(
                f'贴近复算：σ {sigma:.3f} → {nsigma:.3f} m，退距 {used_off:.2f} → {need:.2f} m'
                f'（当前距地标 {d_after:.2f} m）')
            x, y, sigma = nx, ny, nsigma
            off0, off_cap = need, need + RETRY_STANDOFF_STEP * self.max_retries
            used_off = off0

        # ---- Final creep: cover the tolerance sphere Nav2 leaves behind ----
        # (see the comment next to CREEP_SPEED; no planner parameter is touched.)
        d_before_creep = self._distance_to(x, y)
        stop_at = self._dock_target(used_off)
        # The docking point necessarily falls inside the planner inflation boundary
        # (_parking_floor = 0.42 + 3*sigma). This round is harmless, because the creep
        # ends in DONE with no further planning, but a later goal on the same landmark
        # fails with "Start in collision" and has to be issued from outside the disc.
        floor = self._parking_floor(sigma)
        if stop_at < floor:
            self.get_logger().info(
                f'停靠目标 {stop_at:.2f} m 在规划层膨胀边界 {floor:.2f} m 之内'
                f'（比 approach+0.15 = {self.approach + 0.15:.2f} m 更近）：'
                f'本轮收尾后不再规划，安全；同一地标的后续目标需从圈外发起')
        crept, d_creep = self._final_creep(x, y, stop_at, t0, timeout)
        if d_creep is not None and d_before_creep is not None:
            if crept:
                self.get_logger().info(
                    f'末段贴近完成：估距 {d_before_creep:.2f} → {d_creep:.2f} m'
                    f'（规划退距 {used_off:.2f} m，停靠目标 {stop_at:.2f} m）')
            else:
                self.get_logger().warn(
                    f'末段贴近没走到停靠目标 {stop_at:.2f} m（估距 {d_creep:.2f} m），'
                    f'终点会落在 approach+0.25 m 附近；请查 σ 与障碍')

        # ---- APPROACHING -> DONE ----
        fsm.on('near')                         # NAVIGATING -> APPROACHING
        d_final = self._distance_to(x, y)      # use the live pose, not a -1 feedback value
        if d_final is None:
            d_final = max(0.0, self._nav_distance)
        fb.state = APPROACHING
        fb.distance_to_target = d_final
        fb.elapsed_time = time.monotonic() - t0
        goal_handle.publish_feedback(fb)
        # Nav2's SUCCEEDED only means "reached the point it was given", not "parked
        # inside the parking ring". Beyond ARRIVAL_SLACK the parking semantics are
        # broken, and reporting a failure beats a fake success. TF fake successes were
        # already caught and retried in the NAVIGATING loop.
        if d_final > used_off + ARRIVAL_SLACK:
            return abort_with(f'Nav2 报成功但仍在距目标 {d_final:.2f} m 处 '
                              f'(> {used_off + ARRIVAL_SLACK:.2f} m)', d_final)
        res.success = True
        # Close the tally before composing the message, so this trip's collisions go back
        # to the client as well.
        tally_text = self._tally_finish(True)
        res.message = (f'抵达 {color}（估计位置 ({x:.2f},{y:.2f})，'
                       f'approach={used_off:.2f} m，耗时 {fb.elapsed_time:.1f}s'
                       + ('，分 {} 段收敛 {} → {:.2f} m'.format(
                           len(stages), '/'.join(f'{s:.2f}' for s in stages), used_off)
                          if len(stages) > 1 else '')
                       + (f'，末段自主贴近 → {d_creep:.2f} m' if crept else '')
                       + f'）；到达过程{tally_text}')
        res.final_distance = d_final
        self.get_logger().info(res.message)
        fsm.on('arrived')                      # APPROACHING -> DONE
        goal_handle.succeed()
        return res


def main(args=None):
    """Node entry point: ``action_server = goal_selector.action_server:main``."""
    rclpy.init(args=args)
    node = GoalSelector()
    # MultiThreadedExecutor is required: execute() is a long task, and a single-threaded
    # executor would block Nav2 feedback and cancel requests.
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
