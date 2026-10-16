#!/usr/bin/env python3
"""Black / white dynamic obstacles: the two 0.4 m cubes of the dynamic world.

``worlds/landmark_world_dynamic.world`` adds ``obstacle_white`` and ``obstacle_black`` next
to the four coloured landmarks.  This node drives them along the closed random-walk loops
that ``scripts/world_setup.py`` generated, using ``/gazebo/set_entity_state``, so the cubes
roam the whole 8x8 m arena.

Nothing else in the stack had to change.  ``detector_params.yaml`` carries a ``white`` /
``black`` threshold set and ``detector_node.py`` marks everything outside
``(red, green, blue, yellow)`` as ``role="obstacle"``, which ``ekf_core.py`` never feeds
into the update step: ``/landmark_map`` stays the four coloured corners and ``/goto_block``
can never be aimed at a cube.  ``nav2_params.yaml`` points both costmaps' ``obstacle_layer``
at ``/scan``, and the waffle's 360 deg lidar sits inside the cube's height band, so a cube
reaches the planner as an ordinary laser obstacle.  That is what makes it a *dynamic
obstacle* rather than a *moving landmark*: it triggers a re-plan and never corrupts the EKF
map.

Geometry and speed arrive as ``<colour>_path`` / ``<colour>_speed`` / ``<colour>_phase``
from the generated ``dynamic_paths.yaml``, which ``m8_demo.launch.py`` appends as a second
``--params-file``.  Without that file the paths fall back to the 24-gons in
``DEFAULT_PATHS``, so the node can still be run by hand.  Mutual clearance comes from
``world_rules`` at generation time and from ``world_rules.CellClaims`` at run time; see
``_robot_blocks``, ``_claim_statics`` and ``advance`` for the run-time rules.

The node starts **disabled** (``enabled``, default false), so the cubes stay where the world
file put them and a run stays reproducible until the operator says otherwise:

    ros2 service call /dynamic_obstacles/start slam_interfaces/srv/StartObstacles "{speed: 1.2}"
    ros2 service call /dynamic_obstacles/set_enabled std_srvs/srv/SetBool "{data: false}"
    ros2 service call /dynamic_obstacles/reset std_srvs/srv/Empty "{}"

An empty ``color`` selects every cube.  Speed is chosen when the cubes set off, never while
they move: a mid-motion change would jump a cube along its path while its twist still
described the old point.  ``~/start`` therefore refuses to run while ``enabled`` is true, and
``ros2 param set`` on ``white_speed`` / ``black_speed`` is rejected until the cubes are idle;
``speed <= 0`` means "keep the speed ``params.yaml`` holds".  ``reset`` rewinds the phase
clock, teleports both cubes home and stops them, which makes it the third step of the reset
contract, after ``/reset_world`` and ``/reset_slam``: the order matters, because
``/reset_world`` restores every model (these two included) and only afterwards does this
node decide where "home" is.  All three are ``std_srvs/srv/Empty``, the type
``gazebo_ros_init`` publishes them with.

``world_rules.replay_stats`` replays the same per-tick rules offline, so the trajectories
``validate_scene`` accepted are the ones this node replays; its robot-clearance number is the
parked-spawn-point one, because the offline replay knows where the robot starts, not where it
drives.  The claims themselves are internal: nothing about them is published, and the robot
side learns where a cube is only from ``/observations``.

Timing
------
Both timers are wall-clock timers (``ClockType.STEADY_TIME``), not ``get_clock()``: under
``use_sim_time`` the node clock advances only when ``/clock`` arrives, so a ros-clock timer
is quantised to the ``/clock`` edges and every tick-counted window (``yield_ticks``,
``mutual_after``, ``mutual_lock``) drifts away from the offline model.  ``on_control``
accumulates the sim-clock delta and spends it in fixed ``1/rate_hz`` slices -- arc
``speed / sim_rate_hz`` and turn ``max_yaw_rate / sim_rate_hz`` per slice -- with the slices
derived from simulated time, so the cubes still move at ``speed x RTF`` on screen.

Publishes
---------
/obstacle_markers   visualization_msgs/MarkerArray   pose of every **observed** obstacle

Subscribes
----------
/observations          slam_interfaces/LandmarkArray              where the cubes are seen
/robot_pose            geometry_msgs/PoseWithCovarianceStamped   EKF pose, to project them
/gazebo/model_states   gazebo_msgs/ModelStates                  only w/ require_observation false

Serve
-----
/dynamic_obstacles/start         slam_interfaces/StartObstacles  start, optionally at a new speed
/dynamic_obstacles/set_enabled   std_srvs/SetBool               start / stop at the current speed
/dynamic_obstacles/reset         std_srvs/Empty                 home + stop

Markers (``require_observation``)
---------------------------------
By default a cube is drawn **only** while the camera reports it, and its position is
*derived* rather than read: ``/observations`` supplies a camera-frame (range, bearing),
``/robot_pose`` supplies the EKF pose, and the two are composed the way ``ekf_core`` composes
a landmark observation, including the same near-face and lever-arm range bias;
``observed_to_map`` and ``_marker_pose`` carry the details.  Drawing from
``/gazebo/model_states`` instead would show a placed box for a cube the robot has never seen,
which is the wrong thing to demo, so that mode is off by default; it stays useful for checking
that the world file, the physics and the detector agree, and it works in the Gazebo *world*
frame, which is the frame this stack calls ``map``.
"""

import math
import os
import sys

# scripts/ is not a Python package (install(PROGRAMS) only copies the files into
# lib/sim_bringup/), so the sibling world_rules has to be put on sys.path by hand.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import world_rules as wr  # noqa: E402 - must follow the sys.path insert above

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from slam_interfaces.msg import LandmarkArray
from slam_interfaces.srv import StartObstacles
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, SetBool
from visualization_msgs.msg import Marker, MarkerArray

# Model name prefix: the <model name> in the world file must be obstacle_<colour>.
MODEL_PREFIX = 'obstacle_'

# The 0.4 m cube rests with its centre 0.2 m above the ground, as in the world
# <pose>.
CUBE_SIZE = 0.4
DEFAULT_Z = 0.2

# Robot footprint radius: the same value as nav2's `robot_radius` and world_rules'
# `robot_half`.
ROBOT_RADIUS = 0.22

# Radius at which "cube and robot really touch" (Chebyshev, the measure the claim
# table uses): robot 0.22 m plus the cube's **circumscribed** half-diagonal
# 0.5*0.4*sqrt(2) = 0.28284 m, so the cube surface lies inside the footprint below
# 0.5028 m centre distance.
ROBOT_CONTACT = ROBOT_RADIUS + 0.5 * CUBE_SIZE * math.sqrt(2.0)

# Keep-out radius of "no straight hop may cross the chassis" (see `_crosses_robot`):
# 0.05 m of margin on top of ROBOT_CONTACT and looser than `robot_cross_keep` (0.70),
# whose one-tick landing floor this is -- the closest such segment passes 0.63 m
# from the robot, so 0.55 m never rejects the creep step.
ROBOT_NO_CROSS = 0.55

# Upper bound on the simulated-time budget of one on_control call (sim seconds) and
# on the number of ticks it may run, both to absorb stalls: a normal tick
# accumulates a fraction of 1/rate_hz, so a wake-up advances 0 or 1 tick. Without
# the cap, the time accumulated while the simulation is paused or slowed down would
# let a cube jump ahead and skip the per-tick claims.
SIM_BUDGET_MAX = 0.20
SIM_STEPS_MAX = 8

# The two constants of the bypass detour (see DynamicObstacles._bypass_path). They
# are not parameters because they only trade conservatism for freedom:
#   * how far the detour may leave the bounding box of that colour's walk. The walk
#     itself stops at arena_half - wall_margin - half_diag, keeping half_diag
#     (0.283 m) of margin, so 0.10 m can never push a cube into a wall while still
#     allowing a detour along the wall side;
#   * how many ticks an occupied landing spot is retried before the bypass is given
#     up and the ordinary yielding logic takes over.
BYPASS_BOX_MARGIN = 0.10
BYPASS_STALL_TICKS = 40

# Observation -> map conversion constants; they must match the ekf_core.EKFCore
# constructor defaults (camera_dx / camera_dy / landmark_half_size). They are
# copied rather than imported from ekf_slam because sim_bringup declares a
# dependency on slam_interfaces only, and a cross-package import would add one.
CAMERA_DX = 0.069
CAMERA_DY = -0.047
LANDMARK_HALF_SIZE = 0.2


def wrap(a):
    """Normalise an angle to [-pi, pi), as ekf_core.wrap does."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q):
    """Quaternion -> yaw (about z only), the same formula as ekf_core.quat_to_yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def observed_to_map(px, py, yaw, r, phi):
    """Camera-frame (range, bearing) -> map (x, y), as ekf_core.init_landmark does.

    The detector reports the range from the camera to the **near face** of the cube,
    while the map coordinate is the robot origin to the cube centre: the two differ
    by the near-face offset (the ray hits the face first) and by the lever arm (the
    measurement starts at the camera, not at base_link).
    The formulas are ekf_core.geometry_bias's; as in the EKF, the derivative of that
    bias with respect to theta (~0.083 m/rad) is not taken into account.
    """
    ang = wrap(yaw + phi)
    ux, uy = math.cos(ang), math.sin(ang)
    face = LANDMARK_HALF_SIZE / max(abs(ux), abs(uy), 1e-6)
    c, s = math.cos(yaw), math.sin(yaw)
    lever = (c * CAMERA_DX - s * CAMERA_DY) * ux + (s * CAMERA_DX + c * CAMERA_DY) * uy
    r_eff = float(r) + face + lever
    return px + r_eff * math.cos(ang), py + r_eff * math.sin(ang)


# Fallback trajectories, [centre x, centre y, radius, angular rate rad/s, initial
# phase], used **only** when there is no dynamic_paths.yaml (i.e. when this node is
# started by hand):
# with the generated world and parameter file in place the cubes follow their own
# `<colour>_path` polylines.
# A phase difference of pi keeps the two cubes on opposite sides, so half of the
# 1.2 m ring is always free, and the initial phase +-pi/2 puts neither cube in front
# of the spawn pose (0, 0, yaw=0) nor inside the 110 deg horizontal camera field of
# view, so "white and black are invisible at spawn while red and green are visible"
# still holds.
DEFAULT_PATHS = {
    'white': (0.0, 0.0, 1.2, 0.35, 0.5 * math.pi),
    'black': (0.0, 0.0, 1.2, 0.35, 1.5 * math.pi),
}
FALLBACK_PATH = (0.0, 0.0, 1.2, 0.35, 0.0)

# How many sides the circles are sampled into: at radius 1.2 m a 24-gon has a
# sagitta of 0.5*r*(2*pi/24)^2 ~ 4 mm, far below the detection accuracy.
LEGACY_CIRCLE_VERTICES = 24

# black is drawn dark grey rather than pure black: the RViz background is 48;48;48
# (slam.rviz), so a pure black cube has no visible outline. This affects the RViz
# marker only, not the cube itself (Gazebo and the detector decide that); a text
# label is added as well.
MARKER_RGBA = {'white': (0.95, 0.95, 0.95, 0.95), 'black': (0.10, 0.10, 0.10, 0.95)}
MARKER_RGBA_FALLBACK = (0.9, 0.2, 0.2, 0.9)      # visualizer's ROLE_RGBA['obstacle']


def model_name(color):
    """Gazebo entity name of one obstacle."""
    return MODEL_PREFIX + str(color).strip().lower()


def normalise_name(raw):
    """Strip the quoting and leading dash that Gazebo or ``ros2 echo`` sometimes adds."""
    name = str(raw).strip().strip("'").strip()
    if name.startswith('- '):
        name = name[2:].strip()
    return name


def yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw))


def legacy_circle_path(spec, vertices=LEGACY_CIRCLE_VERTICES):
    """Legacy ``[cx, cy, radius, omega, phase]`` -> ``(closed polyline, speed along it)``.

    Used when there is no generated dynamic_paths.yaml. Sampling the circle into a
    polygon rather than keeping a second ``circle_state`` formula makes
    ``world_rules.sample_loop`` the **only** kinematics implementation: position,
    heading, speed and angular rate are then computed in exactly one place.

    A radius of 0 degenerates to "parked at the centre" (a single vertex, perimeter 0, so
    ``sample_loop`` always returns that point at zero speed), which is the supported way
    of holding one colour still.
    """
    cx, cy, radius, omega, phase = (float(v) for v in spec)
    if abs(radius) < 1e-9:
        return [(cx, cy)], 0.0
    loop = [(cx + radius * math.cos(phase + 2.0 * math.pi * k / vertices),
             cy + radius * math.sin(phase + 2.0 * math.pi * k / vertices))
            for k in range(vertices)]
    return loop, abs(omega) * abs(radius)


class DynamicObstacles(Node):

    # Parameters read once at start-up: the timers and publishers are created in
    # __init__, so changing them later would have no effect -- they are rejected
    # instead of leaving behind a service that reports success and changes nothing.
    RESTART_ONLY = ('colors', 'rate_hz', 'marker_rate_hz', 'marker_topic')

    def __init__(self):
        super().__init__('dynamic_obstacles')

        self.enabled = bool(self.declare_parameter('enabled', False).value)
        # Re-entrancy guard for our own write-back of `enabled`; see _set_enabled_state.
        self._syncing = False
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        self.marker_rate_hz = float(self.declare_parameter('marker_rate_hz', 5.0).value)
        marker_topic = str(self.declare_parameter('marker_topic', '/obstacle_markers').value)
        self.frame_id = str(self.declare_parameter('frame_id', 'map').value)
        self.reset_on_enable = bool(self.declare_parameter('reset_on_enable', True).value)
        self.z = float(self.declare_parameter('obstacle_z', DEFAULT_Z).value)

        # Marker gating. By default no Gazebo ground truth is read: marker positions
        # are derived from the camera observations. Both parameters may change at run
        # time, see on_param_change (display only, the kinematics are untouched).
        self.require_observation = bool(
            self.declare_parameter('require_observation', True).value)
        self.observation_ttl_sec = float(
            self.declare_parameter('observation_ttl_sec', 30.0).value)
        observation_topic = str(
            self.declare_parameter('observation_topic', '/observations').value)
        pose_topic = str(self.declare_parameter('pose_topic', '/robot_pose').value)

        # Kinematics parameters. ROS 2 has no map/struct parameter type, so the flat
        # convention used by the other packages is followed; three per colour:
        #   <colour>_path  = [x0, y0, x1, y1, ...] closed polyline vertices (m), even
        #                    length and >= 6
        #   <colour>_speed = speed along the polyline (m/s, simulated time)
        #   <colour>_phase = start phase 0..1, multiplied by the perimeter to get the
        #                    start arc length
        # A generated dynamic_paths.yaml overrides all three; with an empty path the
        # node falls back to the legacy circles (DEFAULT_PATHS), which is what happens
        # when it is started on its own.
        colors = list(self.declare_parameter('colors', ['white', 'black']).value)
        self.colors = [str(c).strip().lower() for c in colors]
        self.loops = {}          # colour -> polyline vertices [(x, y), ...]
        self.geom = {}           # colour -> (cumulative arc lengths, perimeter)
        self.speed = {}          # colour -> speed along the loop, m/s
        self.start_phase = {}    # colour -> start phase, 0..1
        self.s = {}              # colour -> current arc length, m
        for color in self.colors:
            # The type has to be given explicitly: with `[]` as the default rclpy would
            # infer BYTE_ARRAY, and the floating point array in the override file then
            # makes the whole node fail to start with InvalidParameterTypeException.
            raw_path = [float(v) for v in self.declare_parameter(
                f'{color}_path', Parameter.Type.DOUBLE_ARRAY).value]
            speed = float(self.declare_parameter(f'{color}_speed', 0.0).value)
            phase = float(self.declare_parameter(f'{color}_phase', 0.0).value)
            if len(raw_path) >= 6 and len(raw_path) % 2 == 0:
                loop = [(raw_path[i], raw_path[i + 1]) for i in range(0, len(raw_path), 2)]
            else:
                if raw_path:
                    self.get_logger().warn(
                        f'参数 "{color}_path" 需要长度 >= 6 的偶数项数组 '
                        f'[x0, y0, x1, y1, ...]，实得 {len(raw_path)} 项，已退回缺省圆环')
                loop, default_speed = legacy_circle_path(
                    DEFAULT_PATHS.get(color, FALLBACK_PATH))
                if speed <= 0.0:
                    speed = default_speed
            self.loops[color] = loop
            self.geom[color] = wr.path_geometry(loop)
            self.speed[color] = speed
            self.start_phase[color] = phase % 1.0
            self.s[color] = self.start_phase[color] * self.geom[color][1]

        # Heading and angular rate limits: the position follows the polyline exactly,
        # while yaw only tracks it smoothly inside a chord window; wz has its own cap so
        # a sharp corner cannot spin a cube at a physically impossible rate.
        self.yaw_window = float(self.declare_parameter('yaw_window', 0.30).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 2.0).value)

        # Claim mutex (world map rules): before moving, a cube claims the cell it is about
        # to occupy and waits in place while someone else holds it. The test is "the two
        # circumscribed circles do not overlap" and is orientation independent (see
        # world_rules.CellClaims). cell_size only sets the resolution of the debug maps
        # the node keeps for inspection; <= 0 disables the whole check.
        self.cell_size = float(self.declare_parameter('cell_size', 0.10).value)
        self.cell = wr.CellClaims(self.cell_size) if self.cell_size > 0.0 else None
        # A cube is a 0.4 m square that may carry a yaw angle, so the **circumscribed**
        # radius (half diagonal) is used as its footprint: the real cube always fits
        # inside that circle, hence "circles do not overlap" implies "cubes do not
        # overlap" -- conservative but never a false negative. It is also rotation
        # invariant, so a conflict cannot flicker when a cube turns (see CellClaims).
        self.footprint = 0.5 * CUBE_SIZE * math.sqrt(2.0)
        # Priority plus back-off yielding. The claim mutex alone lets two cubes that meet
        # head to head lock up forever, each holding a cell and neither moving. Priority
        # therefore follows the order in colors: the first colour only ever waits, so it
        # always has a way out; a later colour blocked for yield_ticks ticks steps one
        # cell backwards along its own closed loop -- that cell was just released by
        # itself, so it is free. The rule matches world_rules.replay_stats tick by tick,
        # so a walk validated offline behaves the same way in simulation.
        self.yield_ticks = max(1, int(self.declare_parameter('yield_ticks', 4).value))
        # Taking the cell back one tick later would livelock (advance one tick, retreat
        # one tick = no progress), so a back-off **commits** to yield_lock ticks before
        # trying to advance again. In practice the commit is always cut short, because a
        # retreating cube stops as soon as the cell behind it is held by its peer, so the
        # value only carries the "retreat as far as allowed" meaning.
        self.yield_lock = max(1, int(self.declare_parameter('yield_lock_ticks', 40).value))
        # Escaping the deadlock: once a cube has failed to advance for mutual_after ticks
        # it is told to back off along its own loop for mutual_lock ticks (growing by
        # 1x/2x/3x/4x per round) while the peer is pinned under a "no yielding" gate. Only
        # one side is released, and candidates are ranked by how long they have been
        # blocked; world_rules.replay_stats implements the same rule and params.yaml
        # documents it. mutual_after <= 0 disables this (only the low-priority side
        # yields).
        self.mutual_after = max(0, int(self.declare_parameter('mutual_after', 120).value))
        self.mutual_lock = max(1, int(self.declare_parameter('mutual_lock', 60).value))
        self.blocked = {color: 0 for color in self.colors}
        self.retreats = {color: 0 for color in self.colors}
        # Ticks without progress -- the direct criterion for "pushed". Looking at "neither
        # cube moved" is not enough: a yielding cube does retreat every 40 ticks and even
        # advances in between, which keeps interrupting a global window.
        self.stall = {color: 0 for color in self.colors}
        self.commit = {color: 0 for color in self.colors}   # ticks left in the commit
        self.hold = {color: 0 for color in self.colors}     # ticks left in the no-yield gate
        self.mutual_rounds = 0                              # round counter of the escalation
        self.mutuals = {color: 0 for color in self.colors}  # times a cube was ordered to retreat
        # Hard keep-out circle around the current robot pose. World generation can only
        # remove the **parking spots** from the walks (see the robot section of
        # world_rules): once the robot drives off, a walk still crosses where it passed,
        # and because the cubes are teleported with /gazebo/set_entity_state no amount of
        # Nav2 planning avoids "a cube appears inside the chassis". A run-time layer is
        # therefore needed: before advancing, a cube asks whether the robot is there and,
        # if so, **reverses** along its own walk (see self.dirs).
        # The position comes from /robot_pose, i.e. the EKF pose the robot itself
        # publishes and the same information it uses for obstacle avoidance -- not Gazebo
        # ground truth, so the robot gains no unfair knowledge (ground truth is read only
        # by the markers when require_observation=false and by performance_monitor).
        self.robot_keep = float(self.declare_parameter('robot_keep', 1.10).value)
        # The keep-out radius is the fixed 1.10 m with the error budget folded in: the
        # EKF-SLAM pose the cubes see can be half a metre away from the truth, so a
        # circle sized from an estimated distance may already have the chassis inside
        # it: 0.22 (robot) + 0.283 (cube half diagonal) + 0.35 (EKF deviation)
        # + 0.25 (margin). robot_keep_sigma folds the covariance the robot reports into
        # the radius, capped by robot_keep_max; 0 keeps the fixed radius. Both inputs are
        # quantities the robot publishes itself, never ground truth.
        self.robot_keep_sigma = float(self.declare_parameter('robot_keep_sigma', 0.0).value)
        self.robot_keep_max = float(self.declare_parameter('robot_keep_max', 1.10).value)
        self.robot_sigma = 0.0       # latest position std dev from /robot_pose
        self.robot_keep_eff = self.robot_keep   # latest effective keep-out radius
        # After a reversal the cube must keep going in the new direction for this many
        # ticks before the direction is evaluated again; otherwise it would flip back and
        # forth along the edge of the keep-out circle.
        self.robot_yield = max(1, int(self.declare_parameter('robot_yield', 3).value))
        # A one-step retreat cannot leave the keep-out circle: a tick only covers
        # speed*dt (~0.07 m) against a 1.10 m radius, so a cube would be pinned inside
        # it -- forward blocked by the circle, backwards still inside it, while the robot
        # keeps closing in at 0.3 m/s. On hitting the robot the cube therefore **jumps**
        # along the new direction to the first tick that is both outside the circle and
        # clear of the other cubes; this parameter is the longest arc such a single turn
        # may cover (m). A cube moves at 0.83 m/s against the robot's 0.3 m/s, so one
        # jump is normally enough.
        self.robot_yield_arc = max(0.0, float(
            self.declare_parameter('robot_yield_arc', 3.00).value))
        # `robot_yield_arc` is the furthest distance the search may consider, not the
        # step length of a single tick. A per-tick arc of 0.85 m (~40 m/s) makes the end
        # point pass the "moving away" test while the **swept arc** still cuts through
        # the keep-out circle: the true clearance then drops to 0.4870 m, only 6 mm above
        # the 0.481 m overlap criterion. Capping the arc per tick keeps every swept
        # segment checked. The cap must not be too small, or a cube cannot leave the
        # circle in one step and can only creep an inch at a time, which fails for
        # several ticks when the robot parks near a corner of the walk. It must also
        # exceed the longest blocked segment, because the first way out is one jump along
        # the original direction (see `_turn_around`): with the 1.10 m circle the longest
        # segment is 2.99 m, so a 3.00 m window keeps the whole loop connected. What
        # really stops a cube from passing through the chassis is the **straight segment**
        # check in `_robot_cross`: a landing point is rejected when the line to it
        # crosses the robot.
        self.robot_arc_max = max(0.0, float(
            self.declare_parameter('robot_arc_max', 3.00).value))
        # Hard body circle (m) with two roles: (1) the **straight segment** of a jump must
        # stay outside it, and (2) it is the only constraint left while a cube squeezes
        # past (see `self.crossing`). The soft circle `keep_eff` means "better not to
        # enter", the hard one means "never pass through": cubes are moved with
        # set_entity_state, so Gazebo only sees a teleport and ODE performs no continuous
        # collision check -- a valid landing point whose path crosses the robot can only
        # be caught here. 0.75 m instead of the soft 1.10 m is deliberate: using the soft
        # radius would mean "a cube may not move while the robot is close", which is
        # exactly what froze it earlier. It also keeps 0.19 m of margin over the 0.564 m
        # contact criterion (Chebyshev centre distance).
        self.robot_hard_keep = max(0.0, float(
            self.declare_parameter('robot_hard_keep', 0.75).value))
        # Creep floor (m): while squeezing through the soft circle the landing point of
        # every tick must stay outside this radius (see level 2 of `_turn_around`). It is
        # looser than `robot_hard_keep` because only a jump requires the whole straight
        # segment to stay 0.75 m away; a creep step covers speed*dt (~0.07 m), so a
        # landing point at 0.70 m puts the segment at 0.70 - 0.07 = 0.63 m, still 0.07 m
        # above the 0.564 m contact criterion. 0.70 m lets a cube really squeeze past the
        # robot (otherwise a short arc cut out by the robot becomes a pocket the cube can
        # never leave) without any visible contact.
        self.robot_cross_keep = max(0.0, float(
            self.declare_parameter('robot_cross_keep', 0.70).value))
        self.dodges = {color: 0 for color in self.colors}   # ticks left in the turn cooldown
        # Retreating one tick on hitting the robot is wrong in **direction**: the walk is
        # a closed loop, so hitting the keep-out circle means the current direction is
        # blocked, and the next tick tries that same direction again -- hit, retreat, hit,
        # retreat, with a net displacement of ~0 and the cube tracing a 0.07 m limit
        # cycle along the edge of the circle. The direction therefore has to be part of
        # the state: on a hit the cube turns around and follows the new direction until it
        # hits the circle on the far side, which makes the usable path the full length of
        # the free arc instead of a few centimetres of jitter.
        self.dirs = {color: 1.0 for color in self.colors}   # +1 / -1: travel direction
        self.turns = {color: 0 for color in self.colors}    # number of turns
        # A single jump cannot always cross: the walk is a closed loop and the robot cuts
        # it into two arcs whose ends both sit on the circle, so a turn does find a clean
        # landing point on the other side and the cube bounces between the two arcs --
        # visually pinned next to the robot. Crossing in one jump is geometrically
        # impossible, because the straight segment from one side of the circle to the
        # other necessarily passes near the chassis and `_robot_cross` rejects it every
        # time. A cube therefore **creeps** across along the walk, one tick (~0.07 m) at a
        # time, with every landing point outside the body circle; while crossing,
        # `_try_claim` only tests the body circle (`hard=True`), and the soft circle
        # applies again once the cube is clear.
        self.crossing = {color: False for color in self.colors}   # squeezing through the circle
        # Turning around is the last source of local looping: with the robot parked on the
        # loop, the soft circle cuts the walk into arcs whose ends both sit on the circle,
        # so neither direction gets anywhere and a cube shuttles back and forth inside one
        # arc. The radius is not at fault, the folding back is.
        # A cube therefore must not fold back: it scans forward along the walk for a
        # landing point outside the circle, then follows an arc around the robot's
        # keep-out square the way a person walks around a car parked in the road. While
        # bypassing it **leaves the walk** and its position comes straight from `_path_at`;
        # once the landing point is reached the arc length takes over in the original
        # direction. Only that **original** direction is probed: if the reverse direction
        # counted as "still creeping", the cube has just come from there, almost always
        # finds a landing point there, the bypass never triggers and turning around stays
        # the only way out.
        self.bypass = {color: None for color in self.colors}   # colour -> bypass task, or None
        self.bypasses = {color: 0 for color in self.colors}    # number of bypasses
        # Why the last bypass attempt did not happen (colour -> text), for logging only:
        # `advance` falls back to turning around silently, so a missing bypass log does
        # not say whether the entry test failed or the path search did.
        self.bypass_note = {color: '' for color in self.colors}
        # How far ahead along the walk the search for a landing point outside the circle
        # goes (m); <= 0 disables bypassing and falls back to turning around. It has to
        # exceed the longest blocked arc, which is 2.99 m with the 1.10 m circle, so 12 m
        # leaves a four fold margin and still works when the robot splits one arc into
        # several pieces.
        self.robot_bypass_arc = max(0.0, float(
            self.declare_parameter('robot_bypass_arc', 12.00).value))
        # Extra clearance of the bypass arc over the soft circle (m): the closest point of
        # the path is the soft circle plus this value. 0.15 m makes the detour look like a
        # detour instead of a squeeze along the edge.
        self.robot_bypass_clear = max(0.0, float(
            self.declare_parameter('robot_bypass_clear', 0.15).value))
        # Static landmarks are registered as claimants as well: they never release their
        # cell, so by the world map rules a dynamic cube can never claim it. World
        # generation already keeps both sets of walks clear of the landmarks through
        # validate_scene, this is only the run-time fallback. The coordinates come from
        # the parameter overlay written by world_setup.py (world-side geometry, generated
        # at launch) and leak no ground truth: the robot still has to recognise the
        # landmarks with its camera.
        self.static_boxes = []
        self.static_half = float(self.declare_parameter('static_box_half', 0.0).value)
        raw_boxes = [float(v) for v in self.declare_parameter(
            'static_boxes', Parameter.Type.DOUBLE_ARRAY).value]
        if len(raw_boxes) % 2 == 0:
            self.static_boxes = [(raw_boxes[i], raw_boxes[i + 1])
                                 for i in range(0, len(raw_boxes), 2)]
        elif raw_boxes:
            self.get_logger().warn(
                f'参数 "static_boxes" 需要偶数项数组 [x0, y0, x1, y1, ...]，'
                f'实得 {len(raw_boxes)} 项，已忽略')

        self.last_tick = None
        self._sim_budget = 0.0       # simulated time still to advance (see on_control)
        self.poses = {}
        self.observed = {}           # colour -> (dx, dy, stamp): observation vs dead reckoning
        self.robot_pose = None       # (x, y, yaw) from /robot_pose (EKF estimate)

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_model_states, 10)
        # Gating and pose source. With require_observation=false both subscriptions stay
        # active (the only cost is deserialisation), so switching back to ground truth
        # needs no restart of the node.
        self.create_subscription(LandmarkArray, observation_topic, self.on_observations, 10)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.on_pose, 10)
        # Private names (~/) rather than relative ones: relative names expand to
        # /set_enabled and /reset, which contradicts the documented
        # /dynamic_obstacles/set_enabled, and a root-namespace /reset would collide sooner
        # or later.
        self.create_service(SetBool, '~/set_enabled', self.on_set_enabled)
        self.create_service(StartObstacles, '~/start', self.on_start)
        self.create_service(Empty, '~/reset', self.on_reset)

        # Registered after declare_parameter: parameters passed in by launch during
        # construction do not go through here, only genuine run-time changes are
        # intercepted.
        self.add_on_set_parameters_callback(self.on_param_change)

        # The control tick uses a **wall-clock** timer: under use_sim_time the ROS clock is
        # quantised to 0.1 s by /clock, so a ROS timer wakes twice per 0.1 s edge whatever
        # its period (once with dt=0.1 s and once with dt=0), which turns rate_hz=20 into
        # 7 ticks/s of dt=0.1 s. A wall-clock timer ignores the simulated clock, and the
        # simulated-time budget of on_control brings the granularity back to 1/rate_hz.
        self.create_timer(1.0 / max(1e-3, self.rate_hz), self.on_control,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.0 / max(1e-3, self.marker_rate_hz), self.on_markers,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f'动态障碍: {[model_name(c) for c in self.colors]} '
            f'trajectories={{'
            + ', '.join(f'{c}: {len(self.loops[c])} 顶点 / {self.geom[c][1]:.2f} m / '
                        f'{self.speed[c]:.2f} m/s' for c in self.colors)
            + f'}} -> enabled={self.enabled}（默认静止；带速度启动调 /dynamic_obstacles/start）')
        self.get_logger().info(
            'marker 来源：' + (
                f'仅相机观测（TTL {self.observation_ttl_sec:.1f}s，位置由 {pose_topic} + '
                f'{observation_topic} 推算）' if self.require_observation
                else 'Gazebo 真值 /gazebo/model_states（require_observation=false）'))
        self.get_logger().info(
            '占位互斥：' + (f'开（判据=外接圆不相交，两端半径 '
                       f'{self.footprint:.2f} m ⇒ 中心距 ≥ '
                       f'{2.0 * self.footprint:.3f} m，朝向无关），'
                       f'{len(self.static_boxes) if self.static_half > 0.0 else 0} 个'
                       f'静态占位者，优先级 {self.colors[0]} > ' + ' > '.join(self.colors[1:])
                       + f'，被堵 {self.yield_ticks} 拍后倒车让路'
                       f'（承诺 {self.yield_lock} 拍）'
                       + ('，顶死 ' + f'{self.mutual_after} 拍后裁定单边倒车'
                           f' {self.mutual_lock} 拍解锁'
                           if self.mutual_after > 0 else '，僵局解锁已关')
                       + '，'
                       f'离小车 {self.robot_keep:.2f} m 内倒车让开'
                       + (f'（再按 EKF σ×{self.robot_keep_sigma:.1f} 放大，'
                           f'上限 {self.robot_keep_max:.2f} m）'
                           if self.robot_keep_sigma > 0.0 else '')
                       + f'，撞小车先沿走线贴身穿过软圈（单拍落点须在 '
                       f'{self.robot_cross_keep:.2f} m 外），'
                       f'穿不过去再沿走线一跳跨过（单次最多 {self.robot_arc_max:.2f} m），'
                       + (f'跨不过去就绕行小车一侧回到走线（判据：原方向一跳跨不过、'
                           f'也沿原方向爬不出软圈；往前找落点最多 '
                           f'{self.robot_bypass_arc:.2f} m，绕行弧离软圈 '
                           f'{self.robot_bypass_clear:.2f} m），绕不过去才换向；'
                           if self.robot_bypass_arc > 0.0
                           else '跨不过去就换向（绕行已关）；')
                       + f'跳跃的直线段须整段在 {self.robot_hard_keep:.2f} m 外）'
                       if self.cell is not None else '关'))

    # ------------------------------------------------------------- cell claims

    def _claim_statics(self):
        """Register the static landmarks as permanent claimants.

        They never release a cell, so they hold it for good. The test is "circumcircles
        do not intersect", so a 0.4 m axis-aligned box is registered with its
        circumradius ``static_half*sqrt(2)``: the inner radius would let a dynamic cube
        bite into the corner of a static one.
        """
        if self.cell is None or self.static_half <= 0.0:
            return
        around = self.static_half * math.sqrt(2.0)
        for index, (x, y) in enumerate(self.static_boxes):
            self.cell.commit(f'static_{index}', x, y, around)

    # ------------------------------------------------------------------ control

    def state_at(self, color, s):
        """Pose at arc length ``s`` for this colour: (x, y, yaw, vx, vy, wz)."""
        cum, total = self.geom[color]
        return wr.sample_loop(self.loops[color], cum, total, s, self.speed[color],
                              self.yaw_window, self.max_yaw_rate)

    def rewind(self):
        """Return to the start phase (``<colour>_phase``) and clear the yielding state."""
        for color in self.colors:
            self.s[color] = self.start_phase[color] * self.geom[color][1]
            self.blocked[color] = 0
            self.stall[color] = 0
            self.commit[color] = 0
            self.hold[color] = 0
            self.mutuals[color] = 0
            self.retreats[color] = 0
            self.dodges[color] = 0
            self.dirs[color] = 1.0
            self.turns[color] = 0
            self.crossing[color] = False
            self.bypass[color] = None
            self.bypass_note[color] = ''
        # The observation-minus-dead-reckoning offset is meaningless after the jump back to
        # home, so drop it: keeping it would add the jump to the previous difference.
        self.observed.clear()
        self._sim_budget = 0.0
        self.mutual_rounds = 0
        if self.cell is not None:
            self.cell.reset()
            self._claim_statics()

    def on_control(self):
        """Advance by simulated time, ``1/rate_hz`` simulated seconds per tick.

        ``dt`` cannot come from ``get_clock()``: under ``use_sim_time`` the clock is
        quantised by ``/clock``, which publishes at only 7~8 Hz with increments of
        exactly 0.100000 s, so the difference is either 0.1 or 0. With such a dt
        ``rate_hz`` is meaningless: a 20 Hz configuration runs at 7 ticks/s of
        dt=0.1 s, five times the designed ``sim_rate_hz`` (see ``world_rules``), and
        every tick-based duration (``yield_ticks``, ``mutual_after``, ``mutual_lock``)
        is stretched by the same factor -- the rules verified offline are not the ones
        running in the simulation.

        Each wall-clock wake-up therefore only adds the simulated span to a budget and
        spends it in whole multiples of ``1/rate_hz``. The **granularity** stays at
        the designed ``sim_rate_hz`` while the **total** still follows simulated time,
        so the on-screen speed remains ``speed x RTF``.
        """
        if not self.enabled:
            return
        now = self.get_clock().now()
        if self.last_tick is None:
            # First wake-up or right after a reset: record the baseline only. Starting
            # from /clock 0 the span covers the whole simulated epoch and would fling a
            # cube to the far end of its walk.
            self.last_tick = now
            self._sim_budget = 0.0
            return
        span = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if span <= 0.0:
            return
        self._sim_budget = min(self._sim_budget + span, SIM_BUDGET_MAX)
        step = 1.0 / max(1e-3, self.rate_hz)
        for _ in range(SIM_STEPS_MAX):
            if self._sim_budget < step:
                break
            self._sim_budget -= step
            self.advance(step)

    def _robot_blocks(self, state, keep=None):
        """Whether this tick's position is inside the robot keep-out region (Chebyshev).

        ``keep=None`` uses the **soft circle** ``robot_keep + robot_keep_sigma * sigma``
        (``sigma`` comes from the ``/robot_pose`` covariance diagonal, capped by
        ``robot_keep_max``), which means "better to stay out".

        An explicit ``keep`` is checked against that radius without touching
        ``robot_keep_eff``. Two variants use it: ``robot_hard_keep`` (0.75) while a cube
        is squeezing through the soft circle (see ``self.crossing``), and
        ``robot_cross_keep`` (0.70), the absolute creep floor (see ``_turn_around``).

        With ``robot_keep = 0`` or no ``/robot_pose`` yet everything is allowed: cubes
        should follow their walk rather than freeze the scene waiting for a topic.
        """
        if self.robot_pose is None or self.robot_keep <= 0.0:
            self.robot_keep_eff = 0.0
            return False
        if keep is None:
            keep = self.robot_keep + self.robot_keep_sigma * self.robot_sigma
            if self.robot_keep_max > 0.0:
                keep = min(keep, self.robot_keep_max)
            self.robot_keep_eff = keep
        if keep <= 0.0:
            return False
        rx, ry, _ = self.robot_pose
        return max(abs(state[0] - rx), abs(state[1] - ry)) < keep

    def _probe(self, color, s, keep=None):
        """``(can_stand, blocker)``; pure query, ``_turn_around`` scans candidates with it."""
        state = self.state_at(color, s % self.geom[color][1])
        if self._robot_blocks(state, keep):
            return False, 'robot'
        if self.cell is not None:
            blocker = self.cell.conflict(color, state[0], state[1], self.footprint)
            if blocker is not None:
                return False, blocker
        return True, None

    def _robot_cross(self, ax, ay, bx, by, rx, ry, keep=None):
        """Whether segment ``(ax,ay)-(bx,by)`` crosses the robot box of radius ``keep``.

        ``keep`` defaults to ``robot_hard_keep`` (0.75, the jump variant);
        ``_crosses_robot`` passes the looser ``ROBOT_NO_CROSS`` (0.55, the "never through
        the chassis" variant).

        Cubes are moved by ``/gazebo/set_entity_state``, so Gazebo does not interpolate
        and ODE checks no continuous collision: whether they touch follows from the
        landing point alone, but whether they *look* like crossing follows from the
        **segment between the two points**. A valid landing point whose segment runs
        through the robot reads as a cube passing through it. The code samples points
        along the line (Chebyshev, the measure used by the claims) and takes the
        closest approach to the robot.
        """
        if keep is None:
            keep = self.robot_hard_keep
        if keep <= 0.0:
            return False
        span = math.hypot(bx - ax, by - ay)
        n = max(2, int(span / (0.5 * keep)) + 1)
        for i in range(n + 1):
            t = i / float(n)
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            if max(abs(x - rx), abs(y - ry)) < keep:
                return True
        return False

    def _crosses_robot(self, color, state):
        """Whether the jump's straight segment cuts through the robot, ``force`` included.

        The world rule "cubes never pass through the robot" lives here. Cubes are moved
        with ``set_entity_state``, so Gazebo does not interpolate and ODE checks no
        continuous collision: a valid landing point whose segment runs through the robot
        reads as a cube crossing it. The escape variants of ``_turn_around`` use
        ``force=True`` to move a cube out of the keep-out region, and a walk that passes
        through the robot would let such a cube hop across it, so they are covered here.

        The one exception is a cube that is **already** inside ``ROBOT_NO_CROSS``, pushed
        there by the robot itself (``robot_hard_keep`` = 0.75 > 0.55): moving beats
        freezing, which would pin the cube under the robot (see the escape variant).
        """
        if self.robot_pose is None or ROBOT_NO_CROSS <= 0.0:
            return False
        if self.bypass[color] is not None:
            return False            # bypass paths are checked by _path_clear, not here
        rx, ry = self.robot_pose[0], self.robot_pose[1]
        here = self.state_at(color, self.s[color])
        if max(abs(here[0] - rx), abs(here[1] - ry)) < ROBOT_NO_CROSS:
            return False
        return self._robot_cross(here[0], here[1], state[0], state[1], rx, ry,
                                 ROBOT_NO_CROSS)

    def _escape_arc(self, color, step, sign, rx, ry, mine, span, guard=True):
        """Scan the arc along ``sign*step``; returns ``(clean_k, creep_k, creep_dist)``.

        "Clean" means a jump that leaves the soft circle without hitting another cube;
        "creep" is the candidate farthest from the robot on that arc, used to move
        outwards when both sides are boxed in. Both require the jump's **straight
        segment** to miss the robot (see ``_robot_cross``). ``guard=False`` has a single
        use: the cube already sits inside the hard circle because the robot drove in,
        where moving beats freezing and crossing no longer matters.
        """
        base = self.s[color]
        here = self.state_at(color, base)
        px, py = here[0], here[1]
        clean, creep, creep_d = 0, 0, mine
        for k in range(1, span + 1):
            s = base + sign * k * step
            st = self.state_at(color, s)
            if guard and self._robot_cross(px, py, st[0], st[1], rx, ry):
                break                 # runs over the robot: no distance is safe
            d = max(abs(st[0] - rx), abs(st[1] - ry))
            if d > creep_d:
                creep, creep_d = k, d
            if clean:
                continue
            ok, why = self._probe(color, s)
            if why is not None and why != 'robot':
                break                 # blocked by another cube
            if ok:
                clean = k
        return clean, creep, creep_d

    def _turn_around(self, color, step):
        """Find a way along the walk when the robot keep-out region blocks the tick.

        Returns ``(s, why, k, sign)``. ``sign`` = ``+1`` for a jump along ``step`` (no
        reversal) and ``-1`` for ``-step``, where the caller flips ``self.dirs``. It is
        not the same as ``_try_claim(back=...)``: ``back`` only looks at the sign of the
        arc step, and while the walk runs backwards "continue along step" is itself
        ``back``.

        ``step`` is the **signed** arc length of one tick (``speed*dt*dirs``). Hitting the
        circle means the next position lands inside it, not that the walk beyond it is
        blocked, so four variants are tried in increasing cost: **cross in the current
        direction** (the reverse direction is tried only when that has no landing point)
        -> **creep along the body** -> **move outwards first** -> **escape**. The code
        marks each variant by number.

        The current direction has priority and is **not** compared against the reverse
        one: reversing is almost always valid because the cube came from there, so it
        would win with a single tick and the cube would bounce between the two edges of
        the soft circle instead of leaving it. Reversal is therefore reserved for the
        case where the current direction has no landing point at all, and the creep
        variant is the answer there.

        Why codes: ``'cross'`` creep along the body / ``'creep'`` move outwards first /
        ``'escape'`` leave the body circle / ``'robot'`` with ``k=0``, no way out, so
        the caller waits.
        """
        span = self._yield_span(color, step)
        rx, ry = self.robot_pose[0], self.robot_pose[1]
        here = self.state_at(color, self.s[color])
        mine = max(abs(here[0] - rx), abs(here[1] - ry))
        fwd = self._escape_arc(color, step, 1.0, rx, ry, mine, span)
        back = self._escape_arc(color, step, -1.0, rx, ry, mine, span)
        # Current direction first, and **not** compared against the reverse direction:
        # reversing is almost always valid because the cube came from there, so it would
        # win with a single tick and the cube would bounce between the two edges of the
        # soft circle instead of leaving it. Reversal is reserved for the case where the
        # current direction has no landing point at all, where creeping is the answer.
        picks = [(fwd[0], 1.0)]
        if fwd[0] <= 0:
            picks.append((back[0], -1.0))
        for k, sign in sorted([p for p in picks if p[0] > 0], key=lambda p: p[0]):
            got = self._try_claim(color, self.s[color] + sign * k * step,
                                  back=sign * step < 0.0)
            if got[0] is not None:
                return got[0], None, k, sign
        # 2. Creep along the body: when the walk **crosses** the soft circle, no jump's
        #    straight segment can avoid the robot (a chord through the centre touches it),
        #    so both variants above find no landing point while another reversal keeps the
        #    cube pinned. The answer is to squeeze past the body tick by tick, the way a
        #    person sidesteps a table: one tick is ~ speed*dt ~ 0.07 m, and a landing point
        #    outside `robot_cross_keep` (0.70 m) puts the segment at 0.70 - 0.07 = 0.63 m,
        #    still outside the 0.564 m contact threshold. While crossing,
        #    `self.crossing[color]` is True and `advance()` measures against the chassis
        #    radius instead of the soft circle, otherwise the next tick lands inside it
        #    again. Reversing almost always works (the cube came from outside the circle),
        #    so this variant also keeps the cube from freezing: if it truly cannot pass, it
        #    walks back and covers the other end of that arc.
        for sign in (1.0, -1.0):
            s = self.s[color] + sign * step
            got = self._try_claim(color, s, back=sign * step < 0.0,
                                  keep=self.robot_cross_keep)
            if got[0] is not None:
                self.crossing[color] = True
                return got[0], 'cross', 1, sign
        # 3. No clean landing on either side (circle wider than arc): move outwards first.
        sides = [(fwd[1], fwd[2], 1.0), (back[1], back[2], -1.0)]
        k, _, sign = max(sides, key=lambda x: x[1])
        if k > 0:
            got = self._try_claim(color, self.s[color] + sign * k * step,
                                  back=sign * step < 0.0, force=True)
            if got[0] is not None:
                return got[0], 'creep', k, sign
        # 4. The current position is already inside the hard circle because the robot drove
        #    in: moving beats freezing, which would let the robot press into the cubes. The
        #    robot test is dropped and the tick farthest from the robot is taken.
        if mine < self.robot_hard_keep:
            free = [(self._escape_arc(color, step, sgn, rx, ry, mine, span, guard=False),
                     sgn) for sgn in (1.0, -1.0)]
            k, _, sign = max([(f[1], f[2], sgn) for f, sgn in free], key=lambda x: x[1])
            if k > 0:
                got = self._try_claim(color, self.s[color] + sign * k * step,
                                      back=sign * step < 0.0, force=True)
                if got[0] is not None:
                    return got[0], 'escape', k, sign
        return None, 'robot', 0, 0.0

    def _yield_span(self, color, step):
        """How many ticks the yield logic may look ahead, derived from
        ``robot_yield_arc`` and never a full lap.

        Kept separate because the bypass variant (``_needs_bypass``) needs the same
        window to decide whether the current direction can still make progress; the two
        must agree, otherwise bypassing would trigger while the cube can simply cross.
        """
        total = self.geom[color][1]
        if abs(step) <= 1e-9 or self.robot_yield_arc <= 0.0:
            return 1
        arc = min(self.robot_yield_arc, max(self.robot_arc_max, abs(step)))
        return max(1, min(int(math.ceil(arc / abs(step))),
                          max(1, int(total / abs(step)) - 1)))

    def _loop_box(self, color):
        """Bounding box of this colour's walk, grown by ``BYPASS_BOX_MARGIN``.

        It bounds the bypass path. ``dynamic_obstacles`` does not know the arena size
        (that is a ``world_rules`` parameter), but it knows where its own walk is, and
        the walk already stops at ``arena_half - wall_margin - half_diag``, a full
        ``half_diag`` (0.283 m) short of the wall. Staying inside "walk bounding box +
        0.10 m" therefore cannot reach the wall and still leaves room to bypass, without
        adding another ``arena_half`` to the parameter list.
        """
        pts = self.loops[color]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        m = BYPASS_BOX_MARGIN
        return (min(xs) - m, max(xs) + m, min(ys) - m, max(ys) + m)

    def _creep_out(self, color, step, span, rx, ry, keep):
        """Creep forward tick by tick hugging the body; returns the tick count, 0 = stuck.

        Same measure as the creep variant of ``_turn_around`` (``robot_cross_keep``), but
        this only probes, it never moves the cube. Creeping moves one tick at a time, so
        if every landing point is outside ``keep`` the whole route is (one tick is
        ~0.07 m, the largest sagitta for which "landing point clear => segment clear"
        holds). Every tick on the way also goes through ``_probe``, so a tick held by
        another cube stops the creep.

        Only the current direction is scanned. Reversing is almost always valid because
        the cube came from there, and counting it as "can creep out" would disable
        bypassing entirely.
        """
        soft = self.robot_keep_eff if self.robot_keep_eff > 0.0 else self.robot_keep
        s0 = self.s[color]
        for k in range(1, span + 1):
            s = s0 + k * step
            if not self._probe(color, s, keep=keep)[0]:
                return 0            # below the creep floor or held: cannot creep
            st = self.state_at(color, s)
            if max(abs(st[0] - rx), abs(st[1] - ry)) > soft:
                return k            # outside the soft circle, every tick clear
        return 0

    def _needs_bypass(self, color, step):
        """Whether the current direction is **completely blocked**, so bypassing is worth it.

        Same two tests as the first variants of ``_turn_around`` (a jump across in the
        current direction, or creeping out along it), but this is a **pure query**:
        ``_escape_arc``, ``_creep_out`` and ``_probe`` only read state instead of moving
        the cube the way ``_try_claim`` does, so the caller can ask "bypass?" first and
        then decide which route to take.

        Both tests only ask about the **current direction**: reversing is almost always
        possible, which would keep the reversal branch ahead of bypassing for good.

        A cube **already** inside the body circle (``mine < robot_hard_keep``) always
        returns False. The robot drove in, so the escape variant of ``_turn_around``
        handles it: a bypass arc starts from the current position and would keep the
        cube in danger even longer.
        """
        if self.robot_pose is None or self.robot_keep <= 0.0 or self.robot_bypass_arc <= 0.0:
            return False
        rx, ry = self.robot_pose[0], self.robot_pose[1]
        here = self.state_at(color, self.s[color])
        mine = max(abs(here[0] - rx), abs(here[1] - ry))
        if mine < self.robot_hard_keep:
            return False
        span = self._yield_span(color, step)
        if self._escape_arc(color, step, 1.0, rx, ry, mine, span)[0] > 0:
            return False            # the current direction crosses in one jump
        if self._creep_out(color, step, span, rx, ry, self.robot_cross_keep) > 0:
            return False            # the current direction creeps out of the soft circle
        return True

    def _square_point(self, rx, ry, half, dx, dy):
        """Boundary point of the keep-out region from ``(rx,ry)`` towards ``(dx,dy)``.

        ``_robot_blocks`` uses Chebyshev distance, so the region is an **axis-aligned
        square**, not a circle, and the bypass arc follows that square: "closest approach
        = soft circle + clearance" then holds exactly. A circle approximation would allow
        ``R/sqrt(2)`` on the diagonal and would really push into the soft circle.
        """
        m = max(abs(dx), abs(dy))
        if m <= 1e-9:
            return rx + half, ry
        scale = half / m
        return rx + dx * scale, ry + dy * scale

    def _path_clear(self, color, pts, rx, ry, keep):
        """Whether the bypass path is usable: robot circle, walk bounding box, claims.

        Sampled at ~0.04 m, so a valid endpoint whose segment runs through the robot is
        rejected too (Gazebo does not interpolate and ODE checks no continuous
        collision, so endpoints alone would miss it).
        """
        box = self._loop_box(color)
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            n = max(1, int(math.hypot(bx - ax, by - ay) / 0.04) + 1)
            for j in range(n + 1):
                t = j / float(n)
                x = ax + (bx - ax) * t
                y = ay + (by - ay) * t
                if max(abs(x - rx), abs(y - ry)) < keep:
                    return False
                if not (box[0] <= x <= box[1] and box[2] <= y <= box[3]):
                    return False
                if self.cell is not None and self.cell.conflict(
                        color, x, y, self.footprint) is not None:
                    return False
        return True

    def _bypass_path(self, color, step):
        """Build a path around the robot that rejoins the walk; ``(points, s)`` or None.

        Three steps: (1) scan **forward** along the walk by ``robot_bypass_arc`` metres
        for the first landing point B that is outside the soft circle and free of claims
        (forward only, so the direction after rejoining does not flip); (2) project both
        the current point A and B radially onto the "soft circle + robot_bypass_clear"
        square to get K and J; (3) walk that square from K to J, trying both directions
        and preferring the shorter one.

        ``_path_clear`` checks the path point by point; if both directions fail it
        returns None and the caller falls back to reversing.
        """
        if self.robot_pose is None or self.robot_keep <= 0.0:
            self.bypass_note[color] = '没有小车位姿或让路已关'
            return None
        total = self.geom[color][1]
        rx, ry = self.robot_pose[0], self.robot_pose[1]
        keep = self.robot_keep_eff if self.robot_keep_eff > 0.0 else self.robot_keep
        half = keep + self.robot_bypass_clear
        here = self.state_at(color, self.s[color])
        ax, ay = here[0], here[1]
        mine = max(abs(ax - rx), abs(ay - ry))
        if mine < keep:
            self.bypass_note[color] = (
                f'自己就在软圈里（离车 {mine:.2f} m < {keep:.2f} m），得先脱身')
            return None             # inside the circle: escape first via _turn_around
        sign = 1.0 if step > 0.0 else -1.0
        res = max(0.05, abs(step))
        reach = int(math.ceil(self.robot_bypass_arc / res))
        rejoin = None
        near = 0
        held = 0
        for i in range(1, reach + 1):
            s = self.s[color] + sign * i * res
            st = self.state_at(color, s)
            if max(abs(st[0] - rx), abs(st[1] - ry)) < half:
                near += 1
                continue            # not yet outside "soft circle + clearance"
            if self._probe(color, s)[0]:
                rejoin = (s % total, st[0], st[1])
                break
            held += 1
        if rejoin is None:
            self.bypass_note[color] = (
                f'往前 {self.robot_bypass_arc:.2f} m（{reach} 拍）没有可回归的落点：'
                f'{near} 拍还在软圈+余量内、{held} 拍被占位表占住')
            return None
        s_end, bx, by = rejoin
        kx, ky = self._square_point(rx, ry, half, ax - rx, ay - ry)
        jx, jy = self._square_point(rx, ry, half, bx - rx, by - ry)
        a0 = math.atan2(ky - ry, kx - rx)
        a1 = math.atan2(jy - ry, jx - rx)
        sweep = wrap(a1 - a0)       # shortest sweep, wrapped into [-pi, pi)
        for span in (sweep, sweep - math.copysign(2.0 * math.pi, sweep)):
            # Sampled evenly in angle, with ~0.08 m edges so _path_clear stays dense.
            count = max(2, int(abs(span) * half * 1.5 / 0.08))
            arc = []
            for i in range(count + 1):
                ang = a0 + span * (i / float(count))
                arc.append(self._square_point(rx, ry, half,
                                              math.cos(ang), math.sin(ang)))
            pts = [(ax, ay)] + arc + [(bx, by)]
            if self._path_clear(color, pts, rx, ry, keep):
                self.bypass_note[color] = ''
                return pts, s_end
        self.bypass_note[color] = (f'绕行弧两个方向都不合格（落点 {bx:.2f},{by:.2f}，'
                                   f'离车 {half:.2f} m 的正方形弧被占位或越界挡下）')
        return None

    def _path_at(self, job, d):
        """Position and unit tangent ``d`` metres along the bypass path."""
        cum = job['cum']
        pts = job['pts']
        idx = 1
        while idx < len(cum) - 1 and cum[idx] < d:
            idx += 1
        seg = cum[idx] - cum[idx - 1]
        t = 0.0 if seg <= 1e-9 else (d - cum[idx - 1]) / seg
        ax, ay = pts[idx - 1]
        bx, by = pts[idx]
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        length = math.hypot(bx - ax, by - ay)
        if length <= 1e-9:
            return x, y, 0.0, 0.0
        return x, y, (bx - ax) / length, (by - ay) / length

    def _begin_bypass(self, color, step):
        """Start bypassing; True means this tick is handed over to the bypass path."""
        path = self._bypass_path(color, step)
        if path is None:
            return False
        pts, s_end = path
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        if cum[-1] <= 1e-6:
            return False
        self.bypass[color] = {'pts': pts, 'cum': cum, 'len': cum[-1],
                              'd': 0.0, 's_end': s_end, 'stall': 0,
                              'yaw': self.state_at(color, self.s[color])[2]}
        self.bypasses[color] += 1
        self.crossing[color] = False     # the creep allowance does not apply here
        self.dodges[color] = self.robot_yield
        self.blocked[color] = 0
        self.get_logger().warn(
            f'{model_name(color)} 前方 {self.robot_keep_eff:.2f} m 内是小车'
            f'（EKF σ={self.robot_sigma:.2f}），走线这一段过不去 —— 绕行 '
            f'{cum[-1]:.2f} m 到走线的另一侧继续（累计绕行 {self.bypasses[color]} 次）',
            throttle_duration_sec=5.0)
        return True

    def _step_bypass(self, color, dt):
        """Advance one tick along the bypass path; the arc length resumes on completion."""
        job = self.bypass[color]
        if job is None:
            return
        d = job['d'] + self.speed[color] * dt
        if d >= job['len']:
            self.bypass[color] = None
            self.s[color] = job['s_end']
            self.blocked[color] = 0
            self.send(color, self.state_at(color, self.s[color]))
            return
        x, y, tx, ty = self._path_at(job, d)
        # Yaw rate limit: the bypass arc turns 90 degrees at each corner of the square, so
        # taking the tangent directly would snap the orientation. A cube's orientation is
        # invisible, but a pose jump would show up as an anomaly in other probes, so this
        # follows `sample_loop` and limits the rate with `max_yaw_rate`.
        want = math.atan2(ty, tx) if (abs(tx) + abs(ty) > 1e-9) else job['yaw']
        delta = wrap(want - job['yaw'])
        limit = max(0.0, self.max_yaw_rate) * dt
        if limit > 0.0 and abs(delta) > limit:
            delta = math.copysign(limit, delta)
        job['yaw'] = wrap(job['yaw'] + delta)
        state = (x, y, job['yaw'],
                 self.speed[color] * tx, self.speed[color] * ty,
                 delta / dt if dt > 1e-9 else 0.0)
        blocker = None
        if self.cell is not None:
            blocker = self.cell.conflict(color, x, y, self.footprint)
        if blocker is None and self.robot_pose is not None and self.robot_keep > 0.0:
            # The bypass arc was drawn around "soft circle + clearance" for the robot
            # position at planning time. If the robot drives into it during the bypass the
            # arc is no longer safe, so the cube waits it out rather than squeezing through
            # (contact threshold 0.564 m, the 0.75 m hard circle being its upper bound).
            rx, ry = self.robot_pose[0], self.robot_pose[1]
            if max(abs(x - rx), abs(y - ry)) < self.robot_hard_keep:
                blocker = 'robot'
        if blocker is not None:
            # Another cube (or a robot conflict) occupies the bypass path: wait for it to
            # move away. Waiting too long means the path is unusable, a cube parked on the
            # arc being the usual cause, so bypassing is abandoned and normal yielding
            # resumes.
            job['stall'] += 1
            if job['stall'] > BYPASS_STALL_TICKS:
                self.bypass[color] = None
                who = '小车' if blocker == 'robot' else model_name(blocker)
                self.get_logger().warn(
                    f'{model_name(color)} 绕行途中被 {who} 堵住太久，'
                    f'放弃绕行，退回沿走线让路', throttle_duration_sec=5.0)
            return
        job['d'] = d
        job['stall'] = 0
        if self.cell is not None:
            self.cell.commit(color, x, y, self.footprint)
        self.send(color, state)

    def _try_claim(self, color, s, back=False, force=False, keep=None):
        """Place ``color`` at arc length ``s``; None if the robot or the claims block it.

        ``back=True`` marks a tick that walks backwards, so the twist is negated: a pose
        moving backwards while the velocity still points forward makes ODE drag the cube
        the other way, and the motion looks jerky.

        ``keep`` is described in ``_robot_blocks``: the soft circle by default,
        ``robot_cross_keep`` while squeezing through it.

        ``force=True`` skips the robot keep-out test on the landing point, and only the
        two variants of ``_turn_around`` that move outwards when neither side can be
        crossed use it; both have already checked that the segment misses the robot, or
        that the cube sits inside the hard circle anyway. It **only** skips that
        landing-point test: no jump may cut through the robot (``_crosses_robot``), the
        rule "cubes never pass through the robot" has no exception.
        """
        total = self.geom[color][1]
        s = s % total
        state = self.state_at(color, s)
        if back:
            state = (state[0], state[1], state[2], -state[3], -state[4], -state[5])
        if not force and self._robot_blocks(state, keep):
            return None, 'robot', state
        if self._crosses_robot(color, state):
            return None, 'robot', state
        if self.cell is not None:
            blocker = self.cell.conflict(color, state[0], state[1], self.footprint)
            if blocker is not None:
                return None, blocker, state
            self.cell.commit(color, state[0], state[1], self.footprint)
        self.s[color] = s
        self.send(color, state)
        return s, None, state

    def advance(self, dt):
        """Advance one tick: each cube walks ``speed * dt``, yielding if another blocks it.

        Cubes are processed in ``colors`` order, which is the priority order, mirroring
        the tick-by-tick replay of ``world_rules.replay_stats`` line for line: a
        high-priority cube that cannot claim waits, a low-priority one blocked for
        ``yield_ticks`` backs up one tick along its closed loop.

        The robot outranks that order. The walk only avoids the **dwelling points**
        known when the scene is generated, so once the robot drives away the walk may
        still pass through where it is; cubes teleport and no robot-side planning can
        avoid that. Any priority therefore looks for a way out as soon as the robot
        keep-out region blocks a tick: first a jump across the whole region in the
        current direction (the region is 1.10 m wide, covered by the 3.00 m of
        ``robot_yield_arc``), and only then a reversal (``self.dirs`` flipped), which is
        not re-evaluated for ``robot_yield`` ticks. A reversal rather than "back up one
        tick" because the walk is a closed loop: if the current direction is blocked,
        the reverse one is the way through, while backing up is cancelled by the next
        forward tick and pins the cube on the edge of the region. See ``_turn_around``.

        Concurrency is prevented in the service (``set_enabled false`` precedes
        ``start``), so no lock is needed; ``on_control`` and ``push`` run on the same
        executor thread.
        """
        if not self.cli.service_is_ready():
            self.get_logger().warn('等待 /gazebo/set_entity_state …', throttle_duration_sec=5.0)
            return
        moved_fwd = False
        for rank, color in enumerate(self.colors):
            total = self.geom[color][1]
            if self.hold[color] > 0:
                self.hold[color] -= 1
            if self.dodges[color] > 0:
                self.dodges[color] -= 1
            step = self.speed[color] * dt * self.dirs[color]
            if total <= 1e-12 or abs(step) <= 1e-12:
                continue

            # Bypass takes over: the cube leaves the walk for the bypass path.
            if self.bypass[color] is not None:
                self._step_bypass(color, dt)
                self.stall[color] = 0        # moving, not blocked
                continue

            # Squeezing past the robot: this tick checks the chassis radius only.
            keep = self.robot_cross_keep if self.crossing[color] else None

            # Promise period: this tick **backs up and nothing else**. Forward motion is not
            # blocked, otherwise "back out, then immediately reclaim the freed cell" would
            # make the cube oscillate one tick forward and one tick back: no yielding at all.
            if self.commit[color] > 0:
                self.commit[color] -= 1
                back_s, _, _ = self._try_claim(color, self.s[color] - step,
                                               back=(step > 0.0))
                if back_s is not None:
                    self.retreats[color] += 1
                    self.stall[color] += 1
                    continue
                # The cell behind is occupied as well, which is where a deadlock lives. The
                # promise is dropped and this tick returns to the normal rules, letting even
                # rank 0 be ordered backwards rather than having nobody yield.
                self.commit[color] = 0

            # Nothing has moved forward this tick yet; a successful one clears it below.
            self.stall[color] += 1

            moved, blocker, _ = self._try_claim(color, self.s[color] + step, keep=keep)
            if moved is not None:
                self.blocked[color] = 0
                self.stall[color] = 0
                moved_fwd = True
                continue

            self.blocked[color] += 1

            # The robot keep-out region blocks the tick. Priority does not apply here: the
            # "wait, never back up" rule for rank 0 avoids a head-on standoff with another
            # cube, while a parked robot would keep the cube trapped in the region. The
            # answer is to creep past, and to cross or reverse only if that fails (see
            # _turn_around); a reversal is not re-evaluated while its cooldown
            # (self.dodges) runs, so the cube does not flip direction along the region edge.
            if blocker == 'robot' and self.dodges[color] <= 0:
                # Order matters: ask "bypass?" first, because `_turn_around` is not a pure
                # query. It calls `_try_claim` to move the cube onto the first variant that
                # works (and sets `self.crossing` on the way), which is too late to undo,
                # while `_needs_bypass` only reads state and can safely run first.
                if self._needs_bypass(color, step):
                    if self._begin_bypass(color, step):
                        continue
                    self.get_logger().warn(
                        f'{model_name(color)} 原方向被小车堵死、本该绕行，但绕行路径没找到'
                        f'（{self.bypass_note.get(color) or "原因未知"}），'
                        f'退回换向', throttle_duration_sec=5.0)
                got, why, k, sign = self._turn_around(color, step)
                if got is None:
                    self.get_logger().warn(
                        f'{model_name(color)} 要给小车让路，但沿走线 '
                        f'{self.robot_yield_arc:.2f} m 都出不了禁入圈'
                        f'（{model_name(why)} 占住），原地等待',
                        throttle_duration_sec=5.0)
                    continue
                if sign < 0.0:
                    self.dirs[color] = -self.dirs[color]
                self.dodges[color] = self.robot_yield
                self.turns[color] += 1
                self.retreats[color] += 1
                tag = '原方向' if sign > 0.0 else '换向'
                if why == 'cross':
                    jump = f'贴身爬过软圈（{k} 拍 / {k * abs(step):.2f} m）'
                elif why == 'creep':
                    jump = f'先往外蹭 {k} 拍 / {k * abs(step):.2f} m'
                elif why == 'escape':
                    jump = f'脱离车身圈 {k} 拍 / {k * abs(step):.2f} m'
                else:
                    jump = ('跨过' if k == 1
                            else f'跨过 {k} 拍 / {k * abs(step):.2f} m')
                self.get_logger().warn(
                    f'{model_name(color)} 前方 {self.robot_keep_eff:.2f} m 内是小车'
                    f'（EKF σ={self.robot_sigma:.2f}），沿走线{tag}{jump}'
                    f'（累计让路 {self.turns[color]} 次）',
                    throttle_duration_sec=5.0)
                continue

            if self.dodges[color] > 0:
                got, why, _ = self._try_claim(color, self.s[color] - step,
                                              back=(step > 0.0), keep=keep)
                if got is not None:
                    self.retreats[color] += 1
                    # Yielding to the robot outranks priority between cubes: this is dodging a
                    # vehicle rather than being blocked by another cube, so it is no deadlock
                    # and the stall counter is cleared. Otherwise a parked robot would make two
                    # cubes look mutually stuck and the unlock would fire for nothing.
                    self.stall[color] = 0
                    self.get_logger().warn(
                        f'{model_name(color)} 被 {model_name(blocker)} 堵住，倒车让路'
                        f'（累计 {self.retreats[color]} 次）', throttle_duration_sec=5.0)
                else:
                    self.get_logger().warn(
                        f'{model_name(color)} 被 {model_name(blocker)} 堵住，'
                        f'倒车方向也被 {model_name(why)} 占住，继续等',
                        throttle_duration_sec=5.0)
                continue

            if self.hold[color] > 0 or (rank == 0 and self.commit[color] <= 0):
                # A high-priority cube normally only waits: it is the one others yield to,
                # and a cell it gives up would be taken by the opponent at once, leaving
                # both to shuffle in place. `hold` is the absolute "no yielding" gate the
                # unlock puts on the opponent, because the unlock needs a **one-sided**
                # retreat. `commit > 0` means this cube has been ruled the one to retreat,
                # so even rank 0 retreats: "a cube never yields" is exactly what a mutual
                # deadlock is made of.
                self.get_logger().warn(
                    f'{model_name(color)} 的下一拍位置已被 {model_name(blocker)} 占住，原地等待',
                    throttle_duration_sec=5.0)
                continue

            if self.commit[color] <= 0 and self.blocked[color] < self.yield_ticks:
                self.get_logger().warn(
                    f'{model_name(color)} 的下一拍位置已被 {model_name(blocker)} 占住，原地等待',
                    throttle_duration_sec=5.0)
                continue

            back, why, _ = self._try_claim(color, self.s[color] - step,
                                           back=(step > 0.0))
            if back is not None:
                self.retreats[color] += 1
                # Yielding commits to the full yield_lock ticks: stepping back for one tick
                # and reclaiming the cell on the next one yields nothing. (The promise is
                # dropped early when the cell behind is taken; see the yield_lock notes.)
                self.commit[color] = self.yield_lock - 1
                self.get_logger().warn(
                    f'{model_name(color)} 被 {model_name(blocker)} 堵住，倒车让路'
                    f'（累计 {self.retreats[color]} 次）', throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(
                    f'{model_name(color)} 被 {model_name(blocker)} 堵住，'
                    f'倒车方向也被 {model_name(why)} 占住，继续等', throttle_duration_sec=5.0)

        # Deadlock unlock: a cube that cannot move for mutual_after ticks in a row is ruled
        # to **reverse** and leave the crossing along the other end of its loop, while the
        # opponent gets the "no yielding" gate. This matches the tick-by-tick rule of
        # world_rules.replay_stats: both retreating at once is a symmetric limit cycle, so
        # the retreat must be one-sided.
        if self.mutual_after > 0 and self.cell is not None:
            if moved_fwd:
                self.mutual_rounds = 0
            cands = sorted(self.colors, key=lambda c: -self.stall[c])
            if self.stall[cands[0]] >= self.mutual_after:
                for color in cands:
                    if self.hold[color] > 0:
                        continue
                    # Only a cube that is **really** blocked stays a candidate: `stall >=
                    # mutual_after` is the trigger of the unlock, and testing it again here
                    # keeps the loop from sliding onto a healthy cube. A cube in its `commit`
                    # period must not be skipped either, because the one that most needs the
                    # unlock is usually the one that has been yielding. Skipping it lands the
                    # loop on a cube with `stall == 0` and flips a healthy cube around,
                    # sending it back into the opponent that is already blocked.
                    if self.stall[color] < self.mutual_after:
                        continue
                    step = self.speed[color] * dt * self.dirs[color]
                    if abs(step) <= 1e-12 or self.geom[color][1] <= 1e-12:
                        continue
                    # Retreating requires the cell behind to be free, so candidates are tried
                    # from the most-blocked one downwards, taking the first with room behind.
                    # The one blocked longest may have the opponent right behind it, and the
                    # opponent is then ruled to retreat instead, which beats a permanent lock.
                    rear = self.state_at(color, self.s[color] - step)
                    if self._robot_blocks(rear):
                        continue
                    if self.cell.conflict(
                            color, rear[0], rear[1], self.footprint) is not None:
                        continue
                    ticks = self.mutual_lock * min(self.mutual_rounds + 1, 4)
                    # A reversal, not "back up N ticks". Two cubes pressed against the same
                    # crossing along **their own** loops would drive straight back once the
                    # retreat promise expires, so `dirs` never changes and the deadlock returns
                    # with every `commit` expiry. Flipping `dirs` sends the ruled cube out of
                    # the crossing along the **other** end of its walk, which clears the side
                    # of the opponent, and it will not press back in (the robot logic cannot
                    # flip it during the `dodges` cooldown either). The `rear` test above uses
                    # the step from the direction in force, which is exactly the cell the cube
                    # will walk after the flip, so the test must stay ahead of the flip.
                    self.dirs[color] = -self.dirs[color]
                    self.commit[color] = 0
                    self.crossing[color] = False
                    self.stall[color] = 0
                    self.blocked[color] = 0
                    self.mutuals[color] += 1
                    self.mutual_rounds += 1
                    for other in self.colors:
                        if other != color:
                            self.blocked[other] = 0
                            self.hold[other] = ticks * 2
                    self.get_logger().warn(
                        f'{model_name(color)} 连着 {self.mutual_after} 拍没能前进（被 '
                        f'{"、".join(model_name(c) for c in self.colors if c != color)} 顶住），'
                        f'裁定由它换向解锁（沿走线另一端走，对家禁止退让 '
                        f'{ticks * 2} 拍）'
                        f'（累计 {self.mutuals[color]} 次）', throttle_duration_sec=5.0)
                    break

        for color in self.colors:
            self._update_crossing(color)

    def _update_crossing(self, color):
        """**Exit** condition of the squeezing state: the cube is back outside the soft circle,
        or the robot constraints are off.

        The entry condition lives in ``_turn_around``, set when the soft circle blocks the
        only single-tick creep that works. Entering without ever leaving would make the cube
        ignore the soft circle forever, so a robot could only push it out to
        ``robot_cross_keep``; leaving without ever entering would pin the cube on the edge of
        the soft circle. Both directions are needed.
        """
        if not self.crossing[color]:
            return
        if self.bypass[color] is not None:
            # While bypassing, the cube is not on the walk and `state_at(s)` is a stale pose
            # that cannot answer "is it back outside the soft circle?". Crossing is cleared
            # when the bypass begins, so it only has to stay clear.
            self.crossing[color] = False
            return
        if self.robot_pose is None or self.robot_keep <= 0.0:
            self.crossing[color] = False
            return
        if not self._robot_blocks(self.state_at(color, self.s[color])):
            self.crossing[color] = False

    def push(self):
        """Place both cubes at the current arc length without advancing time."""
        if not self.cli.service_is_ready():
            self.get_logger().warn('等待 /gazebo/set_entity_state …', throttle_duration_sec=5.0)
            return
        for color in self.colors:
            self.blocked[color] = 0
            self.stall[color] = 0
            self.commit[color] = 0
            self.hold[color] = 0
            self.dodges[color] = 0
            self.crossing[color] = False
            self.bypass[color] = None
            self.bypass_note[color] = ''
            self.send(color, self.state_at(color, self.s[color]))
        self.mutual_rounds = 0
        if self.cell is not None:
            # No conflict check here: the start poses are a fact of this tick. Static landmarks
            # are claimed first and the dynamic cubes after, and generation keeps the two walk
            # sets far from the landmarks.
            self.cell.reset()
            self._claim_statics()
            for color in self.colors:
                state = self.state_at(color, self.s[color])
                self.cell.commit(color, state[0], state[1], self.footprint)

    def send(self, color, state):
        x, y, yaw, vx, vy, wz = state
        req = SetEntityState.Request()
        req.state.name = model_name(color)
        req.state.pose = Pose(position=Point(x=float(x), y=float(y), z=self.z),
                              orientation=yaw_to_quaternion(yaw))
        # Velocity goes with the pose, otherwise ODE drags the cube.
        req.state.twist = Twist(linear=Vector3(x=float(vx), y=float(vy)),
                                angular=Vector3(z=float(wz)))
        req.state.reference_frame = 'world'
        future = self.cli.call_async(req)
        name = req.state.name
        future.add_done_callback(
            lambda fut, n=name: self.on_push_done(fut, n))

    def on_push_done(self, future, name):
        try:
            result = future.result()
        except Exception as exc:                        # noqa: BLE001 - rclpy raises broadly
            self.get_logger().warn(f'{name}: set_entity_state 调用失败 {exc}',
                                   throttle_duration_sec=5.0)
            return
        if result is not None and not result.success:
            # Usually the model is missing from the world, or is still <static>true</static>.
            self.get_logger().warn(
                f'{name}: set_entity_state 被拒绝 ({result.status_message})；'
                f'检查 world 里是否有该模型且为 <static>false</static>',
                throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- services

    def _set_enabled_state(self, want):
        """Change the run state and keep the ``enabled`` parameter truthful.

        ``self.enabled`` is no snapshot of the parameters: the service writes the new value
        back here, so the parameter tree answers what is really running. The callback that
        our own write triggers is let through by ``_syncing``, which the "use the service
        to start or stop" rule below would otherwise reject.
        """

        self.enabled = bool(want)
        self._syncing = True
        try:
            self.set_parameters([Parameter('enabled', value=self.enabled)])
        except Exception as exc:                            # noqa: BLE001 - write may fail
            self.get_logger().warn(f'回写 enabled 参数失败：{exc}', throttle_duration_sec=5.0)
        finally:
            self._syncing = False

    def _suffix(self, name):
        """``white_speed`` -> ``('white', 'speed')``; None for other parameters."""
        for color in self.colors:
            for suffix in ('_path', '_speed', '_phase'):
                if name == color + suffix:
                    return color, suffix[1:]
        return None

    def on_param_change(self, params):
        """Trajectory parameters can change at run time, but only while the cubes are idle.

        Assigning here is what makes ``ros2 param set ... white [...]`` take effect: the
        kinematics read ``self.paths``, ``self.speed`` and ``self.start_phase``, which are
        the live projections of these parameters.
        """

        for p in params:
            if p.name == 'enabled' and self._syncing:
                continue
            traj = self._suffix(p.name)
            if traj is not None:
                color, what = traj
                if self.enabled:
                    # Changing the route or the speed mid-motion would teleport the cube and
                    # desynchronise pose and twist, and the change only shows up at the next
                    # push anyway, so the cube must stop first.
                    return SetParametersResult(
                        successful=False,
                        reason=(f'色块正在运动，不能改 {what}；请先停止：'
                                'ros2 service call /dynamic_obstacles/set_enabled '
                                'std_srvs/srv/SetBool "{data: false}"'))
                if what == 'path':
                    try:
                        values = [float(v) for v in p.value]
                    except (TypeError, ValueError):
                        # p.value may be None (type mismatch) or hold a string. An exception
                        # raised in a callback travels up the executor and takes the whole node
                        # with it, and launch gives obstacles no respawn, so the cubes would stay
                        # frozen. Rejecting the request is the better answer.
                        return SetParametersResult(
                            successful=False,
                            reason=f'"{p.name}" 需要偶数项的数值数组 [x0, y0, x1, y1, ...]')
                    if len(values) < 6 or len(values) % 2 != 0:
                        return SetParametersResult(
                            successful=False,
                            reason=(f'"{p.name}" 需要长度 >= 6 的偶数项 '
                                    f'[x0, y0, x1, y1, ...]，实得 {len(values)} 项'))
                    loop = [(values[i], values[i + 1]) for i in range(0, len(values), 2)]
                    self.loops[color] = loop
                    self.geom[color] = wr.path_geometry(loop)
                    # A new route makes the arc length meaningless: reset to the start phase so
                    # the cube does not land at a random point of the new route. The direction
                    # goes back to +1 as well, which is equivalent to a fresh start.
                    self.s[color] = self.start_phase[color] * self.geom[color][1]
                    self.dirs[color] = 1.0
                    self.turns[color] = 0
                    self.dodges[color] = 0
                    self.bypass[color] = None
                    self.bypass_note[color] = ''
                    self.get_logger().info(
                        f'{color}: path 已更新为 {len(loop)} 顶点 / '
                        f'{self.geom[color][1]:.2f} m（色块静止，已生效）')
                elif what == 'speed':
                    value = float(p.value)
                    if value < 0.0:
                        return SetParametersResult(
                            successful=False, reason=f'"{p.name}" 不能为负（0 = 停住）')
                    self.speed[color] = value
                    self.get_logger().info(
                        f'{color}: speed={value:.3f} m/s（色块静止，已生效）')
                else:
                    self.start_phase[color] = float(p.value) % 1.0
                    self.s[color] = self.start_phase[color] * self.geom[color][1]
                    self.get_logger().info(
                        f'{color}: phase={self.start_phase[color]:.3f}（色块静止，已生效）')
            elif p.name == 'require_observation':
                # Only affects what RViz draws, so it is safe at any time and takes effect at
                # once rather than being listed in RESTART_ONLY, which would leave the user with
                # a "set succeeded but nothing happened" surprise.
                self.require_observation = bool(p.value)
                self.get_logger().info(
                    'marker 来源切换为：' + ('仅相机观测' if self.require_observation
                                       else 'Gazebo 真值（调试用，演示时勿开）'))
            elif p.name == 'observation_ttl_sec':
                self.observation_ttl_sec = float(p.value)
                self.get_logger().info(
                    f'观测时间窗 -> {self.observation_ttl_sec:.1f}s'
                    + ('（<=0：见过就一直画，不超时）' if self.observation_ttl_sec <= 0.0
                       else ''))
            elif p.name in ('yaw_window', 'max_yaw_rate'):
                setattr(self, p.name, float(p.value))
                self.get_logger().info(f'{p.name} -> {float(p.value):.3f}（下一拍生效）')
            elif p.name == 'yield_ticks':
                self.yield_ticks = max(1, int(p.value))
                self.get_logger().info(
                    f'被堵 {self.yield_ticks} 拍后倒车让路（下一拍生效）')
            elif p.name == 'yield_lock_ticks':
                self.yield_lock = max(1, int(p.value))
                self.get_logger().info(
                    f'让路承诺期 -> {self.yield_lock} 拍（下一拍生效）')
            elif p.name == 'mutual_after':
                self.mutual_after = max(0, int(p.value))
                self.get_logger().info(
                    f'僵局解锁触发拍数 -> {self.mutual_after}'
                    + ('（0：关闭，退回"只有低优先级让路"）' if self.mutual_after <= 0
                       else '（下一拍生效）'))
            elif p.name == 'mutual_lock':
                self.mutual_lock = max(1, int(p.value))
                self.get_logger().info(
                    f'僵局解锁单次倒车 -> {self.mutual_lock} 拍（基准值，按轮次最多 ×4）')
            elif p.name == 'robot_keep':
                self.robot_keep = float(p.value)
                self.get_logger().info(
                    f'小车禁入圈 -> {self.robot_keep:.2f} m'
                    + ('（<=0：关掉运行期让路）' if self.robot_keep <= 0.0 else '（下一拍生效）'))
            elif p.name in ('robot_keep_sigma', 'robot_keep_max'):
                setattr(self, p.name, float(p.value))
                self.get_logger().info(
                    f'{p.name} -> {float(p.value):.2f}'
                    + ('（0：不放大/不设上限）' if float(p.value) <= 0.0 else '（下一拍生效）'))
            elif p.name == 'robot_yield':
                self.robot_yield = max(1, int(p.value))
                self.get_logger().info(
                    f'撞小车换向后再沿新方向走 {self.robot_yield} 拍才允许再次换向（下一拍生效）')
            elif p.name == 'robot_yield_arc':
                self.robot_yield_arc = max(0.0, float(p.value))
                self.get_logger().info(
                    f'撞小车时沿走线最远考虑 {self.robot_yield_arc:.2f} m'
                    + ('（0：只跳一拍）' if self.robot_yield_arc <= 0.0 else '（下一拍生效）'))
            elif p.name == 'robot_arc_max':
                self.robot_arc_max = max(0.0, float(p.value))
                self.get_logger().info(
                    f'撞小车时单拍最多跨 {self.robot_arc_max:.2f} m 走线'
                    + ('（0：只跳一拍）' if self.robot_arc_max <= 0.0 else '（下一拍生效）'))
            elif p.name == 'robot_bypass_arc':
                self.robot_bypass_arc = max(0.0, float(p.value))
                self.get_logger().info(
                    f'撞小车时沿走线往前找落点最远 {self.robot_bypass_arc:.2f} m'
                    + ('（0：关掉绕行，退回纯换向）' if self.robot_bypass_arc <= 0.0
                       else '（下一拍生效）'))
            elif p.name == 'robot_bypass_clear':
                self.robot_bypass_clear = max(0.0, float(p.value))
                self.get_logger().info(
                    f'绕行弧离软圈外扩 {self.robot_bypass_clear:.2f} m（下一拍生效）')
            elif p.name == 'robot_hard_keep':
                self.robot_hard_keep = max(0.0, float(p.value))
                self.get_logger().info(
                    f'撞小车时跳跃的直线段须整段在 {self.robot_hard_keep:.2f} m 外'
                    + ('（0：关掉穿车检查）' if self.robot_hard_keep <= 0.0 else '（下一拍生效）'))
            elif p.name == 'robot_cross_keep':
                self.robot_cross_keep = max(0.0, float(p.value))
                self.get_logger().info(
                    f'撞小车时贴身爬行的单拍落点须在 {self.robot_cross_keep:.2f} m 外'
                    + ('（0：贴到车身也不拦）' if self.robot_cross_keep <= 0.0
                       else '（下一拍生效）'))
            elif p.name == 'cell_size':
                value = float(p.value)
                # The test is "circumscribed circles do not intersect", and `cell_size` only
                # sets the resolution of the debug map; rebuilding the claim table has no side
                # effect and clears the yield state as well.
                self.cell_size = value
                self.cell = wr.CellClaims(value) if value > 0.0 else None
                for color in self.colors:
                    self.blocked[color] = 0
                    self.stall[color] = 0
                    self.commit[color] = 0
                    self.hold[color] = 0
                self.mutual_rounds = 0
                if self.cell is not None:
                    self._claim_statics()
                    for color in self.colors:
                        state = self.state_at(color, self.s[color])
                        self.cell.commit(color, state[0], state[1], self.footprint)
                self.get_logger().info(
                    f'占位表已重建（调试地图格 {value:.3f} m，'
                    + ('启用' if self.cell is not None else '关闭') + '）')
            elif p.name in self.RESTART_ONLY:
                return SetParametersResult(
                    successful=False, reason=f'"{p.name}" 只在节点启动时读取，需重起节点才生效')
            elif p.name == 'enabled':
                return SetParametersResult(
                    successful=False,
                    reason=('请用 /dynamic_obstacles/start 或 /dynamic_obstacles/set_enabled 服务启停'))
        return SetParametersResult(successful=True)

    def on_set_enabled(self, request, response):
        want = bool(request.data)
        if want == self.enabled:
            response.success = True
            response.message = 'already ' + ('running' if want else 'idle')
            return response
        self._set_enabled_state(want)
        self.last_tick = None
        if want and self.reset_on_enable:
            self.rewind()
        self.push()
        self.get_logger().info(
            f'动态障碍 -> {"MOVING" if want else "idle"} '
            f'(' + ', '.join(f'{c}@s={self.s[c]:.2f}m' for c in self.colors) + ')')
        response.success = True
        response.message = 'obstacles ' + ('moving' if want else 'idle')
        return response

    def on_start(self, request, response):
        """Start moving, optionally with a new speed.

        The new speed is written into the parameters first, through the parameter callback,
        so ``param get`` stays truthful; the cubes start afterwards.
        """
        color = str(request.color).strip().lower()
        if color and color not in self.speed:
            response.success = False
            response.message = f'未知颜色 "{request.color}"；可选 {sorted(self.speed)}'
            return response
        if self.enabled:
            response.success = False
            response.message = ('色块正在运动，不能改速度；请先停止：'
                                'ros2 service call /dynamic_obstacles/set_enabled '
                                'std_srvs/srv/SetBool "{data: false}"')
            return response

        targets = [color] if color else list(self.speed)
        # speed is the recommended field and omega a legacy alias, so speed wins.
        asked = float(request.speed) if float(request.speed) > 0.0 else float(request.omega)
        if asked > 0.0:
            # Only the colors that really change are committed, keeping tree and state in step.
            wanted = [c for c in targets if abs(self.speed[c] - asked) > 1e-9]
            if wanted:
                # The second positional argument is type_, not value, so value= has to be given
                # explicitly; otherwise rclpy raises TypeError and takes the whole node down.
                try:
                    results = self.set_parameters(
                        [Parameter(f'{c}_speed', value=asked) for c in wanted])
                except Exception as exc:        # noqa: BLE001 - a bad request must not be fatal
                    # An exception escaping a service callback travels up the executor to main and
                    # takes the node down, and launch gives obstacles no respawn, so the cubes
                    # would freeze in place.
                    self.get_logger().error(f'改速度时异常：{exc}')
                    response.success = False
                    response.message = f'改速度时异常：{exc}'
                    return response
                rejected = [r for r in results if not r.successful]
                if rejected:
                    response.success = False
                    response.message = f'改速度失败：{rejected[0].reason}'
                    return response

        self._set_enabled_state(True)
        self.last_tick = None
        if self.reset_on_enable:
            self.rewind()
        self.push()

        detail = ', '.join(f'{c}={self.speed[c]:.3f}' for c in targets)
        response.success = True
        response.colors = list(targets)
        response.speed_applied = [float(self.speed[c]) for c in targets]
        # Legacy scripts read omega_applied, which is filled with the same value so they do not
        # report 0.
        response.omega_applied = list(response.speed_applied)
        response.message = f'MOVING (speed m/s: {detail})'
        self.get_logger().info(f'动态障碍 -> MOVING {detail} m/s')
        return response

    def on_reset(self, request, response):
        # Reset must **stop** the cubes rather than zeroing the phase: with enabled still true
        # the ground truth drifts over a metre within seconds, which breaks the "reset to the
        # last launch" contract and makes evaluation unreproducible. Stopping leaves the scene
        # bit-identical to a fresh launch.
        self._set_enabled_state(False)
        self.rewind()
        self.last_tick = None
        self.push()
        self.get_logger().info('动态障碍复位到起始相位（home 位姿，enabled=False）')
        return response

    # ----------------------------------------------------------------- markers

    def on_model_states(self, msg):
        self.poses = {normalise_name(n): p for n, p in zip(msg.name, msg.pose)}

    def on_pose(self, msg):
        p = msg.pose.pose
        self.robot_pose = (float(p.position.x), float(p.position.y),
                           quaternion_to_yaw(p.orientation))
        # Position uncertainty (covariance diagonal, the larger of x and y), used at run time
        # to inflate the keep-out region: the cubes only see an estimated robot pose, so the
        # region has to grow with the estimation error.
        c = msg.pose.covariance
        self.robot_sigma = math.sqrt(max(float(c[0]), float(c[7]), 0.0))

    def on_observations(self, msg):
        """The camera saw the white or black cube: record where it is now.

        The position is **projected** here, not copied from /gazebo/model_states: the
        (range, bearing) pair of the camera frame is projected into the map frame with the
        current EKF pose, which is exactly how the EKF treats landmark observations.
        White and black stay out of the state vector only because their role is
        'obstacle'.
        """
        if self.robot_pose is None:
            self.get_logger().warn('/robot_pose 还没到，观测无法投影到 map 系',
                                   throttle_duration_sec=10.0)
            return
        px, py, yaw = self.robot_pose
        now = self.get_clock().now()
        for lm in msg.landmarks:
            colour = str(lm.color).strip().lower()
            if colour not in self.speed:
                continue
            # Judge by what the detector decided, not by the name: the detector derives role from
            # whether the colour belongs to LANDS, so a threshold change that moves a band cannot
            # make this disagree with the EKF map.
            if str(lm.role).strip().lower() != 'obstacle':
                continue
            r, phi = float(lm.range), float(lm.bearing)
            if not (math.isfinite(r) and math.isfinite(phi)) or r <= 0.0:
                continue
            x, y = observed_to_map(px, py, yaw, r, phi)
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            # What is stored is the offset of the observation relative to the pose estimated at
            # this instant, not an absolute position. `_marker_pose` adds it to the **current**
            # estimated pose every tick, so the marker moves with the cube; an absolute position
            # would jump once per /observations message, which reads as teleporting.
            # This is not copying Gazebo ground truth: the position still comes from the last
            # observation plus the commands this node issued, the observation stays the only
            # external source, and the require_observation gate is unchanged.
            cx, cy = self.state_at(colour, self.s[colour])[:2]
            self.observed[colour] = (x - cx, y - cy, now)

    def on_markers(self):
        stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now()
        state = 'MOVING' if self.enabled else 'idle'
        arr = MarkerArray()
        for idx, color in enumerate(self.colors):
            pose, text = self._marker_pose(color, state, now)

            cube = Marker()
            cube.header.frame_id = self.frame_id
            cube.header.stamp = stamp
            cube.ns = 'obstacles'
            cube.id = idx
            cube.type = Marker.CUBE
            cube.lifetime.sec = 0
            cube.scale = Vector3(x=CUBE_SIZE, y=CUBE_SIZE, z=CUBE_SIZE)
            rgba = MARKER_RGBA.get(color, MARKER_RGBA_FALLBACK)
            cube.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            if pose is None:
                cube.action = Marker.DELETE
            else:
                cube.action = Marker.ADD
                cube.pose = pose
            arr.markers.append(cube)

            # The label is published **every frame**, whether or not the cube is drawn: an ADDed
            # marker with lifetime=0 stays in RViz forever and does not disappear when the cube
            # of the same id is DELETEd.
            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = 'obstacle_labels'
            label.id = idx + 1000
            label.type = Marker.TEXT_VIEW_FACING
            label.scale.z = 0.25
            # High contrast, so black is readable on a dark RViz background.
            label.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
            label.lifetime.sec = 0
            if pose is None:
                label.action = Marker.DELETE
            else:
                label.action = Marker.ADD
                label.pose = Pose(position=Point(x=pose.position.x,
                                                 y=pose.position.y,
                                                 z=pose.position.z + 0.45),
                                  orientation=Quaternion(w=1.0))
                label.text = text
            arr.markers.append(label)
        self.pub.publish(arr)

    def _marker_pose(self, color, state, now):
        """The pose to draw ``color`` at this frame, plus its label text; ``None`` to skip it.

        By default (``require_observation``) only observations the camera reported are used;
        after ``observation_ttl_sec`` without a new one the result is None and the caller
        emits a DELETE, so "cubes that cannot be seen are not drawn". Setting it false falls
        back to Gazebo ground truth, for checking that world, physics and detector agree.

        Once seen, the position is **recomputed every frame**: ``self.observed`` holds the
        offset relative to the estimated pose, and adding the **current** pose here makes the
        cube move continuously instead of waiting at the last observation for the next one to
        snap it forward.
        """
        if not self.require_observation:
            truth = self.poses.get(model_name(color))
            if truth is None:
                return None, ''
            # Build a fresh Pose rather than referencing self.poses[name]: ROS 2 Python messages
            # have reference semantics, so sharing the ModelStates object would let a later z
            # assignment corrupt the ground truth.
            return (Pose(position=Point(x=float(truth.position.x),
                                        y=float(truth.position.y),
                                        z=float(truth.position.z)),
                         orientation=truth.orientation), f'{model_name(color)} {state}')

        seen = self.observed.get(color)
        if seen is None:
            return None, ''
        age = (now - seen[2]).nanoseconds * 1e-9
        # ttl <= 0 disables the timeout, as with the visualizer's obs_ttl_sec.
        if self.observation_ttl_sec > 0.0 and age > self.observation_ttl_sec:
            return None, ''
        # Neither height nor heading is observable: the nominal height and an identity orientation
        # are used rather than the true values, which would make "derived from observations" only
        # half true.
        cx, cy = self.state_at(color, self.s[color])[:2]
        return (Pose(position=Point(x=cx + seen[0], y=cy + seen[1], z=self.z),
                     orientation=Quaternion(w=1.0)),
                f'{model_name(color)} SEEN {age:.1f}s {state}')


def main(argv=None):
    rclpy.init(args=argv)
    node = DynamicObstacles()
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
