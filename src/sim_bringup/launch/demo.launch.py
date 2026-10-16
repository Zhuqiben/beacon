"""Full-system demo: Gazebo + robot + perception/SLAM + Nav2.

One command brings up the whole project:

    ros2 launch sim_bringup demo.launch.py                 # with the Gazebo GUI
    ros2 launch sim_bringup demo.launch.py gui:=false      # headless (Docker)

Notes on a few deliberate choices, all of them about the command chain:

  1. Nav2 comes from *this package* (``nav2_navigation.launch.py``), not from
     ``nav2_bringup/launch/navigation_launch.py``: the stock file remaps
     velocity_smoother's ``cmd_vel_smoothed`` output onto ``cmd_vel``, which pushes the
     collision monitor out of the chain. See the header of the vendored file.

  2. ``collision_monitor`` is launched by that vendored file and registered with the same
     lifecycle manager, because a bare ``Node(...)`` never reaches the active state.

  3. ``robot_state_publisher`` gets ``use_sim_time`` so its TF tree is stamped from
     ``/clock``; otherwise every ``lookupTransform`` at a sim timestamp fails.

  4. The robot description goes through ``xacro``: ``turtlebot3_waffle.urdf`` is an
     unprocessed template whose link names are all ``${namespace}...``.

  5. The spawn pose is exposed as ``spawn_x`` / ``spawn_y`` / ``spawn_yaw_deg``, all 0.
     Spawning away from the origin beats teleporting the robot later, because odometry and
     TF then stay consistent from t=0; a good viewpoint sees at least three landmarks,
     e.g. (1.75, 1.00) at -140 deg.

  6. The world is exposed as ``world`` (default ``worlds/landmark_world.world``) so that
     m8_demo.launch.py can put the same stack on a generated scene without forking this
     file.

  7. ``creep_topic`` is where the ``goal_selector`` final-creep / back-off twist goes,
     ``cmd_vel`` by default. Those twists bypass ``collision_monitor``, which listens to
     ``/cmd_vel_smoothed`` only, so passing ``creep_topic:=cmd_vel_smoothed`` routes them
     through ``velocity_smoother -> collision_monitor -> /cmd_vel`` and gives them a brake.

  8. ``tf_relay`` is started here. The Gazebo diff-drive plugin's RELIABLE delivery of
     ``odom->base_footprint`` settles at a backlog of several seconds while the
     BEST_EFFORT copy of the same message stays fresh, and the same stall hits
     ``map->odom`` inside the long-lived tf2 Buffer *of the ``controller_server``
     process*, where it never heals. The relay ingests ``/tf`` with BEST_EFFORT and
     re-publishes the configured edges with RELIABLE, so those buffers keep getting fresh
     samples. Both edges must be named explicitly -- ``tf_relay.py`` defaults to
     ``odom:base_footprint`` alone, with no ``'*'`` wildcard and no ``exclude_frames``.
     The two wheel edges are deliberately not relayed: nothing in the pose chain needs
     them.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

# ParameterValue is NOT in launch.substitutions (importing it from there raises
# ImportError at launch time). It lives in launch_ros.parameter_descriptions.
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('sim_bringup')
    tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    tb3_desc = get_package_share_directory('turtlebot3_description')
    gz_ros = get_package_share_directory('gazebo_ros')

    # The world is a launch argument so m8_demo.launch.py can hand over its generated
    # scene without forking this file.
    world = LaunchConfiguration('world')
    default_world = os.path.join(bringup, 'worlds', 'landmark_world.world')
    creep_topic = LaunchConfiguration('creep_topic')
    params = os.path.join(bringup, 'config', 'params.yaml')
    nav2_params = os.path.join(bringup, 'config', 'nav2_params.yaml')
    waffle_model = os.path.join(bringup, 'models', 'turtlebot3_waffle', 'model.sdf')
    urdf = os.path.join(tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    gui = LaunchConfiguration('gui')

    # Spawn pose. With 0/0/0 the robot starts at the origin, which is also the marker
    # frame of the map. Spawning it away from the origin is preferable to teleporting it
    # later: the odometry and the whole TF tree stay consistent from t=0.
    #
    # spawn_yaw_deg is in DEGREES because that is how the world frame's four landmark
    # corners are described; spawn_entity.py wants radians, hence the conversion below.
    # Passing degrees straight to -Y would spin the robot by a meaningless angle with no
    # error message.
    spawn_yaw_rad = PythonExpression(
        ['float("', LaunchConfiguration('spawn_yaw_deg'), '") * ',
         repr(math.pi), ' / 180.0'])

    # Our own model directory comes first so models/turtlebot3_waffle (the RGB-D
    # variant) shadows the stock one shipped inside turtlebot3_gazebo. The stock
    # directory must stay on the path afterwards: the overlay still loads its meshes
    # through model://turtlebot3_common/... which only exists there.
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(bringup, 'models') + os.pathsep +
        os.path.join(tb3_gz, 'models') + os.pathsep +
        os.environ.get('GAZEBO_MODEL_PATH', ''))

    # Ground truth for evaluation comes from /gazebo/model_states, which is published by
    # the gazebo_ros_state *world* plugin declared in worlds/landmark_world.world. It
    # deliberately is not loaded from here: being a WorldPlugin, `-s
    # libgazebo_ros_state.so` would only produce the Gazebo error "System is attempting
    # to load a plugin, but detected an incorrect plugin type".
    #
    # relay_frames must be given explicitly: tf_relay.py's own default is
    # ['odom:base_footprint'] alone, which does not cover map->odom, the edge that
    # freezes permanently inside controller_server's tf2 Buffer.
    relay = Node(package='sim_bringup',
                 executable='tf_relay.py',
                 name='tf_relay',
                 parameters=[{'relay_frames': ['map:odom', 'odom:base_footprint']}],
                 output='screen')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui}.items())

    # turtlebot3_description ships turtlebot3_waffle.urdf as an *unprocessed xacro
    # template*: every link name is literally "${namespace}base_link", and the file
    # declares <xacro:arg name="namespace" default=""/>. Reading it as plain text
    # therefore publishes a TF tree whose frames are named "${namespace}base_link", which
    # matches nothing: Nav2 cannot find "odom", every costmap stays inactive, no cmd_vel
    # is ever produced and the robot never moves. It must be run through xacro first,
    # exactly as upstream turtlebot3_gazebo does.
    robot_desc = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf]),
        value_type=str)

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               parameters=[{'robot_description': robot_desc,
                            'use_sim_time': True}],
               output='screen')

    spawn = Node(package='gazebo_ros',
                 executable='spawn_entity.py',
                 arguments=['-entity', 'waffle',
                            '-file', waffle_model,
                            '-x', LaunchConfiguration('spawn_x'),
                            '-y', LaunchConfiguration('spawn_y'),
                            '-z', '0.01',
                            '-Y', spawn_yaw_rad],
                 output='screen')

    detector = Node(package='landmark_detector',
                    executable='detector_node',
                    name='landmark_detector',
                    parameters=[params],
                    output='screen')

    # The detector is a LifecycleNode and does not activate itself; this helper
    # drives configure -> activate for it after Gazebo has had time to spawn.
    autostart = Node(package='landmark_detector',
                     executable='lifecycle_autostart',
                     name='detector_autostart',
                     parameters=[{'target': '/landmark_detector',
                                  'delay_sec': 6.0}],
                     output='screen')

    slam = Node(package='ekf_slam',
                executable='slam_node',
                name='ekf_slam',
                parameters=[params, {'min_landmarks': ParameterValue(
                    LaunchConfiguration('min_landmarks'), value_type=int)}],
                output='screen')

    lmap = Node(package='landmark_map',
                executable='map_node',
                name='landmark_map',
                parameters=[params],
                output='screen')

    # The final creep and the back-off manoeuvre publish Twist directly rather than
    # going through Nav2, so with the default cmd_vel they never reach
    # collision_monitor, which subscribes to /cmd_vel_smoothed only: the creep is the
    # one self-driven motion in the stack that would otherwise have no collision brake.
    # Remapping the publisher to cmd_vel_smoothed puts it back on the chain
    # (velocity_smoother -> collision_monitor -> /cmd_vel), and no Python change is
    # needed because only the publisher's topic name moves, not the one in
    # action_server.py.
    goal = Node(package='goal_selector',
                executable='action_server',
                name='goal_selector',
                parameters=[params],
                remappings=[('cmd_vel', creep_topic)],
                output='screen')

    visualizer = Node(package='visualizer',
                      executable='viz_node',
                      name='visualizer',
                      parameters=[params],
                      output='screen')

    monitor = Node(package='performance_monitor',
                   executable='monitor_node',
                   name='performance_monitor',
                   parameters=[params],
                   output='screen')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch',
                                                   'nav2_navigation.launch.py')),
        launch_arguments={'params_file': nav2_params,
                          'use_sim_time': 'true'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo client (gzclient). '
                                          'Must be false inside a container.'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world file. Default is the '
                                          'landmark_world.world shipped with this '
                                          'package; m8_demo.launch.py passes the scene '
                                          'it generates instead.'),
        DeclareLaunchArgument('spawn_x', default_value='0',
                              description='Spawn x [m]. Default 0 keeps the robot at '
                                          'the arena centre, the viewpoint the default '
                                          'world is built around.'),
        DeclareLaunchArgument('spawn_y', default_value='0',
                              description='Spawn y [m]. Default 0.'),
        DeclareLaunchArgument('spawn_yaw_deg', default_value='0',
                              description='Spawn yaw [DEGREES]. Default 0. '
                                          'Converted to radians for spawn_entity.py; '
                                          'never pass radians here.'),
        DeclareLaunchArgument('min_landmarks', default_value='2',
                              description='EKF update gating threshold: a frame whose '
                                          'valid updates are below this value is '
                                          'predicted only. Default 2 matches '
                                          'config/params.yaml; lower it to 1 to accept '
                                          'single-landmark frames.'),
        DeclareLaunchArgument('creep_topic', default_value='cmd_vel',
                              description='Topic the goal_selector final-creep / back-off '
                                          'twist goes out on. Default cmd_vel keeps those '
                                          'twists out of collision_monitor; '
                                          'cmd_vel_smoothed routes them through it and '
                                          'restores the collision brake on the creep.'),
        set_model_path,
        relay,
        gazebo,
        rsp,
        spawn,
        detector,
        autostart,
        slam,
        lmap,
        goal,
        nav2_launch,
        visualizer,
        monitor,
    ])
