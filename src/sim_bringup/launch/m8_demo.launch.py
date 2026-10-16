"""Colour-homing demo on the dynamic block scene.

    ros2 launch sim_bringup m8_demo.launch.py                 # Gazebo GUI
    ros2 launch sim_bringup m8_demo.launch.py gui:=false      # headless (Docker)
    ros2 launch sim_bringup m8_demo.launch.py gui:=true rviz:=true   # + RViz map view

"Drive to the landmark of colour X" needs no new interface: the goal_selector's GoToBlock
action already takes ``target_color``, meaning "which block to stand next to". This file
adds the dynamic scene and the node that animates it. It is separate from demo.launch.py
only because that file is the project-one entry point and already takes the world as a
launch argument, so all this file has to do is pass one in.

The world is worlds/landmark_world_dynamic.world: worlds/landmark_world.world plus two
non-static 0.4 m cubes (obstacle_white, obstacle_black) that scripts/dynamic_obstacles.py
pushes around; disabled (the default) they simply stand still.

Nothing about the layout stays fixed, so that a policy cannot be tuned to the positions
instead of to what the robot can see:

  * the four landmarks are drawn on a 5 cm grid inside their own quadrants (red third,
    green fourth, blue second, yellow first): the four-quadrant structure of the arena
    survives, the positions do not;
  * the cubes random-walk the whole arena and the walk is saved as a closed polyline.
    There are no territories and no patrol rings -- "they never overlap" comes from the
    cell registry alone (a cube claims the cells of its next pose and waits when somebody
    else holds them), and the loop is closed only so the node can replay it by arc length;
  * ``validate_scene`` rejects a walk covering less than ``walk_cover_min`` of the
    walkable area, or less than ``walk_cover_union`` for both together, because such a
    walk degenerates into a ballistic streak that travels far and covers almost nothing;
  * both walks keep clear of the robot's parking spot (``spawn_x`` / ``spawn_y``), which
    enters ``world_rules._blockers`` as a no-go point exactly like the landmarks: a parked
    robot cannot dodge, and the walks are frozen into the world file at launch.

The draw happens once per launch and its seed is reported in the log, so a session is
reproducible by passing ``world_seed`` back in, and nothing about it is visible to the
robot: the true poses go to the world file and ``/gazebo/model_states``, while the
landmarks are still only learned from ``/observations``.

tf_relay is deliberately not started here: demo.launch.py starts it before Nav2 and this
file includes that launch description, so starting it again would only bring up a second
node called tf_relay on the same topic.
"""

import os
import subprocess
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fwd(name, default):
    """Forward an argument to demo.launch.py, using demo.launch.py's own default."""
    return LaunchConfiguration(name, default=default)


def _generate_scene(context, script, template, params):
    """Run world_setup.py once and return ``(derived world path, override path)``.

    It has to finish synchronously during launch, because gzserver reads the world file
    exactly once: the derived world must be on disk before this launch description is
    visited. A failure does not stop the stack; it falls back to the template world and
    says so in the log, so that a launch which silently skipped the randomisation cannot
    look like a normal one.

    The ground truth stays inside this process and the generated files: no topic other than
    ``/gazebo/model_states`` leaks the draw, and the robot still has only the camera to
    find the blocks with.
    """
    scene_dir = os.path.expanduser(LaunchConfiguration('scene_dir').perform(context).strip())
    seed = LaunchConfiguration('world_seed').perform(context).strip()
    spawn_x = _fwd('spawn_x', '0').perform(context).strip()
    spawn_y = _fwd('spawn_y', '0').perform(context).strip()
    cmd = [sys.executable, script, '--template', template, '--params', params,
           '--out-dir', scene_dir, '--json']
    # The robot's spawn point becomes a no-go point for the world generator: a parked robot
    # cannot dodge and the walk is a closed loop fixed at launch, so the spot has to be
    # excluded while the walk is generated. This is the launch file knowing where it parks
    # the robot, not Gazebo ground truth.
    if spawn_x:
        cmd += ['--spawn-x', spawn_x]
    if spawn_y:
        cmd += ['--spawn-y', spawn_y]
    if seed and seed.lstrip('+-').isdigit() and int(seed) >= 0:
        cmd += ['--seed', seed]
    else:
        # No --seed means a fresh seed per launch, i.e. the world is randomised once per
        # launch and not on every tick.
        print('[world_setup] 未指定 world_seed，本次启动使用随机种子')
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except OSError as exc:
        print(f'[world_setup] 无法执行 {script}：{exc}；退回模板 world {template}')
        return template, ''
    for line in (out.stdout or '').splitlines():
        print(f'[world_setup] {line}')
    world = os.path.join(scene_dir, 'landmark_world_scene.world')
    overlay = os.path.join(scene_dir, 'dynamic_paths.yaml')
    if out.returncode != 0 or not os.path.exists(world):
        print(f'[world_setup] 生成失败（rc={out.returncode}）；退回模板 world {template}')
        for line in (out.stderr or '').splitlines():
            print(f'[world_setup:err] {line}')
        return template, ''
    if not os.path.exists(overlay):
        overlay = ''
    return world, overlay


def _nodes(context, bringup, lib, template, params):
    """Build the nodes for the world this launch ended up with.

    They cannot be built in generate_launch_description, because the world has to be
    decided first: demo.launch.py's ``world`` argument and the obstacles node's second
    parameter file both come from it.
    """
    want = LaunchConfiguration('randomize').perform(context).strip().lower()
    override = LaunchConfiguration('world').perform(context).strip()
    if override:
        print(f'[m8_demo] world={override}（命令行指定，跳过四方格随机化）')
        world, overlay = override, ''
    elif want in ('0', 'false', 'no', 'off'):
        print(f'[m8_demo] randomize=false，用模板 world：{template}')
        world, overlay = template, ''
    else:
        world, overlay = _generate_scene(
            context, os.path.join(lib, 'world_setup.py'), template, params)

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'demo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': _fwd('gui', 'true'),
            'spawn_x': _fwd('spawn_x', '0'),
            'spawn_y': _fwd('spawn_y', '0'),
            'spawn_yaw_deg': _fwd('spawn_yaw_deg', '0'),
            'min_landmarks': _fwd('min_landmarks', '2'),
        }.items())

    # config/slam.rviz is the file to open for the SLAM map, the costmaps, the markers and
    # the poses. Off by default: gzclient and rviz2 together saturate the 4 vCPUs of this
    # VM.
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(bringup, 'config', 'slam.rviz')],
                condition=IfCondition(LaunchConfiguration('rviz', default='false')),
                output='screen')

    # The driver of the black and white cubes: /gazebo/set_entity_state pushes them along
    # the closed polyline generated for this launch. landmark_detector classifies them as
    # role="obstacle" (they never enter /landmark_map) and Nav2 sees them through /scan.
    # dynamic_paths.yaml is the second parameter file and is the single source of truth
    # for the draw; the paths built into the node are only used when it runs on its own.
    # They start disabled (enabled: false in config/params.yaml) so the scene is still and
    # reproducible, and are moved through ~/start, ~/set_enabled and ~/reset.
    obstacle_params = [params] + ([overlay] if overlay else [])
    obstacles = Node(package='sim_bringup',
                     executable='dynamic_obstacles.py',
                     name='dynamic_obstacles',
                     parameters=obstacle_params,
                     output='screen')

    return [demo, obstacles, rviz]


def generate_launch_description():
    bringup = get_package_share_directory('sim_bringup')
    # scripts/ lives in <prefix>/lib/sim_bringup (install(PROGRAMS ...)), not in share/.
    lib = os.path.join(get_package_prefix('sim_bringup'), 'lib', 'sim_bringup')
    template = os.path.join(bringup, 'worlds', 'landmark_world_dynamic.world')
    params = os.path.join(bringup, 'config', 'params.yaml')
    scene_dir = os.path.join(os.path.expanduser('~'), '.ros', 'sim_bringup_scene')

    # The world is generated by scripts/world_setup.py once per launch instead of being
    # read from a file: the four static landmarks are drawn inside their own quadrants on a
    # 5 cm grid, and collision avoidance between the two cubes rests entirely on cell
    # occupancy.
    #   world:=''        (default) randomise
    #   world:=<path>    use that world verbatim, skipping the randomisation
    #   randomize:=false use the template world, skipping the randomisation
    world_arg = DeclareLaunchArgument(
        'world', default_value='',
        description='world file to use verbatim, skipping the quadrant randomisation; '
                    'empty (default) = generate a randomised world per launch')

    randomize_arg = DeclareLaunchArgument(
        'randomize', default_value='true',
        description='place the four landmarks randomly inside their quadrants at launch')

    seed_arg = DeclareLaunchArgument(
        'world_seed', default_value='-1',
        description='seed for the world randomisation; -1 = a fresh one every launch')

    scene_dir_arg = DeclareLaunchArgument(
        'scene_dir', default_value=scene_dir,
        description='where the generated world / dynamic_paths.yaml / scene.json go '
                    '(defaults to world_rules.default_scene_dir())')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='start rviz2 on config/slam.rviz (SLAM map, costmaps, markers, poses)')

    return LaunchDescription([
        rviz_arg,
        world_arg,
        randomize_arg,
        seed_arg,
        scene_dir_arg,
        # The nodes are built inside an OpaqueFunction because the world has to be settled
        # first.
        OpaqueFunction(function=_nodes,
                       args=[bringup, lib, template, params]),
    ])
