# From clone to a completed goal

One linear path through BEACON: get the code, install the dependencies, build, run, send a
goal, and check that it worked. Every step ends with something you can verify, so a failure
is caught where it happens instead of three steps later.

The other two documents go deeper on a single step each — [INSTALL.md](INSTALL.md) on the
system side, [BUILD.md](BUILD.md) on the build. This page is the order in which you do
things. Steps 1 to 4 are one-time; steps 5 to 8 are what a normal session looks like.

## What you need

| | |
|---|---|
| OS | Ubuntu 22.04 (Jammy), x86_64 |
| Disk | about 6 GB free: ROS 2 desktop, Gazebo, and `build/` + `install/` |
| Graphics | a working OpenGL 3.0 context for the Gazebo window; software rendering works but is slow |
| Network | only for the clone and the `apt` install |

No robot, no camera and no second machine. Everything is simulated on the one host.

## 1. Get the code

```bash
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
```

**Check:** `ls src` lists nine directories — `client_node`, `ekf_slam`, `goal_selector`,
`landmark_detector`, `landmark_map`, `performance_monitor`, `sim_bringup`,
`slam_interfaces`, `visualizer`.

Keep the checkout on a native Linux filesystem, as above. A VirtualBox shared folder or an
NTFS mount cannot hold the symlinks that step 3 creates.

## 2. Install the system dependencies

```bash
./scripts/install_deps.sh
```

It adds the ROS 2 apt repository if ROS 2 is missing, installs the `apt` packages, runs
`rosdep` over `src/`, and then reports anything still unresolvable. It is idempotent and
needs `sudo` for the install part. Expect a few minutes the first time.

To look without touching the machine:

```bash
./scripts/install_deps.sh --check
```

The full package list, and the manual `apt` route for a machine where the script cannot be
used, are in [INSTALL.md](INSTALL.md).

**Check** all four, in this order — each one catches a different class of missing package:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix nav2_collision_monitor
ros2 pkg prefix turtlebot3_gazebo
gazebo --version
python3 -c 'import numpy, scipy, cv2'
```

`gazebo --version` must print `Gazebo multi-robot simulator, version 11.x`. If `ros2` itself
is not found, the ROS 2 install did not complete.

The script installs into the machine, not into your shell, so `source /opt/ros/humble/setup.bash`
is still needed here and in every new terminal.

## 3. Build the workspace

```bash
source /opt/ros/humble/setup.bash          # already done above, harmless to repeat
colcon build --symlink-install
source install/setup.bash
```

Run it from `~/ros2_ws`, the directory that contains `src/`. The first build takes roughly a
minute; only the message generation in `slam_interfaces` is compiled.

**Check:** the last line reads

```
Summary: 9 packages finished [1min 12s]
```

Any other count means a package was skipped — `colcon list` shows what it found.

`source install/setup.bash` is not optional: until it runs, `ros2` cannot see any of the
nine packages. Every new terminal needs both `source` commands, in that order.

## 4. Verify before running

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon list
ros2 pkg executables sim_bringup
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q
```

**Check:** `colcon list` prints nine packages; `ros2 pkg executables sim_bringup` prints six
tools, each with its `.py` suffix — `benchmark.py`, `dynamic_obstacles.py`, `tf_relay.py`,
`visibility_map.py`, `world_rules.py`, `world_setup.py`; and pytest ends with a line of
`N passed` and no failures.

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is required. `colcon build` installs the `launch_testing`
and `ament_*` pytest plugins, and they abort collection when loaded into a plain run.

If pytest cannot import `ekf_slam` or `goal_selector`, `install/setup.bash` was not sourced
in that shell.

## 5. First run: the fixed arena

Terminal A:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sim_bringup demo.launch.py
```

**What you should see:** a Gazebo window with a walled arena, four coloured blocks near the
walls, and a TurtleBot 3 Waffle at the centre. The terminal scrolls through the nodes
starting: `tf_relay`, `robot_state_publisher`, `spawn_entity`, `detector_node`, the
lifecycle autostart, `slam_node`, `map_node`, `action_server`, `viz_node`,
`monitor_node`.

Give it about ten seconds. The detector has to come up through its lifecycle transitions
before it publishes anything, and the SLAM filter needs the robot to see a block before the
map is worth anything.

**Check** in a third terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 node list
ros2 topic hz /observations
```

`ros2 node list` should show the nodes from the launch file, and `/observations` should
publish a few times per second as the camera sees blocks. No `/observations` means the
detector never reached `active`.

To stop, Ctrl-C in terminal A and wait for Gazebo to close. Close Gazebo with Ctrl-C, not
with the window's close button — the launch system needs the signal to bring the rest down.

## 6. Send a goal

The stack is up and doing nothing until it is asked for a colour. Terminal B:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run client_node client --color red
```

`--color` accepts `red`, `green`, `blue` or `yellow` and defaults to `yellow`.

The client prints feedback once per second:

```
[fb   1] SEARCHING    target_known=False dist= -1.00 m elapsed=  0.10 s
[fb   8] SEARCHING    target_known=True  dist=  3.42 m elapsed=  7.90 s
[fb  11] NAVIGATING   target_known=True  dist=  3.10 m elapsed= 10.90 s
[fb  34] APPROACHING  target_known=True  dist=  0.71 m elapsed= 33.90 s
```

and then the result:

```
success=True  final_distance=0.598 m
```

**Check:** `success=True` and a `final_distance` close to `0.6 m`. That is the whole system
working end to end — detection, SLAM, mapping, planning, control.

Useful flags: `--approach 0.8` for a larger standoff, `--timeout 90` for a longer deadline,
`--cancel-after 5` to exercise the abort path.

## 7. Randomised arena, moving obstacles

`demo.launch.py` always loads the same world, which is what you want while getting the
system running. The research configuration generates a fresh arena per launch and puts two
roaming cubes in it:

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490
```

The seed drives landmark placement and the two obstacle walks, so `world_seed:=1490` always
produces the same arena. Leave `world_seed` off and a fresh seed is drawn and printed — pass
it back in to reproduce that session.

Add RViz on the SLAM map, or drop the Gazebo window entirely:

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490 rviz:=true
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490 gui:=false
```

The generated world, the obstacle walks and `scene.json` are written to
`~/.ros/sim_bringup_scene/`, outside the repository. Then send a goal as in step 6.

## 8. Measure the result

`benchmark.py` scores the running stack against Gazebo's ground truth. With the stack still
up, in a third terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run sim_bringup benchmark.py --duration 60 --csv perf.csv
```

It reports localisation RMS against `/gazebo/model_states`, map error against the true
landmark poses in `scene.json`, the observation rate, and per-stage latency. `--duration 0`
runs until Ctrl-C. `--scene` points at a different `scene.json` if you run it against a world
generated earlier.

The same numbers stream live on `/performance_monitor`, and `monitor_node` writes a summary
to `/performance_report`.

## 9. Shut down

Ctrl-C in the client, then in the launch terminal. Then confirm nothing is left behind:

```bash
ros2 node list
```

An empty output means the shutdown was clean. A leftover node is usually a Gazebo process
that outlived the launch; `pkill -f gazebo` clears it.

## The whole thing, one block

```bash
# once
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
./scripts/install_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q

# every session, terminal A
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sim_bringup demo.launch.py

# every session, terminal B
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run client_node client --color red
```

## If a step fails

| Step | Symptom | Where to look |
|---|---|---|
| 1 | clone fails, or `src` is empty | network or URL; the repository is `Zhuqiben/beacon` |
| 2 | `apt` cannot find `ros-humble-*` | the ROS 2 apt repository is missing — manual route in [INSTALL.md](INSTALL.md#installing-by-hand) |
| 2 | `rosdep` reports the nine package names as unknown | `--ignore-src` was dropped from the command |
| 3 | `colcon: command not found` | `python3-colcon-common-extensions`, and source ROS 2 first |
| 3 | symlink errors | workspace is not on a native Linux filesystem |
| 3 | fewer than 9 packages built | `colcon list`; see [BUILD.md](BUILD.md#build-problems) |
| 4 | pytest cannot import a package | `install/setup.bash` not sourced |
| 4 | pytest aborts during collection | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` missing |
| 5 | blank or black Gazebo window | OpenGL — `LIBGL_ALWAYS_SOFTWARE=1`, or `gui:=false` |
| 5 | `/observations` silent | detector not `active`; check the launch log for a lifecycle error |
| 6 | `/goto_block not available` | the action server is not up — was `demo.launch.py` still running? |
| 6 | `success=False` with a timeout | `--timeout 90`, and confirm the colour is one of the four |
| 8 | map error looks meaningless | no `scene.json` for that run — pass `--scene` explicitly |

Installation problems — OpenGL, `cv2`, `xacro`, `rosdep`, Nav2 resources, `ROS_DOMAIN_ID` —
are in the troubleshooting section of [INSTALL.md](INSTALL.md#troubleshooting). Build
problems are in [BUILD.md](BUILD.md#build-problems).

## See also

* [INSTALL.md](INSTALL.md) — platform, dependencies, manual `apt` route, run-time problems.
* [BUILD.md](BUILD.md) — the build in detail, rebuild rules, build troubleshooting.
* [`../README.md`](../README.md) — what the system does, interfaces, parameters, tools.
* [`.github/workflows/ci.yml`](../.github/workflows/ci.yml) — the same build and test
  sequence, run on every push.
