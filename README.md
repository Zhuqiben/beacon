<p align="center">
  <a href="README.md">English</a> | <a href="README.zh-CN.md">简体中文</a>
</p>

# BEACON

[![CI](https://github.com/Zhuqiben/beacon/actions/workflows/ci.yml/badge.svg)](https://github.com/Zhuqiben/beacon/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![Gazebo Classic 11](https://img.shields.io/badge/Gazebo%20Classic-11-orange.svg)](https://classic.gazebosim.org/)

**B**earing-based **E**stimation, **A**ctive **C**olour landmark h**O**ming, and **N**avigation.

A ROS 2 (Humble) simulation stack in which a TurtleBot 3 Waffle explores a Gazebo arena,
detects coloured landmark blocks with its depth camera, builds a landmark map with an
EKF-SLAM filter, and then drives itself to whichever colour it is asked for -- avoiding two
roaming black-and-white cubes on the way.

Everything runs on a single machine. There is no hardware dependency: the robot, its
camera, the sensor noise and the moving obstacles all live in simulation, and the ground
truth used for scoring comes from Gazebo itself.

## Contents

* [What the system does](#what-the-system-does)
* [Packages](#packages)
* [Requirements](#requirements)
* [Build](#build)
* [Run](#run)
* [Interfaces](#interfaces)
* [Tools](#tools)
* [Parameters](#parameters)
* [Tests](#tests)
* [Design notes](#design-notes)
* [Contributing](#contributing)
* [Citation](#citation)
* [License](#license)

## What the system does

A `GoToBlock` goal names a colour (`red`, `green`, `blue` or `yellow`). The action server
walks a small state machine:

| State | Behaviour |
|---|---|
| `SEARCHING` | Rotate / creep until the requested colour has been seen often enough to be trusted. |
| `NAVIGATING` | Hand the estimated landmark position to Nav2's `navigate_to_pose` action and drive there. |
| `APPROACHING` | Close the last stretch with a slow final creep to the requested standoff distance. |
| `DONE` / `ABORTED` | Report the standoff actually reached, or why the goal failed. |

While a goal is active the server keeps the camera's view of the scene reconciled with the
map, so a landmark that turns out to be somewhere else does not keep the plan alive.

## Packages

| Package | Role |
|---|---|
| `slam_interfaces` | `Landmark`, `LandmarkArray`, `LandmarkMap` messages, the `GoToBlock` action and the `StartObstacles` service. |
| `landmark_detector` | Lifecycle node: synchronised RGB-D input, HSV thresholding, range and bearing per block. Also ships a colour-picking calibration tool. |
| `ekf_slam` | Pure-numpy EKF-SLAM core plus the ROS node. Odometry prediction, landmark updates with a chi-square gate, `map->odom` TF. |
| `landmark_map` | Turns the landmark estimate into a latched `OccupancyGrid` with per-landmark inflation. |
| `goal_selector` | The `GoToBlock` action server, the FSM behind it, and the robot-to-block collision ledger. |
| `performance_monitor` | Pipeline latency and localisation-error metrics on `/performance_monitor`. |
| `visualizer` | Annotated camera stream and RViz markers for the estimated map. |
| `sim_bringup` | Launch files, worlds, models, parameters, and the command line tools below. |
| `client_node` | Interactive / scriptable `GoToBlock` client. |

## Requirements

Ubuntu 22.04 (Jammy) on x86_64, with ROS 2 Humble and Gazebo Classic 11:

* ROS 2 Humble (`ros-humble-desktop`)
* Gazebo Classic 11 with `gazebo_ros`
* `turtlebot3_gazebo` and `turtlebot3_description` (the Waffle model and its diff-drive plugin)
* Nav2, including `nav2_collision_monitor`
* `python3-numpy`, `python3-scipy`, `python3-opencv`, `cv_bridge`, `xacro`

All of them are installed by one idempotent script:

```bash
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
./scripts/install_deps.sh
```

[docs/INSTALL.md](docs/INSTALL.md) lists every package, the manual `apt` route, the ROS 2
apt repository bootstrap and the usual virtual machine pitfalls.
`./scripts/install_deps.sh --check` reports what is missing without changing the machine.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run it from the workspace root — the directory that holds `src/` — and source ROS 2 first,
otherwise `colcon` cannot find the packages that `sim_bringup` and `slam_interfaces` are
built against. A successful build ends with `Summary: 9 packages finished [...]`; a fresh
shell needs `source install/setup.bash` before anything from this workspace is visible.

[docs/BUILD.md](docs/BUILD.md) covers the rest: what each package produces, why
`--symlink-install`, which edits need a rebuild and which do not, building a single package
(`--packages-up-to`), when to wipe `build/` and `install/`, and the build failures seen in
practice. [docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) walks the whole thing once, from clone
to a completed goal.

## Run

Full stack with the Gazebo GUI:

```bash
ros2 launch sim_bringup demo.launch.py
```

Headless (containers, CI):

```bash
ros2 launch sim_bringup demo.launch.py gui:=false
```

Then ask for a colour from a second terminal:

```bash
ros2 run client_node client --color red
```

`demo.launch.py` accepts a handful of arguments; `ros2 launch sim_bringup demo.launch.py -s`
lists them all. The interesting ones are `world` (which scene to load), the `spawn_x` /
`spawn_y` / `spawn_yaw_deg` triple, `min_landmarks` (how many valid landmark updates an EKF
correction needs) and `creep_topic` (where the final creep twist is published).

[docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) is the same thing from the other end: a numbered
path from `git clone` to a completed `GoToBlock` goal, with something to check at each step.

### Randomised scene

`m8_demo.launch.py` generates a fresh arena from a seed and launches the same stack on it,
with the two roaming obstacles animated:

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490
```

The seed drives landmark placement and the two cubes; the same seed always produces the same
scene. With no `world_seed` a random one is drawn and printed. Add `rviz:=true` for the SLAM
map view, or `gui:=false` for a headless run.

## Interfaces

| Name | Type | Direction |
|---|---|---|
| `goto_block` | `slam_interfaces/action/GoToBlock` | action, served by `goal_selector` |
| `~/start`, `~/set_enabled`, `~/reset` | `slam_interfaces/srv/StartObstacles`, `std_srvs/srv/SetBool`, `std_srvs/srv/Empty` | services, served by the `dynamic_obstacles` node |
| `/observations` | `slam_interfaces/msg/LandmarkArray` | detector -> SLAM |
| `/landmark_map` | `slam_interfaces/msg/LandmarkMap` | SLAM -> map, visualiser, goal selector |
| `/robot_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | SLAM estimate |
| `/map` | `nav_msgs/msg/OccupancyGrid` | latched, from `landmark_map` |
| `/landmark_markers`, `/observations/annotated` | `visualization_msgs/msg/MarkerArray`, `sensor_msgs/msg/Image` | RViz / debug |
| `/performance_monitor`, `/performance_report` | `std_msgs/msg/Float32MultiArray`, `std_msgs/msg/String` | metrics |
| `/reset_slam` | `std_srvs/srv/Empty` | clears the filter and reinitialises the map |

## Tools

```bash
ros2 run sim_bringup world_setup.py        # one randomisation pass; writes a world file
ros2 run sim_bringup world_rules.py        # the placement and roaming rules themselves
ros2 run sim_bringup dynamic_obstacles.py  # drives the two cubes
ros2 run sim_bringup benchmark.py          # metrics against /gazebo/model_states ground truth
ros2 run sim_bringup visibility_map.py     # which arena positions can see how many landmarks
ros2 run sim_bringup tf_relay.py           # /tf freshness relay
ros2 run landmark_detector sample_color    # click a pixel, print the HSV thresholds to paste
```

The `sim_bringup` tools keep their `.py` suffix because they are installed straight from
`scripts/` by `install(PROGRAMS ...)`; `sample_color` is a console script and has none.
`ros2 pkg executables sim_bringup` lists what is actually installed.

## Parameters

| File | Contents |
|---|---|
| `sim_bringup/config/params.yaml` | Detector thresholds, EKF noise and gating, map inflation, obstacle speeds. |
| `sim_bringup/config/nav2_params.yaml` | The Nav2 controller, planner and collision monitor configuration. |
| `ekf_slam/config/ekf_params.yaml` | Filter defaults as loaded by the node. |
| `landmark_detector/config/detector_params.yaml` | Colour thresholds per landmark. |
| `landmark_map/config/map_params.yaml` | Grid resolution, extent and inflation. |
| `goal_selector/config/goal_selector_params.yaml` | Standoffs, timeouts and retry policy. |

Parameters are declared flat rather than nested; the launch files and `world_setup.py` pass
them as `key:=value` overrides.

## Tests

```bash
cd ~/ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q
```

The suites cover the pure layers (EKF core, coordinate transforms, colour thresholds, the
FSM and the collision ledger), so they run without a ROS graph or Gazebo.

Continuous integration runs flake8, shellcheck, a manifest and asset check, a full
`colcon build` and this same test command on ROS 2 Humble. See
[.github/workflows/ci.yml](.github/workflows/ci.yml).

## Design notes

* `nav2_navigation.launch.py` is a copy of Nav2's `navigation_launch.py`. The stock file
  remaps `velocity_smoother`'s output onto `cmd_vel`, which pushes `collision_monitor` out of
  the chain; the copy changes that one remap and puts the monitor back in.
* `tf_relay.py` fixes the latency of `/tf`. Gazebo's diff-drive plugin publishes
  `odom->base_footprint` in bursts — nothing for a few seconds, then the whole backlog at
  once — so a RELIABLE reader sees a stamp that is seconds old while a BEST_EFFORT reader of
  the same message stays fresh.

## Contributing

Contributions are welcome. [CONTRIBUTING.md](CONTRIBUTING.md) covers the development
environment, the lint and test commands, the house style for comments, and how to open a
pull request. Everyone taking part is expected to follow the
[Code of Conduct](CODE_OF_CONDUCT.md).

Bug reports and feature requests have templates in the issue tracker. Security problems go
through the private process described in [SECURITY.md](SECURITY.md) rather than a public
issue. Notable changes per release are in [CHANGELOG.md](CHANGELOG.md).

## Citation

If you use BEACON in academic work, cite it with the metadata in
[CITATION.cff](CITATION.cff); GitHub's "Cite this repository" button reads the same file.

## License

MIT. See [LICENSE](LICENSE).

Two files are derived from Apache-2.0 sources and keep that licence; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the details and the full upstream text.
