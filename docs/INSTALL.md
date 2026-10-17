<p align="center">
  <a href="INSTALL.md">English</a> | <a href="INSTALL.zh-CN.md">简体中文</a>
</p>

# Installing BEACON

BEACON is simulation-only. The robot, its RGB-D camera, the sensor noise and the moving
obstacles all live in Gazebo, so no hardware, no extra driver and no network configuration
are required.

## Supported platform

| Component | Version used for development |
|---|---|
| Ubuntu | 22.04.4 LTS (Jammy), x86_64 |
| ROS 2 | Humble |
| Gazebo | Classic 11.10 |
| Python | 3.10 |

Other Ubuntu releases and other ROS 2 distributions are untested: the launch files,
the parameter files and the ROS package names in `scripts/install_deps.sh` all assume
Humble.

## Quick start

```bash
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
./scripts/install_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`install_deps.sh` installs the apt packages listed below, adds the ROS 2 apt repository
first if ROS 2 is missing, runs `rosdep` over `src/`, and finally checks that Gazebo and
the ROS packages it depends on are actually resolvable. It is idempotent, so re-running it
never breaks a working machine.

It installs into the machine, not into the current shell, so `source /opt/ros/humble/setup.bash`
still has to be run here before `colcon build`. Every new terminal needs it again.

To see what is missing without touching the machine:

```bash
./scripts/install_deps.sh --check
```

## Packages

The whole set is installed by the script. It is reproduced here so the requirements can be
reviewed or installed by hand.

| Group | Packages |
|---|---|
| ROS 2 base | `ros-humble-desktop` |
| Simulator | `gazebo`, `ros-humble-gazebo-ros-pkgs` |
| Robot model | `ros-humble-turtlebot3-description`, `ros-humble-turtlebot3-gazebo`, `ros-humble-xacro`, `ros-humble-robot-state-publisher`, `ros-humble-joint-state-publisher`, `ros-humble-joint-state-publisher-gui` |
| Navigation | `ros-humble-nav2-bringup`, `ros-humble-nav2-collision-monitor` |
| Python runtime | `python3-numpy`, `python3-scipy`, `python3-opencv` |
| Tooling | `python3-colcon-common-extensions`, `python3-rosdep2`, `python3-pip`, `python3-pytest`, `build-essential`, `cmake`, `git` |
| Repository bootstrap | `curl`, `gnupg`, `lsb-release`, `software-properties-common` |

`ros-humble-desktop` already brings in `rclpy`, `ros2cli`, `rviz2`, `tf2`, `cv_bridge`,
`message_filters`, the standard message packages and `ros2launch`. `gazebo-ros-pkgs`
brings in `gazebo_ros`, `gazebo_plugins` and `gazebo_msgs`; `nav2-bringup` brings in the
controller, planner, smoother, behaviour, BT navigator, waypoint follower, velocity
smoother and lifecycle manager servers. `nav2-collision-monitor` is listed separately
because `nav2-bringup` does not depend on it, and the navigation launch file does.

No project-specific PyPI package is used, so there is no `requirements.txt` and no
virtual environment. Everything is either a ROS 2 or a distribution package.

## Installing by hand

If ROS 2 is not on the machine yet, add the repository once:

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

Then install the workspace dependencies:

```bash
sudo apt install -y \
  ros-humble-desktop \
  gazebo ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-description ros-humble-turtlebot3-gazebo \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
  python3-numpy python3-scipy python3-opencv \
  python3-colcon-common-extensions python3-rosdep2 python3-pip python3-pytest \
  build-essential cmake git
```

`rosdep` comes from `python3-rosdep2`, which Ubuntu ships in `universe`. The ROS
repository also offers a newer `python3-rosdep`, but the two packages declare a
`Conflicts` on each other, so install only one of them.

`rosdep` is the authoritative cross-check for the `package.xml` declarations; it is worth
running even after the list above:

```bash
sudo rosdep init                                          # only if not done yet
rosdep update
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

`--ignore-src` is what makes this work. The nine BEACON packages depend on each other by
name (`sim_bringup` depends on all eight others), and those names are not in the public
rosdistro; the flag tells `rosdep` to treat a dependency it finds inside `src/` as already
satisfied instead of trying to look it up. Keep it, and expect the run to end with
`All required rosdeps installed successfully`.

## Verifying the installation

```bash
source /opt/ros/humble/setup.bash
gazebo --version                    # Gazebo 11.x
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix nav2_collision_monitor
```

Then build and test the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q
```

A successful build reports `9 packages finished`. The test suite covers the pure layers and
does not need a running ROS graph or Gazebo.

If the build misbehaves, or if you want to know which edits need a rebuild and which do
not, see [BUILD.md](BUILD.md) instead of guessing at flags.

Finally, start the simulation with the Gazebo GUI:

```bash
ros2 launch sim_bringup demo.launch.py
```

or without it, which is what a container or a CI job should use:

```bash
ros2 launch sim_bringup demo.launch.py gui:=false
```

## Troubleshooting

Build failures themselves — `colcon` not found, symlink errors, stale generated interfaces,
`install/` owned by root, a missing installed resource — are covered in
[BUILD.md](BUILD.md#build-problems). What follows is installation and run-time problems.

**Gazebo reports no usable OpenGL context, or the GUI window is blank.**
Gazebo Classic needs OpenGL 3.0 or newer. In a virtual machine enable 3D acceleration in
the hypervisor settings, and if the guest has no GPU driver fall back to software
rendering with `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch sim_bringup demo.launch.py`. For a
headless run, pass `gui:=false`.

**`colcon build` fails while creating symlinks.**
`--symlink-install` needs a filesystem that supports symbolic links. Keep the workspace on
a native Linux filesystem, for example `~/ros2_ws`, and not inside a shared folder or a
Windows-mounted path.

**The build cannot find a message from `slam_interfaces`.**
`src` has to be sourced after building: `source install/setup.bash`. A fresh terminal
needs that command again, together with `source /opt/ros/humble/setup.bash`.

**Tests import a package but fail at collection.**
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` keeps third-party pytest plugins out of the run.
Sourcing `install/setup.bash` first is what makes the workspace packages importable.

**`ModuleNotFoundError: No module named 'cv2'`.**
Install `python3-opencv`. Do not install `opencv-python` from PyPI: `cv_bridge` is built
against the distribution OpenCV.

**`xacro` is not found, or the robot never appears.**
Install `ros-humble-xacro`. The Waffle description ships as an unprocessed xacro file and
the launch file runs it through `xacro` before `robot_state_publisher` sees it.

**Nav2 fails to find a resource such as a behaviour plugin.**
`ros-humble-nav2-bringup` and `ros-humble-nav2-collision-monitor` are both required; the
navigation launch file is a vendored copy of Nav2's own launch file with the collision
monitor inserted into the chain.

**Nodes on different machines or in different shell sessions do not see each other.**
Set `ROS_DOMAIN_ID` to the same value everywhere, for example `export ROS_DOMAIN_ID=42`.
The default domain is shared with every other ROS 2 user on the network.

**`rosdep` fails with `ERROR: unable to process source ...` .**
The rosdep database is stale or was never updated: run `rosdep update`. If sources are
missing entirely, run `sudo rosdep init` first. `install_deps.sh` treats both steps as
best-effort and only warns, because the explicit apt list is what actually has to be
present; `./scripts/install_deps.sh --check` reports the truthful state afterwards.

**apt wants to remove `python3-rosdep2` in order to install `python3-rosdep`.**
The two providers conflict. Keep `python3-rosdep2` from Ubuntu `universe` unless there is
a specific reason to move to the newer ROS-repository package.

## See also

* [WALKTHROUGH.md](WALKTHROUGH.md) — the whole sequence once, from clone to a completed goal.
* [BUILD.md](BUILD.md) — the build in detail, and build troubleshooting.
* [`../README.md`](../README.md) — what the system does, how to run it, the interfaces.
