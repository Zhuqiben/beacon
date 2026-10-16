# Third-party notices

BEACON is distributed under the MIT licence (see [LICENSE](LICENSE)). Two files are copies
or derivatives of Apache-2.0 code and one configuration file begins from an upstream
default. Apache-2.0 requires the upstream notices to be retained and the changes to be
stated, so both are recorded in the file headers and summarised here.

## Apache-2.0

Full text: [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt)

### `src/sim_bringup/launch/nav2_navigation.launch.py`

* Derived from `nav2_bringup/launch/navigation_launch.py` from
  [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2).
* Copyright (c) 2018 Intel Corporation. Apache-2.0.
* The original licence header is retained and the modifications are listed at the top of
  the file. Everything outside those four changes is byte-identical to upstream.

### `src/sim_bringup/models/turtlebot3_waffle/model.sdf` and `model.config`

* Derived from `turtlebot3_gazebo/models/turtlebot3_waffle` from
  [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations).
* Copyright (c) ROBOTIS. Apache-2.0.
* The stock camera is replaced in place by the RGB-D sensor the arena needs, and the sensor
  visualisation is switched off. The exact list of changes is in the file header.
* The meshes are **not** redistributed here: they are resolved at run time from the
  `turtlebot3_common` package installed by `scripts/install_deps.sh`.

## Derived configuration

### `src/sim_bringup/config/nav2_params.yaml`

The parameter names and the default values follow Nav2's own Humble parameter set; the
structure was then rewritten for this stack and the annotations explaining the velocity
topic chain are this project's.

## Runtime dependencies

ROS 2, Gazebo Classic, Nav2, TurtleBot 3, OpenCV, NumPy and SciPy are installed by the user
rather than redistributed here, and each keeps its own licence. `docs/INSTALL.md` and the
`package.xml` of each package list them.
