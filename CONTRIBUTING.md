# Contributing to BEACON

Thanks for taking the time to look at BEACON. Bug reports, fixes, documentation
improvements and feature ideas are all welcome.

BEACON is a research-grade simulation stack: a ROS 2 Humble workspace in which a
TurtleBot 3 Waffle detects coloured landmarks with its depth camera, keeps an EKF-SLAM
estimate of them, and drives to whichever colour it is asked for. It runs entirely in
Gazebo Classic, so no robot hardware is needed to work on it.

## Before you start

* **Small fixes** (typos, a broken link, a wrong comment, a one-line bug) — open a pull
  request directly.
* **Anything larger** (a new package, a change in behaviour, a new sensor or algorithm) —
  open an issue first so the design can be agreed on before you spend time on it. This
  avoids ending up with a large patch that cannot be merged.
* **Questions** — ask in an issue. There is no chat channel.

## Getting a development environment

The supported platform is Ubuntu 22.04 (Jammy) on x86_64 with ROS 2 Humble and Gazebo
Classic 11. See [docs/INSTALL.md](docs/INSTALL.md) for the full list and for the manual
`apt` route, or [docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) for the whole sequence in order
from a fresh clone to a completed goal.

```bash
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
./scripts/install_deps.sh          # idempotent; --check only reports
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Keep the checkout on a native Linux filesystem. `--symlink-install` needs symbolic links,
which shared folders and Windows-mounted paths do not reliably provide.

[docs/BUILD.md](docs/BUILD.md) explains the build in more detail: which edits take effect
without a rebuild and which do not, how to build a single package while iterating, and when
wiping `build/` and `install/` is the right move. Read it before debugging a build problem.

## Running the tests

The suites cover the pure layers (EKF core, coordinate transforms, colour thresholds, the
goal FSM and the collision ledger), so they need neither a ROS graph nor Gazebo:

```bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q
```

Point pytest at one package while iterating:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/ekf_slam -q
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` keeps unrelated third-party pytest plugins out of the
run. Sourcing `install/setup.bash` is what makes the workspace packages importable.

## Style

Python follows PEP 8 with 4-space indentation, single-quoted strings and a 120-column
limit. The settings live in [.flake8](.flake8) and [.editorconfig](.editorconfig), so an
editor that reads EditorConfig picks them up automatically. The pinned linter version is in
[requirements-dev.txt](requirements-dev.txt):

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m flake8 src
```

Shell scripts must pass shellcheck:

```bash
shellcheck scripts/*.sh
```

The `slam_interfaces` package is plain `ament_cmake` message/action/service definitions;
keep it to the generated defaults.

Line endings are LF everywhere, enforced by [.gitattributes](.gitattributes). Do not add
trailing whitespace.

## Comments

Comments in this project explain *why*, not *what*. The ROS 2, Nav2 and Gazebo details
that the code depends on but does not show — why `collision_monitor` has to be a lifecycle
node, why a topic is remapped twice, why a parameter defaults to a bare `[]` — are worth
keeping and are the reason the code is readable at all. Comments that restate the next
line are not.

`# ---- section ----` banners, aligned trailing comments and module docstrings are the
house style; match the surrounding file.

## Project layout

```
src/
  slam_interfaces/      messages, action and service definitions (ament_cmake)
  landmark_detector/    RGB-D colour detection, lifecycle node
  ekf_slam/             EKF-SLAM core plus the ROS node
  landmark_map/         landmark estimate -> OccupancyGrid
  goal_selector/        GoToBlock action server, FSM, collision ledger
  performance_monitor/  latency and localisation-error metrics
  visualizer/           annotated camera stream and RViz markers
  client_node/          GoToBlock client
  sim_bringup/          launch files, worlds, models, parameters, CLI tools
docs/                   installation, build and walkthrough guides, in English and zh-CN
scripts/                host-side tooling (install_deps.sh)
```

Anything a package needs at runtime goes through `install(...)` in its `CMakeLists.txt` or
`data_files` in its `setup.py`, or it will not be present after a build.

## Adding a package

1. Create `src/<package_name>/` following the layout of an existing package of the same
   build type.
2. Add `package.xml` with the current version, the MIT license, the maintainer line used by
   the other packages, and a `<depend>` entry for everything you import.
3. Add `setup.py` (`ament_python`) or `CMakeLists.txt` (`ament_cmake`).
4. Register it in `src/sim_bringup/package.xml` if the launch files need it.
5. Add it to the package table in [README.md](README.md).

## Documentation

Four documents describe the project: the root [README.md](README.md), and
[docs/INSTALL.md](docs/INSTALL.md), [docs/BUILD.md](docs/BUILD.md) and
[docs/WALKTHROUGH.md](docs/WALKTHROUGH.md) for installing, building and running it.

Each has a Simplified Chinese edition next to it — `README.zh-CN.md`,
`docs/INSTALL.zh-CN.md`, `docs/BUILD.zh-CN.md`, `docs/WALKTHROUGH.zh-CN.md` — and every page
opens with a language switcher that links to its counterpart. When you change one of the
English pages, update the corresponding `zh-CN` page in the same pull request, or say in the
pull request that the translation is still to come.

Keep commands, package names, topic names, parameter names and file paths in ASCII on every
page: they are what a reader copies into a terminal.

## Commit messages

* One logical change per commit.
* Imperative subject line, at most 72 characters, no trailing full stop:
  `Fix the landmark expiry check in the EKF update`.
* Put the reasoning in the body when it is not obvious from the diff.
* Reference an issue when one exists: `Fixes #12`.

## Pull requests

* Branch off `main` and keep the change focused.
* Make sure `colcon build --symlink-install`, `python3 -m flake8 src` and the pytest suite
  all pass locally. CI runs the same three checks.
* Update the documentation when behaviour, defaults, parameters or interfaces change, and
  add an entry under `Unreleased` in [CHANGELOG.md](CHANGELOG.md).
* Describe how the change was tested. If it touches navigation or the FSM, say which launch
  file and seed you ran it with.
* Do not commit `build/`, `install/`, `log/`, `__pycache__/` or editor state; `.gitignore`
  already covers them.

Keep pull requests small. A large patch that mixes a refactor with a behaviour change is
hard to review and is likely to be asked to be split.

## Reporting bugs

Include:

* the Ubuntu and ROS 2 versions (`lsb_release -a`, `ros2 --version`),
* whether Gazebo ran with or without the GUI (`gui:=false`),
* the exact commands you ran,
* the relevant console output, and
* whether it reproduces on the default parameters, and if so the `world_seed` you used.

Please check first that the problem is not one of the ones listed in the Troubleshooting
section of [docs/INSTALL.md](docs/INSTALL.md).

## License

BEACON is released under the MIT License. By contributing you agree that your contribution
is licensed under the same terms. See [LICENSE](LICENSE).

This project follows the [Code of Conduct](CODE_OF_CONDUCT.md).
