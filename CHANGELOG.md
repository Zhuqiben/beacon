# Changelog

All notable changes to BEACON are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project
uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

* Dropped the summarizing sentence that opened the design notes in both READMEs. The two
  bullets now state what is unusual about `nav2_navigation.launch.py` and `tf_relay.py`
  directly, without a preamble about where the reasoning lives.

## [0.0.1] - 2026-10-16

First public release: the initial import of the BEACON stack, which has no earlier public
history.

### Added

* `slam_interfaces` — `Landmark`, `LandmarkArray` and `LandmarkMap` messages, the
  `GoToBlock` action and the `StartObstacles` service.
* `landmark_detector` — lifecycle node doing synchronised RGB-D input, HSV thresholding and
  per-block range and bearing estimation, plus an interactive colour calibration tool.
* `ekf_slam` — pure-numpy EKF-SLAM core with odometry prediction, chi-square-gated landmark
  updates and a `map->odom` transform, plus the ROS node around it.
* `landmark_map` — latched `OccupancyGrid` with per-landmark inflation built from the
  landmark estimate.
* `goal_selector` — the `GoToBlock` action server, the search/navigate/approach state
  machine and the robot-to-block collision ledger.
* `performance_monitor` — pipeline latency and localisation-error metrics.
* `visualizer` — annotated camera stream and RViz markers for the estimated map.
* `sim_bringup` — Gazebo worlds, the TurtleBot 3 Waffle model, launch files, parameter
  files and command line tools (`world_setup`, `world_rules`, `dynamic_obstacles`,
  `benchmark`, `visibility_map`, `tf_relay`).
* `client_node` — interactive and scriptable `GoToBlock` client.
* Randomised arena generation from a seed via `m8_demo.launch.py`: landmark placement inside
  each quadrant, two roaming obstacles whose walks are drawn and validated per seed, and the
  generated world, walks and `scene.json` written outside the repository.
* Unit tests for the pure layers: EKF core, coordinate transforms, colour thresholds, the
  goal FSM and the collision ledger.
* `scripts/install_deps.sh`, an idempotent dependency installer with a `--check` mode and
  shellcheck-clean sourcing of the ROS 2 environment.
* `docs/INSTALL.md` — supported platform, quick start, manual `apt` route and
  troubleshooting. `docs/BUILD.md` covers the build itself, what each package produces and
  how to rebuild only what changed. `docs/WALKTHROUGH.md` is the whole sequence once, from
  `git clone` to a completed `GoToBlock` goal, with a verification check at each of its nine
  steps, a single-block speedrun and a symptom-to-section table.
* Simplified Chinese editions of the four main documents — `README.zh-CN.md`,
  `docs/INSTALL.zh-CN.md`, `docs/BUILD.zh-CN.md` and `docs/WALKTHROUGH.zh-CN.md`. They mirror
  the English pages section by section, and every page in either language opens with a
  language switcher that links to its counterpart. Commands, package names, topics,
  parameters and paths stay in ASCII on both sides.
* `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, `CITATION.cff`,
  `THIRD_PARTY_NOTICES.md` and this changelog.
* GitHub issue and pull request templates; the issue chooser points at the discussion and
  security routes.
* Continuous integration on ROS 2 Humble: a flake8 job, a shellcheck job and a build-and-test
  job, plus `.github/scripts/check_manifests.py`, which validates the package metadata, every
  YAML and XML asset and the line endings of the tracked text files.
* `.gitattributes`, `.editorconfig`, `.flake8` and `requirements-dev.txt` for consistent line
  endings, indentation and lint settings. The code base is clean under the recorded flake8
  configuration.
* Package metadata carries the maintainer, repository URL and bug tracker URL; the
  `scripts/install_deps.sh` documentation and the two files derived from Apache-2.0 sources
  are labelled as such.

### Fixed

* The README and this changelog claimed `m8_demo.launch.py` could chain a `GoToBlock` goal
  automatically. It does not: it randomises the arena, starts the stack and animates the two
  obstacles, leaving the goal to the client.
* `ros2 run sim_bringup benchmark` and `ros2 run client_node client red` do not work as
  written. The six `sim_bringup` tools keep their `.py` suffix (`benchmark.py`), and the
  client takes `--color red`; a bare positional argument is ignored and `yellow` is sent.

[Unreleased]: https://github.com/Zhuqiben/beacon/compare/v0.0.1...HEAD
[0.0.1]: https://github.com/Zhuqiben/beacon/releases/tag/v0.0.1
