<p align="center">
  <a href="README.md">English</a> | <a href="README.zh-CN.md">简体中文</a>
</p>

# Measurement data, figures and animations

This folder holds the measurement material behind the numbers quoted in the main
README: raw logs from real simulation runs, the figures built from those logs, and
three animations of complete missions.

Everything here was produced by running this repository unmodified at tag
`v0.0.1` (commit `afa6277`) on ROS 2 Humble with Gazebo Classic, inside a 4 vCPU /
7 GB Ubuntu 22.04 virtual machine.

| Folder | Contents |
| --- | --- |
| [`data/`](data) | `results.csv` (one row per mission) and the raw logs of the reference run |
| [`figures/`](figures) | Static figures built from the logs |
| [`gifs/`](gifs) | Animations of a complete mission |

## How the data was recorded

A read-only ROS 2 node subscribed to the running system. No package source was
changed for this: the node only listens and writes CSV files.

| Topic | Used for |
| --- | --- |
| `/robot_pose` | EKF-SLAM pose estimate |
| `/gazebo/model_states` | ground truth pose of the robot |
| `/landmark_map` | landmark estimates with their EKF sigma and validity flag |
| `/observations` | per-frame landmark detections |
| `/performance_monitor` | per-stage processing latency (detector, EKF-SLAM, planner, pose bridge) |
| `/observations/annotated` | camera frames with the detection boxes drawn, sampled at ~2.8 Hz |

Each run is launched with the repository's own entry points:

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=<seed> gui:=true
ros2 run client_node client --color <colour> --timeout 280
```

`gui:=true` is required in this environment: Gazebo renders the robot's camera
through the X display, so a headless `gzserver` produces no images and the
detector legitimately never fires.

Mission outcome (`success`, `final_distance_m`, `duration_s`) is the figure the
client node itself reports, not a number computed after the fact.

## Results

Eight seeded missions over five world seeds (`1490`, `3141`, `777`, `42`, `7`)
and all four goal colours. Full precision in [`data/results.csv`](data/results.csv).

| Run | Goal | Success | Duration | Final distance | Path length | Observations | Localisation RMS | Map error (mean) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `s1490_red` | red | yes | 36.3 s | 0.554 m | 3.06 m | 636 | 2.7 cm | 9.5 cm |
| `s1490_green` | green | yes | 16.9 s | 0.550 m | 2.87 m | 265 | 7.2 cm | 6.7 cm |
| `s1490_blue` | blue | yes | 26.8 s | 0.547 m | 3.10 m | 434 | 3.6 cm | 8.7 cm |
| `s1490_yellow` | yellow | yes | 19.3 s | 0.550 m | 2.55 m | 275 | 4.5 cm | 6.8 cm |
| `s3141_red` | red | yes | 30.4 s | 0.545 m | 2.89 m | 577 | 7.8 cm | 7.2 cm |
| `s42_yellow` | yellow | yes | 15.3 s | 0.550 m | 1.61 m | 274 | 6.2 cm | 3.4 cm |
| `s777_green` | green | yes | 17.6 s | 0.552 m | 2.68 m | 427 | 2.6 cm | 11.5 cm |
| `s7_blue` | blue | yes | 24.3 s | 0.549 m | 2.41 m | 540 | 2.5 cm | 11.2 cm |
| **mean** | | **8/8** | **23.4 s** | **0.550 m** | **2.65 m** | **428** | **4.6 cm** | **8.1 cm** |

`final_distance` sits just under the 0.60 m approach offset by construction: the
client stops as soon as it is inside the approach radius, so the number measures
how well the estimated landmark position matches reality rather than the driving.

## Figures

### `fig_trajectory.png`

Top-down view of the reference run: the grey band is the Gazebo ground-truth
path, the blue line is the EKF-SLAM estimate, squares are the landmark ground
truth positions and the dashed circle is the 0.60 m approach offset around the
goal.

### `fig_localisation_error.png`

Position error against ground truth over the whole run (top) and heading error
(bottom). Both stay inside a few centimetres / a couple of degrees; the residual
does not drift, because the map is re-observed rather than dead reckoned.

### `fig_landmark_map.png`

Landmark estimate error (solid) against the EKF sigma (dotted) for each colour.
The two curves converge, which is the property that matters: the filter's own
uncertainty is not a fiction.

### `fig_latency.png`

Per-stage processing latency. The detector dominates (tens of milliseconds, one
full 640x480 frame per sample); EKF-SLAM, the planner and the pose bridge stay in
the low single-digit milliseconds. The planner reports 0 ms whenever a plan is
reused instead of recomputed, which is why some runs show an empty planner box.

### `fig_observations.png`

Cumulative landmark observations and their instantaneous rate for the reference
run: 636 observations in 36 s, about 14 per second while driving.

### `fig_runs_overview.png` and `fig_accuracy_summary.png`

The eight runs side by side: duration and final distance on the left figure,
localisation error and landmark map accuracy on the right one.

## Animations

### `gifs/gif_mission_red.gif` (1.2 MB)

The complete reference mission, seed `1490`, goal `red`. Left: the robot's own
camera with the detector's boxes. Right: the map being built, the trajectory
followed and the current heading. Frames were taken from
`/observations/annotated` at about 2.8 Hz and are played at 8 fps, so the
animation runs roughly three times real time. The idle stretch before the robot
starts turning is cut, so both panels move from the first frame.

### `gifs/gif_mission_blue.gif` (1.0 MB)

The same view for seed `7` with goal `blue`.

### `gifs/gif_mission_four_goals.gif` (0.9 MB)

Four goals driven back to back in one session, from one launch and one map, in
the order red, blue, yellow, green. Left: a camera spawned 8 m above the world,
looking straight down. Right: the map and the trajectory the filter thinks it
drove. The overhead frames come from `/overhead/overhead/image_raw` at roughly
0.6 Hz and are played at 8 fps; the stretches where the robot is parked between
goals are thinned out, so the clock in the map title jumps rather than stalling.
The layout of the two panels is the same, which is the point: the left one is
the world as it is, the right one is the world as the filter believes it.

| Goal | Time | Path | Final distance | RMS error |
|---|---|---|---|---|
| red | 34.5 s | 3.14 m | 0.550 m | 3.6 cm |
| blue | 34.9 s | 4.83 m | 0.552 m | 5.2 cm |
| yellow | 30.8 s | 4.73 m | 0.550 m | 7.8 cm |
| green | 103.4 s | 4.11 m | 0.590 m | 6.2 cm |

Green takes three times as long as the other three even though it is not the
longest leg: for 75 % of it the estimated position does not move at all, while
the state machine stays in `NAVIGATING`. It is the only leg that has to cross
the middle of the arena, where the two moving obstacles are, and it never
touched a block.

### `gifs/gif_navigation.gif` (0.7 MB)

Map-only animation of the reference run: landmarks appear as they are observed,
with their sigma drawn as a dashed circle, while the EKF trajectory is drawn on
top of the ground-truth path.

## Reproducing

```bash
# 1. build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. run one mission (GUI required for the camera to render)
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490 gui:=true &
ros2 run client_node client --color red --timeout 280

#    or drive several goals in a row against the same launch and the same map
for c in red blue yellow green; do ros2 run client_node client --color $c --timeout 170; done

# 3. record while it runs
ros2 topic echo /robot_pose --once          # sanity check
#    then subscribe to the topics listed above and write CSV
```

Raw column layout of the reference run, in [`data/raw_s1490_red/`](data/raw_s1490_red):

| File | Columns |
| --- | --- |
| `pose.csv` | `t, ekf_x, ekf_y, ekf_yaw, gt_x, gt_y, gt_yaw, err, yaw_err` |
| `map.csv` | `t, color, x, y, sigma, valid` |
| `latency.csv` | `t, stage, ms` (stage 0 detector, 1 EKF-SLAM, 2 planner, 3 pose bridge) |
| `observations.csv` | `t, count, colors` |
| `frames.csv` | `idx, t, file` — index of the camera frames used for the animations |
| `metrics.json` | summary of the run, the same fields as one row of `results.csv` |

## Notes and caveats

- World seed `2024` is not part of the set. Scene generation does not terminate
  for that seed: `world_rules.py` spins inside `_ray_novelty` while building the
  dynamic-obstacle tracks, so `world_setup.py` never returns and the launch file
  never brings the nodes up. Every other seed tried here generated normally.
- The runs were recorded on a virtual machine with software OpenGL, so the
  absolute latency numbers are pessimistic compared with a machine with a GPU.
  They are still a fair relative comparison between the stages.
- The animations come from the robot's camera topic rather than a desktop
  recording, so they show exactly what the detector saw. The exception is
  `gif_mission_four_goals.gif`, whose left panel is a plain Gazebo camera model
  spawned above the arena: `libgazebo_ros_camera` pointed straight down at
  `z = 8`, `FOV 1.05`, 720×720 at 5 Hz.
