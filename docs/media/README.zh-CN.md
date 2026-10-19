<p align="center">
  <a href="README.md">English</a> | <a href="README.zh-CN.md">简体中文</a>
</p>

# 实测数据、图表与动画

本目录存放主 README 中各项数字的实测依据：真实仿真运行产生的原始日志、由日志绘制的图表，以及完整任务的动画。

所有内容均来自**未改动源码**的本仓库 `v0.0.1`（提交 `afa6277`），运行环境为 ROS 2 Humble + Gazebo Classic，4 vCPU / 7 GB 的 Ubuntu 22.04 虚拟机。

| 目录 | 内容 |
| --- | --- |
| [`data/`](data) | `results.csv`（每次任务一行）与参考运行的原始日志 |
| [`figures/`](figures) | 由日志绘制的静态图表 |
| [`gifs/`](gifs) | 完整任务的动画 |

## 数据是如何采集的

采集端是一个只读的 ROS 2 节点，只订阅运行中的系统并写 CSV，不修改任何包源码。

| 话题 | 用途 |
| --- | --- |
| `/robot_pose` | EKF-SLAM 位姿估计 |
| `/gazebo/model_states` | 机器人真值位姿 |
| `/landmark_map` | 地标估计值、EKF sigma 与有效性标记 |
| `/observations` | 每帧的地标观测 |
| `/performance_monitor` | 各阶段耗时（检测、EKF-SLAM、规划、位姿桥接） |
| `/observations/annotated` | 叠加了检测框的相机画面，约 2.8 Hz 采样 |
| `/overhead/overhead/image_raw` | 俯视渲染画面，用作四目标动画的左栏 |
| `/obstacle_markers` | 机器人当前认为每个动态方块在哪（`gif_mission_dynamic.gif` 的右栏） |

每次运行都使用仓库自身的入口：

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=<seed> gui:=true
ros2 run client_node client --color <colour> --timeout 280
```

此环境下必须 `gui:=true`：Gazebo 通过 X 显示渲染机器人相机，无头的 `gzserver` 不会产生图像，检测器自然始终检测不到任何东西。

动态障碍那一次在第一段目标之前就把方块开起来，并一直开到结束：

```bash
ros2 service call /dynamic_obstacles/start slam_interfaces/srv/StartObstacles "{speed: 0.6}"
```

方块只有在机器人相机看到它时才会出现在地图里，因为这是系统唯一的位置来源。方块离开视野后，
标记会被撤销（`markers.csv` 中 `action: 2`），而不是冻结在最后一次位姿上。

任务结果（`success`、`final_distance_m`、`duration_s`）取自客户端节点自身的上报，不是事后推算的数字。

## 结果

八次带种子的任务，覆盖五个世界种子（`1490`、`3141`、`777`、`42`、`7`）与全部四种目标颜色。完整精度的数据见 [`data/results.csv`](data/results.csv)。

| 运行 | 目标 | 成功 | 耗时 | 末距 | 路径长度 | 观测数 | 定位 RMS | 地标图误差（均值） |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `s1490_red` | 红 | 是 | 36.3 s | 0.554 m | 3.06 m | 636 | 2.7 cm | 9.5 cm |
| `s1490_green` | 绿 | 是 | 16.9 s | 0.550 m | 2.87 m | 265 | 7.2 cm | 6.7 cm |
| `s1490_blue` | 蓝 | 是 | 26.8 s | 0.547 m | 3.10 m | 434 | 3.6 cm | 8.7 cm |
| `s1490_yellow` | 黄 | 是 | 19.3 s | 0.550 m | 2.55 m | 275 | 4.5 cm | 6.8 cm |
| `s3141_red` | 红 | 是 | 30.4 s | 0.545 m | 2.89 m | 577 | 7.8 cm | 7.2 cm |
| `s42_yellow` | 黄 | 是 | 15.3 s | 0.550 m | 1.61 m | 274 | 6.2 cm | 3.4 cm |
| `s777_green` | 绿 | 是 | 17.6 s | 0.552 m | 2.68 m | 427 | 2.6 cm | 11.5 cm |
| `s7_blue` | 蓝 | 是 | 24.3 s | 0.549 m | 2.41 m | 540 | 2.5 cm | 11.2 cm |
| **均值** | | **8/8** | **23.4 s** | **0.550 m** | **2.65 m** | **428** | **4.6 cm** | **8.1 cm** |

末距稳定落在 0.60 m 接近偏移之下一点，这是设计使然：客户端一旦进入接近半径即停车，因此这个数字衡量的是**地标位置估计与真值的吻合程度**，而不是行驶水平。

## 图表

### `fig_trajectory.png`

参考运行的俯视图：灰色粗线是 Gazebo 真值轨迹，蓝线是 EKF-SLAM 估计，方块是地标真值位置，虚线圆是目标周围 0.60 m 的接近偏移。

### `fig_localisation_error.png`

上图为全程位置误差（对真值），下图为航向误差。两者始终保持在数厘米、数度以内，且不随时间漂移——地图是反复观测出来的，不是航迹推算。

### `fig_landmark_map.png`

每种颜色的地标估计误差（实线）与 EKF sigma（虚线）。两条曲线收敛到同一量级，这正是关键性质：滤波器自报的不确定度不是编出来的。

### `fig_latency.png`

各阶段处理耗时。检测器占大头（数十毫秒，每个采样一次完整 640×480 帧）；EKF-SLAM、规划与位姿桥接都在个位数毫秒。规划在复用既有路径时上报 0 ms，因此部分运行的规划箱线为空。

### `fig_observations.png`

参考运行的累计地标观测数与瞬时速率：36 s 内 636 次观测，行驶中约每秒 14 次。

### `fig_runs_overview.png` 与 `fig_accuracy_summary.png`

八次运行的横向对比：左图给出耗时与末距，右图给出定位误差与地标图精度。

## 动画

### `gifs/gif_mission_red.gif`（1.2 MB）

参考任务全程，种子 `1490`，目标红色。左侧是机器人相机画面与检测框，右侧是地图构建过程、已走轨迹与当前朝向。画面取自 `/observations/annotated`（约 2.8 Hz），以 8 fps 播放，约为真实速度的三倍。机器人开始转动前的静止片段已剪掉，两侧画面从第一帧起都在变化。

### `gifs/gif_mission_blue.gif`（1.0 MB）

种子 `7`、目标蓝色的同款视角。

### `gifs/gif_mission_four_goals.gif`（0.9 MB）

一次启动、一张地图、连续跑完四个目标：红、蓝、黄、绿。左侧是挂在世界坐标上方 8 m、
垂直向下的相机，右侧是滤波器认为走过的地图与轨迹。俯视画面取自
`/overhead/overhead/image_raw`（约 0.6 Hz），以 8 fps 播放；两次目标之间车停着不动的
片段被抽稀，所以地图标题里的计时会跳变而不是卡住。两栏的取景完全一致，这正是这张图
的意义：左边是世界的真实样子，右边是滤波器心里的样子。

| 目标 | 耗时 | 路径 | 末距 | 定位 RMS |
|---|---|---|---|---|
| 红 | 34.5 s | 3.14 m | 0.550 m | 3.6 cm |
| 蓝 | 34.9 s | 4.83 m | 0.552 m | 5.2 cm |
| 黄 | 30.8 s | 4.73 m | 0.550 m | 7.8 cm |
| 绿 | 103.4 s | 4.11 m | 0.590 m | 6.2 cm |

绿色一段的耗时是另外三段的三倍，但它并不是最长的一段：其中 75% 的时间里估计位置完全没有
变化，而状态机始终停留在 `NAVIGATING`。只有这一段需要穿过场地中部、也就是两个动态障碍所在的
区域，而全程没有碰到色块。

### `gifs/gif_mission_dynamic.gif`（0.9 MB）

同样的四个目标、同样的顺序，但两个动态障碍在第一段开始前就已启动，并以 `0.6 m/s`
全程移动。这正是本项目想展示的场景：机器人必须一边躲避两个自己也在跑的方块，一边不断
重新规划。

两栏的含义同上，区别在于什么东西在动。左侧画面里白色与黑色方块逐帧平移，机器人身上
那个实线圆是真实位姿。右侧画面里，方块只有在机器人自己的相机报告它时才以实心方块出现，
并带一条最近观测的虚线尾迹；一旦离开视野，方块就消失，只剩尾迹。这是有意为之：地图没
有别的方块位置来源，画出机器人看不见的方块等于谎报系统所知。

| 目标 | 耗时 | 路径 | 末距 | 定位 RMS |
|---|---|---|---|---|
| 红 | 34.5 s | 3.13 m | 0.553 m | 2.7 cm |
| 蓝 | 30.7 s | 4.86 m | 0.549 m | 4.1 cm |
| 黄 | 26.5 s | 4.83 m | 0.550 m | 7.2 cm |
| 绿 | 107.2 s | 4.28 m | 0.594 m | 7.6 cm |

四段全部成功，全程没有碰到方块。绿色一段的表现与 `gif_mission_four_goals.gif` 完全
一致：耗时约为另外三段的三倍，且大部分时间里估计位置冻结、状态机停留在 `NAVIGATING`。
两次独立运行在同一段上复现同一个卡顿，说明问题出在场地中部的穿越，而不是那一次运行恰好
遇到的方块轨迹。

动画以 10 fps 播放，四段合起来约为真实速度的十倍。素材取自 1.8 Hz 的俯视录像，
两次目标之间的停顿同样被抽稀。

### `gifs/gif_navigation.gif`（0.7 MB）

仅地图视角的参考运行动画：地标被观测到后逐个出现，虚线圆为其 sigma，EKF 轨迹叠加在真值路径上。

## 复现方法

```bash
# 1. 构建
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. 跑一次任务（相机需要 GUI 才能渲染）
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490 gui:=true &
ros2 run client_node client --color red --timeout 280

#    或者连续跑多个目标，同一张地图、同一次启动
for c in red blue yellow green; do ros2 run client_node client --color $c --timeout 170; done

# 3. 运行期间记录
ros2 topic echo /robot_pose --once          # 先确认话题存在
#    然后订阅上文列出的话题并写入 CSV
```

参考运行的原始列定义见 [`data/raw_s1490_red/`](data/raw_s1490_red)：

| 文件 | 列 |
| --- | --- |
| `pose.csv` | `t, ekf_x, ekf_y, ekf_yaw, gt_x, gt_y, gt_yaw, err, yaw_err` |
| `map.csv` | `t, color, x, y, sigma, valid` |
| `latency.csv` | `t, stage, ms`（stage 0 检测、1 EKF-SLAM、2 规划、3 位姿桥接） |
| `observations.csv` | `t, count, colors` |
| `frames.csv` | `idx, t, file`——动画所用的相机帧索引 |
| `metrics.json` | 该次运行的汇总，字段与 `results.csv` 的一行相同 |

## 说明与限制

- 世界种子 `2024` 未纳入统计。该种子的场景生成不会终止：`world_rules.py` 在 `_ray_novelty` 中死循环，`world_setup.py` 永不返回，launch 因此起不来任何节点。其余尝试过的种子均正常。
- 数据采集于使用软件 OpenGL 的虚拟机，绝对耗时比有 GPU 的机器悲观。阶段之间的**相对**比较仍然有效。
- 动画取自机器人相机话题而非桌面录屏，因此展示的就是检测器实际看到的画面。例外是 `gif_mission_four_goals.gif` 与 `gif_mission_dynamic.gif`：它们的左栏是在场地上方生成的一个普通 Gazebo 相机模型——`libgazebo_ros_camera` 垂直向下，`z = 8`，`FOV 1.05`，720×720、5 Hz。
