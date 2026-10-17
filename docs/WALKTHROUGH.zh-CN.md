<p align="center">
  <a href="WALKTHROUGH.md">English</a> | <a href="WALKTHROUGH.zh-CN.md">简体中文</a>
</p>

# 从 clone 到目标完成

一条走到底的 BEACON 路径：取代码、装依赖、构建、运行、下发目标，然后确认它确实成功了。每一步都以
一个可验证的结果收尾，于是失败会在发生的地方被抓住，而不是三步之后才发现。

另外两份文档各自把一步讲深——[INSTALL.zh-CN.md](INSTALL.zh-CN.md) 讲系统侧，
[BUILD.zh-CN.md](BUILD.zh-CN.md) 讲构建。本文讲的是顺序。第 1 到第 4 步只需做一次；第 5 到第 8
步就是一次正常使用的样子。

## 你需要什么

| | |
|---|---|
| 操作系统 | Ubuntu 22.04（Jammy），x86_64 |
| 磁盘 | 约 6 GB 可用空间：ROS 2 desktop、Gazebo，以及 `build/` 与 `install/` |
| 图形 | 供 Gazebo 窗口使用的可用 OpenGL 3.0 上下文；软件渲染可行但很慢 |
| 网络 | 只用于 clone 与 `apt` 安装 |

不需要机器人、不需要相机、也不需要第二台机器。一切都在同一台主机上仿真。

## 1. 取代码

```bash
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
```

**检查：** `ls src` 列出九个目录——`client_node`、`ekf_slam`、`goal_selector`、
`landmark_detector`、`landmark_map`、`performance_monitor`、`sim_bringup`、
`slam_interfaces`、`visualizer`。

把检出放在如上所说的原生 Linux 文件系统上。VirtualBox 共享目录或 NTFS 挂载无法承载第 3 步创建的
符号链接。

## 2. 安装系统依赖

```bash
./scripts/install_deps.sh
```

若缺少 ROS 2，它会添加 ROS 2 apt 软件源、安装 `apt` 软件包、对 `src/` 运行 `rosdep`，最后报告仍然
无法解析的项。它是幂等的，安装部分需要 `sudo`。首次运行大约几分钟。

只想查看而不动机器：

```bash
./scripts/install_deps.sh --check
```

完整软件包列表，以及脚本无法使用时的机器该走的手动 `apt` 路线，见
[INSTALL.zh-CN.md](INSTALL.zh-CN.md)。

**检查**以下四条，按这个顺序——每一条抓的是不同类别的缺失软件包：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix nav2_collision_monitor
ros2 pkg prefix turtlebot3_gazebo
gazebo --version
python3 -c 'import numpy, scipy, cv2'
```

`gazebo --version` 必须打印 `Gazebo multi-robot simulator, version 11.x`。如果连 `ros2` 都
找不到，说明 ROS 2 安装没有完成。

脚本安装的是机器上的内容，不是你的 shell，所以这里以及每个新终端仍然需要
`source /opt/ros/humble/setup.bash`。

## 3. 构建工作区

```bash
source /opt/ros/humble/setup.bash          # 上面已经执行过，重复一次无害
colcon build --symlink-install
source install/setup.bash
```

在 `~/ros2_ws`——也就是包含 `src/` 的目录——执行。首次构建大约一分钟；只有 `slam_interfaces`
的消息生成会真正编译。

**检查：** 最后一行是

```
Summary: 9 packages finished [1min 12s]
```

任何其他数字都说明有软件包被跳过——`colcon list` 会显示它找到了什么。

`source install/setup.bash` 不是可选项：在它执行之前，`ros2` 看不到这九个软件包中的任何一个。
每个新终端都需要这两条 `source` 命令，顺序如上。

## 4. 运行前验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon list
ros2 pkg executables sim_bringup
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q
```

**检查：** `colcon list` 打印九个软件包；`ros2 pkg executables sim_bringup` 打印六个工具，每个
都带 `.py` 后缀——`benchmark.py`、`dynamic_obstacles.py`、`tf_relay.py`、`visibility_map.py`、
`world_rules.py`、`world_setup.py`；pytest 以一行 `N passed` 结尾且没有失败。

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 是必需的。`colcon build` 会装上 `launch_testing` 与
`ament_*` 的 pytest 插件，它们被载入一次普通运行时会让收集阶段中止。

如果 pytest 无法导入 `ekf_slam` 或 `goal_selector`，说明那个 shell 没有加载
`install/setup.bash`。

## 5. 首次运行：固定场地

终端 A：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sim_bringup demo.launch.py
```

**你应当看到：** 一个 Gazebo 窗口，四面围墙的场地、靠近墙边的四个彩色方块，以及场地中央的一台
TurtleBot 3 Waffle。终端会滚动输出各节点启动的信息：`tf_relay`、`robot_state_publisher`、
`spawn_entity`、`detector_node`、生命周期自启动、`slam_node`、`map_node`、`action_server`、
`viz_node`、`monitor_node`。

给它大约十秒。检测器必须先走完自己的生命周期转换才会发布任何内容，而 SLAM 滤波器需要机器人先看到
一个方块，地图才有意义。

**在第三个终端检查：**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 node list
ros2 topic hz /observations
```

`ros2 node list` 应当显示启动文件中的那些节点，`/observations` 应当在相机看到方块时每秒发布若干次。
没有 `/observations` 意味着检测器从未进入 `active`。

要停止，在终端 A 按 Ctrl-C 并等待 Gazebo 关闭。请用 Ctrl-C 关闭 Gazebo，而不是窗口的关闭按钮——
启动系统需要这个信号来把其余部分一起收掉。

## 6. 下发目标

整套栈已经跑起来，但在被要求某个颜色之前什么也不做。终端 B：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run client_node client --color red
```

`--color` 接受 `red`、`green`、`blue` 或 `yellow`，默认是 `yellow`。

客户端每秒打印一次反馈：

```
[fb   1] SEARCHING    target_known=False dist= -1.00 m elapsed=  0.10 s
[fb   8] SEARCHING    target_known=True  dist=  3.42 m elapsed=  7.90 s
[fb  11] NAVIGATING   target_known=True  dist=  3.10 m elapsed= 10.90 s
[fb  34] APPROACHING  target_known=True  dist=  0.71 m elapsed= 33.90 s
```

然后是结果：

```
success=True  final_distance=0.598 m
```

**检查：** `success=True`，并且 `final_distance` 接近 `0.6 m`。这说明整套系统端到端工作了——
检测、SLAM、建图、规划、控制。

常用参数：`--approach 0.8` 换取更大的停靠间距，`--timeout 90` 换取更长的截止时间，
`--cancel-after 5` 用来走一遍中止路径。

## 7. 随机场地与移动障碍物

`demo.launch.py` 总是加载同一个世界，这在把系统跑起来时正是你想要的。研究用的配置则每次启动都生成
一个全新场地，并在其中放两个游走的立方体：

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490
```

种子驱动地标摆放与两个方块的行走路径，因此 `world_seed:=1490` 永远产出同一个场地。不写
`world_seed` 时会随机抽一个并打印出来——把它传回去即可复现那次会话。

在 SLAM 地图上叠加 RViz，或干脆去掉 Gazebo 窗口：

```bash
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490 rviz:=true
ros2 launch sim_bringup m8_demo.launch.py world_seed:=1490 gui:=false
```

生成的世界文件、方块行走路径与 `scene.json` 写在仓库之外的 `~/.ros/sim_bringup_scene/`。随后按
第 6 步下发目标。

## 8. 度量结果

`benchmark.py` 用 Gazebo 真值给跑着的栈打分。栈仍在运行时，在第三个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run sim_bringup benchmark.py --duration 60 --csv perf.csv
```

它会报告相对 `/gazebo/model_states` 的定位 RMS、相对 `scene.json` 中真实地标位姿的地图误差、
观测频率以及各阶段延迟。`--duration 0` 则一直运行到 Ctrl-C。若要对更早生成的世界评测，用
`--scene` 指向另一个 `scene.json`。

同样的数字会实时发布在 `/performance_monitor`，而 `monitor_node` 会把摘要写到
`/performance_report`。

## 9. 关闭

先在客户端按 Ctrl-C，再在启动终端按 Ctrl-C。然后确认没有残留：

```bash
ros2 node list
```

输出为空说明关闭是干净的。残留节点通常是活过启动流程的 Gazebo 进程；`pkill -f gazebo` 可以清掉。

## 全部流程，一段搞定

```bash
# 只需一次
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
./scripts/install_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q

# 每次会话，终端 A
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sim_bringup demo.launch.py

# 每次会话，终端 B
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run client_node client --color red
```

## 某一步失败时

| 步骤 | 现象 | 去哪里看 |
|---|---|---|
| 1 | clone 失败，或 `src` 为空 | 网络或 URL；仓库是 `Zhuqiben/beacon` |
| 2 | `apt` 找不到 `ros-humble-*` | 缺少 ROS 2 apt 软件源——手动路线见 [INSTALL.zh-CN.md](INSTALL.zh-CN.md#手动安装) |
| 2 | `rosdep` 说九个软件包名未知 | 命令里漏掉了 `--ignore-src` |
| 3 | `colcon: command not found` | `python3-colcon-common-extensions`，并且先加载 ROS 2 |
| 3 | 符号链接错误 | 工作区不在原生 Linux 文件系统上 |
| 3 | 构建出的软件包少于 9 个 | `colcon list`；见 [BUILD.zh-CN.md](BUILD.zh-CN.md#构建问题) |
| 4 | pytest 无法导入某个包 | 没有加载 `install/setup.bash` |
| 4 | pytest 在收集阶段中止 | 缺少 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| 5 | Gazebo 窗口空白或全黑 | OpenGL——`LIBGL_ALWAYS_SOFTWARE=1`，或 `gui:=false` |
| 5 | `/observations` 没有输出 | 检测器未 `active`；检查启动日志中的生命周期错误 |
| 6 | `/goto_block not available` | 动作服务器没起来——`demo.launch.py` 还在运行吗？ |
| 6 | `success=False` 且超时 | `--timeout 90`，并确认颜色是那四种之一 |
| 8 | 地图误差看起来没有意义 | 那次运行没有 `scene.json`——显式传 `--scene` |

安装类问题——OpenGL、`cv2`、`xacro`、`rosdep`、Nav2 资源、`ROS_DOMAIN_ID`——见
[INSTALL.zh-CN.md](INSTALL.zh-CN.md#故障排查) 的故障排查一节。构建类问题见
[BUILD.zh-CN.md](BUILD.zh-CN.md#构建问题)。

## 相关文档

* [INSTALL.zh-CN.md](INSTALL.zh-CN.md) —— 平台、依赖、手动 `apt` 路线、运行时问题。
* [BUILD.zh-CN.md](BUILD.zh-CN.md) —— 构建细节、重建规则、构建故障排查。
* [README.zh-CN.md](../README.zh-CN.md) —— 系统做什么、接口、参数、工具。
* [`.github/workflows/ci.yml`](../.github/workflows/ci.yml) —— 同一套构建与测试序列，每次 push
  都会运行。
