<p align="center">
  <a href="INSTALL.md">English</a> | <a href="INSTALL.zh-CN.md">简体中文</a>
</p>

# 安装 BEACON

BEACON 是纯仿真的。机器人、RGB-D 相机、传感器噪声和移动障碍物都在 Gazebo 中，因此不需要硬件，
不需要额外驱动，也不需要网络配置。

## 支持的平台

| 组件 | 开发所用版本 |
|---|---|
| Ubuntu | 22.04.4 LTS（Jammy），x86_64 |
| ROS 2 | Humble |
| Gazebo | Classic 11.10 |
| Python | 3.10 |

其他 Ubuntu 版本和其他 ROS 2 发行版未经测试：启动文件、参数文件以及 `scripts/install_deps.sh`
中的 ROS 软件包名都按 Humble 编写。

## 快速开始

```bash
git clone https://github.com/Zhuqiben/beacon.git ~/ros2_ws
cd ~/ros2_ws
./scripts/install_deps.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`install_deps.sh` 会安装下面列出的 apt 软件包、在缺少 ROS 2 时先添加 ROS 2 apt 软件源、对
`src/` 运行 `rosdep`，最后检查 Gazebo 以及它所依赖的 ROS 软件包是否确实可用。它是幂等的，重复
运行不会破坏一台正常工作的机器。

它安装的是系统内容，而不是当前 shell，所以这里在执行 `colcon build` 之前仍然需要
`source /opt/ros/humble/setup.bash`。每开一个新终端都要再来一次。

若只想看缺什么而不改动机器：

```bash
./scripts/install_deps.sh --check
```

## 软件包

整套由脚本安装。这里重复列出，便于审阅或手动安装。

| 分组 | 软件包 |
|---|---|
| ROS 2 基础 | `ros-humble-desktop` |
| 仿真器 | `gazebo`、`ros-humble-gazebo-ros-pkgs` |
| 机器人模型 | `ros-humble-turtlebot3-description`、`ros-humble-turtlebot3-gazebo`、`ros-humble-xacro`、`ros-humble-robot-state-publisher`、`ros-humble-joint-state-publisher`、`ros-humble-joint-state-publisher-gui` |
| 导航 | `ros-humble-nav2-bringup`、`ros-humble-nav2-collision-monitor` |
| Python 运行时 | `python3-numpy`、`python3-scipy`、`python3-opencv` |
| 工具链 | `python3-colcon-common-extensions`、`python3-rosdep2`、`python3-pip`、`python3-pytest`、`build-essential`、`cmake`、`git` |
| 软件源引导 | `curl`、`gnupg`、`lsb-release`、`software-properties-common` |

`ros-humble-desktop` 已经带来 `rclpy`、`ros2cli`、`rviz2`、`tf2`、`cv_bridge`、
`message_filters`、标准消息包和 `ros2launch`。`gazebo-ros-pkgs` 带来 `gazebo_ros`、
`gazebo_plugins` 和 `gazebo_msgs`；`nav2-bringup` 带来控制器、规划器、平滑器、行为、BT 导航器、
航点跟随、速度平滑和生命周期管理各个服务器。`nav2-collision-monitor` 单独列出，因为
`nav2-bringup` 并不依赖它，而导航启动文件需要它。

项目没有使用任何自有的 PyPI 包，因此没有 `requirements.txt`，也不需要虚拟环境。所有依赖要么是
ROS 2 软件包，要么是发行版软件包。

## 手动安装

如果机器上还没有 ROS 2，先添加软件源（只需一次）：

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

然后安装工作区依赖：

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

`rosdep` 来自 `python3-rosdep2`，Ubuntu 在 `universe` 中提供它。ROS 软件源另有一个更新的
`python3-rosdep`，但这两个包互相声明了 `Conflicts`，只能装其中一个。

`rosdep` 是对 `package.xml` 声明的权威交叉检查，即使已经按上面的列表装好，也值得跑一遍：

```bash
sudo rosdep init                                          # 仅首次需要
rosdep update
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

真正让它可用的是 `--ignore-src`。九个 BEACON 软件包彼此按名字依赖（`sim_bringup` 依赖其余八个），
而这些名字并不在公开的 rosdistro 中；该选项让 `rosdep` 把在 `src/` 内找到的依赖视为已满足，而
不是去查询它。请保留这个选项，运行的结尾应当是
`All required rosdeps installed successfully`。

## 验证安装

```bash
source /opt/ros/humble/setup.bash
gazebo --version                    # Gazebo 11.x
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix nav2_collision_monitor
```

然后构建并测试工作区：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src -q
```

构建成功会报告 `9 packages finished`。测试用例覆盖纯逻辑层，无需运行中的 ROS 图或 Gazebo。

如果构建表现异常，或者想知道哪些改动需要重新构建、哪些不需要，请阅读
[BUILD.zh-CN.md](BUILD.zh-CN.md)，而不是靠猜参数。

最后，带 Gazebo 图形界面启动仿真：

```bash
ros2 launch sim_bringup demo.launch.py
```

或不带界面，容器与 CI 任务应当采用这种方式：

```bash
ros2 launch sim_bringup demo.launch.py gui:=false
```

## 故障排查

构建本身的失败——找不到 `colcon`、符号链接错误、生成的接口过期、`install/` 归 root 所有、
资源未安装——见 [BUILD.zh-CN.md](BUILD.zh-CN.md#构建问题)。下面说的是安装与运行时问题。

**Gazebo 报没有可用的 OpenGL 上下文，或图形窗口是空白的。**
Gazebo Classic 需要 OpenGL 3.0 或更高。在虚拟机中请在宿主管理器设置里启用 3D 加速；如果客户机
没有 GPU 驱动，可以退回软件渲染：
`LIBGL_ALWAYS_SOFTWARE=1 ros2 launch sim_bringup demo.launch.py`。无界面运行则传
`gui:=false`。

**`colcon build` 在创建符号链接时失败。**
`--symlink-install` 需要支持符号链接的文件系统。请把工作区放在原生 Linux 文件系统上，例如
`~/ros2_ws`，不要放在共享目录或 Windows 挂载路径中。

**构建找不到 `slam_interfaces` 的消息。**
构建后必须加载 `source install/setup.bash`。新开的终端要再执行一次，并与
`source /opt/ros/humble/setup.bash` 一起。

**测试能导入软件包，但在收集阶段失败。**
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 会把第三方 pytest 插件挡在本次运行之外。先加载
`install/setup.bash` 才能让工作区的软件包可导入。

**`ModuleNotFoundError: No module named 'cv2'`。**
安装 `python3-opencv`。不要从 PyPI 安装 `opencv-python`：`cv_bridge` 是针对发行版的 OpenCV
编译的。

**找不到 `xacro`，或机器人始终不出现。**
安装 `ros-humble-xacro`。Waffle 的描述以未处理的 xacro 文件发布，启动文件会先让 `xacro`
处理它，再交给 `robot_state_publisher`。

**Nav2 找不到某个资源，例如某个行为插件。**
`ros-humble-nav2-bringup` 与 `ros-humble-nav2-collision-monitor` 都需要；导航启动文件是 Nav2
自带启动文件的内嵌副本，只是把碰撞监控插进了链路。

**不同机器或不同 shell 会话中的节点互相看不见。**
把 `ROS_DOMAIN_ID` 在各处设成同一个值，例如 `export ROS_DOMAIN_ID=42`。默认域与网络上所有其他
ROS 2 用户共用。

**`rosdep` 报 `ERROR: unable to process source ...`。**
rosdep 数据库过期或从未更新：运行 `rosdep update`。如果完全没有软件源，先运行
`sudo rosdep init`。`install_deps.sh` 把这两步都当作尽力而为，只给出警告，因为真正必须存在的是
显式的 apt 列表；之后可以用 `./scripts/install_deps.sh --check` 得到真实状态。

**apt 为了安装 `python3-rosdep` 要卸载 `python3-rosdep2`。**
两个提供者冲突。除非有明确理由换到 ROS 软件源中更新的包，否则保留 Ubuntu `universe` 提供的
`python3-rosdep2`。

## 相关文档

* [WALKTHROUGH.zh-CN.md](WALKTHROUGH.zh-CN.md) —— 把整条路径走一遍，从 clone 到目标完成。
* [BUILD.zh-CN.md](BUILD.zh-CN.md) —— 构建细节与构建故障排查。
* [README.zh-CN.md](../README.zh-CN.md) —— 系统做什么、如何运行、接口一览。
