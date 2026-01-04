# AGV（自动导航车）仓库

项目包含用于仿真、描述、MoveIt 配置与控制器的 ROS 2 工作区，用于在仿真与真实机器人上部署 AGV（自动导航车）功能。

**主要目标**：提供一个可构建的 ROS 2 工作区，包含机器人模型（URDF/XACRO）、MoveIt 配置、控制器示例（C++），并能在 Gazebo/RViz 中进行仿真与调试。

---

**目录结构（摘要）**
- `src/agv_bringup`：启动相关配置与 launch 文件（`launch/agv.launch.xml`）。
- `src/agv_description`：机器人模型（`urdf/`、`meshes/`）、展示与 Gazebo 启动（`launch/display.launch.xml`、`launch/gazebo.launch`）。
- `src/agv_moveit_config`：MoveIt 配置（SRDF、规划参数、控制器映射、move_group 启动器与 RViz 配置）。
- `src/Controller_For_Agv`：自定义控制器 C++ 实现（源代码在 `src/`，可通过 colcon 构建）。

安装/构建产物位于 `install/`，构建中间文件位于 `build/`，日志在 `log/`。

---

**环境与先决条件**
- 已安装 ROS 2（例如 Humble / Iron / 你当前使用的发行版）。
- `colcon` 构建工具（`colcon-common-extensions` 推荐）。
- Gazebo（如需要仿真）和 MoveIt2 依赖（根据发行版安装）。

示例依赖安装（请根据你的 ROS 2 发行版调整）：

```bash
# Ubuntu + ROS2 示例（替换 <ros2-distro>）
sudo apt update
sudo apt install -y python3-colcon-common-extensions \
	ros-<ros2-distro>-desktop ros-<ros2-distro>-moveit
```

---

**构建工作区**
在仓库根目录下运行：

```bash
# 1. 来源 ROS 2 环境（替换为你的发行版）
source /opt/ros/<ros2-distro>/setup.bash

# 2. 构建（并行可选）
colcon build --parallel-workers 4

# 3. 来源叠加环境
source install/setup.bash
```

注意：若你使用 `bash` 以外的 shell，请相应使用 `setup.zsh` 或 `setup.ps1`（Windows PowerShell）。

---

**常用启动示例**
在完成 `source install/setup.bash` 后，可运行以下示例：

- 启动 AGV bringup（启动基础节点/配置）:

```bash
ros2 launch agv_bringup agv.launch.xml
```

- 在仿真中启动机器人（Gazebo）:

```bash
ros2 launch agv_description gazebo.launch
```

- 启动 MoveIt 演示（Move Group + RViz）:

```bash
ros2 launch agv_moveit_config demo.launch.py
```

- 单独启动 MoveIt 的 `move_group`:

```bash
ros2 launch agv_moveit_config move_group.launch.py
```

- 启动/加载控制器（如果需要）:

```bash
ros2 launch agv_moveit_config spawn_controllers.launch.py
```

- 在 RViz 中显示模型与配置（若使用 XML 启动）:

```bash
ros2 launch agv_description display.launch.xml
```

如果你需要查看可用的 launch 文件，查看对应包下的 `launch/` 目录。

---

**关于 `Controller_For_Agv`（自定义控制器）**
- 该包位于 `src/Controller_For_Agv`，包含 C++ 源文件（`src/command_template.cpp`、`src/main.cpp`）与 `CMakeLists.txt`。
- 使用 `colcon build` 构建时会被编译并安装到 `install/` 下。可通过运行类似下面命令启动可执行文件（可执行名以构建输出为准）：

```bash
# 举例：替换 <executable_name> 为实际生成的可执行文件名
ros2 run Controller_For_Agv <executable_name>
```

提示：若不确定可执行文件名，构建后检查 `install/` 下 `lib/Controller_For_Agv/` 目录或查看 `CMakeLists.txt` 中 `add_executable` 指定的名字。

---

**开发与调试提示**
- 在开发前常做的步骤：

```bash
source /opt/ros/<ros2-distro>/setup.bash
colcon build --packages-select Controller_For_Agv -c
source install/setup.bash
```

- 使用 `ros2 topic list`、`ros2 node list`、`ros2 param list` 等命令排查运行时状态。
- 查看控制器/启动器输出使用：`ros2 run` 或 `ros2 launch` 的终端日志，或通过 `ros2 topic echo` 观察话题数据。

---

**常见问题**
- 若找不到依赖：确认已安装对应 ROS 包（MoveIt、控制器驱动等），并且 `source /opt/ros/<ros2-distro>/setup.bash` 已执行。
- 构建报错：先尝试清理 `build/` 与 `install/`（或使用 `colcon build --packages-select <pkg> --cmake-clean-cache`），确认 CMake 错误信息并补齐依赖。

---

**贡献与许可证**
- 欢迎在本仓库基础上改进模型、控制器或 MoveIt 配置。提交 PR 前请保证：代码通过本地构建、关键节点能启动、并带有简短说明文档或注释。
- 本仓库未在此处声明许可证；如需发布或共享，请在仓库根目录添加 `LICENSE` 文件并在 `package.xml` 中适当声明许可项。

---

