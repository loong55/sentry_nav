# sdformat tools

`Sdformat` (`SDF`) is used by `Gazebo` simulator to describe simulated robots and simulated environment (sim world). As ROS users, we know `URDF` is more common to describe robot's geometry, kinematic and dynamic properties in ROS community, which is used by many ROS applications like `rviz`, `moveit`, `robot_state_publisher`.

this package provides some easy, convenient and flexible tools to help us make the best use of `SDF` in ROS community:

* `xmacro4sdf`: write modular `SDF` xml by using macro  (not nest model) , refer to [xmacro](https://github.com/gezp/xmacro).
* `sdf2urdf`: convert `SDF` to `URDF` with some limitation.
* `urdf_generator`: urdf utility class (support sdf2urdf, merge).

a workflow of using SDF in ROS community with these tools:

![](workflow.png)


## Usage

Installation

```bash
pip install xmacro
# cd src of ros workspace
git clone https://github.com/gezp/sdformat_tools.git
# cd ros workspace
colcon build
```

xmacro4sdf

```bash
xmacro4sdf model.sdf.xmacro
```

sdf2urdf

```bash
sdf2urdf model.sdf
```

urdf_generator

```python
from sdformat_tools.urdf_generator import UrdfGenerator

urdf_generator = UrdfGenerator()
urdf_generator.parse_from_sdf_file(robot_sdf_path)
urdf_generator.remove_joint('world_joint')
urdf_generator.extend_urdf_file(control_urdf_path)
robot_urdf_xml = urdf_generator.to_string()
```

### 意译
`Sdformat` (`SDF`) 是Gazebo模拟器用来描述模拟机器人和模拟环境（模拟世界）的格式。在ROS社区中，`URDF` 是描述机器人几何、运动学和动态属性的标准格式，被广泛应用于ROS应用程序，如 `rviz`、`moveit`、`robot_state_publisher`。
这个软件包提供了一系列工具，帮助我们在ROS社区中更有效地使用 `SDF`：
* `xmacro4sdf`: 使用宏创建模块化的 `SDF` xml 文件（不嵌套模型），基于 [xmacro](https://github.com/gezp/xmacro)。
* `sdf2urdf`: 将 `SDF` 文件转换为 `URDF` 文件，但存在一些限制。
* `urdf_generator`: 一个URDF实用类，支持从 `SDF` 文件生成 `URDF` 文件和合并 `URDF` 文件。
以下是使用这些工具在ROS社区中处理 `SDF` 文件的工作流程：
![](workflow.png)
## 使用方法
安装步骤：
1. 安装 `xmacro`:
   ```bash
   pip install xmacro
   ```
2. 克隆 `sdformat_tools` 到ROS工作区的 `src` 目录:
   ```bash
   cd ~/ros_ws/src
   git clone https://github.com/gezp/sdformat_tools.git
   ```
3. 构建工作区:
   ```bash
   cd ~/ros_ws
   colcon build
   ```
使用 `xmacro4sdf`:
```bash
xmacro4sdf model.sdf.xmacro
```
使用 `sdf2urdf`:
```bash
sdf2urdf model.sdf
```
使用 `urdf_generator`:
```python
from sdformat_tools.urdf_generator import UrdfGenerator
urdf_generator = UrdfGenerator()
urdf_generator.parse_from_sdf_file(robot_sdf_path)
urdf_generator.remove_joint('world_joint')
urdf_generator.extend_urdf_file(control_urdf_path)
robot_urdf_xml = urdf_generator.to_string()
```

--------------------------------
以上内容由AI生成，仅供参考和借鉴