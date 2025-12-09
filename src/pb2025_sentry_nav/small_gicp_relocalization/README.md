# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/LihanChen2004/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/LihanChen2004/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=release
    ```

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```


```markdown
# small_gicp_relocalization

[![许可证](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![构建状态](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/LihanChen2004/small_gicp_relocalization/actions/workflows/ci.yml)

一个简单示例：使用 [small_gicp](https://github.com/koide3/small_gicp.git) 实现点云配准和定位

给定一个已配准的点云（基于里程计坐标系）和先验点云（使用 [pointlio](https://github.com/LihanChen2004/Point-LIO) 或类似工具构建的地图），该节点将计算两个点云之间的变换，并发布从 `map` 坐标系到 `odom` 坐标系的校正变换。

## 依赖项

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## 构建

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/LihanChen2004/small_gicp_relocalization.git

cd ..
```

1. 安装依赖项

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. 构建

    ```zsh
    colcon build --symlink-install -DCMAKE_BUILD_TYPE=release
    ```

## 使用方法

1. 在 [启动文件](launch/small_gicp_relocalization_launch.py) 中设置先验点云文件路径

2. 调整 `base_frame` 和 `lidar_frame` 之间的变换关系

    `pointlio` 和 `fastlio` 等算法输出的 `global_pcd_map` 严格基于 `lidar_odom` 坐标系。然而，机器人的初始位置通常由 `odom` 坐标系中的 `base_link` 坐标系定义。为了解决这种差异，代码会监听从 `base_frame`（速度参考坐标系）到 `lidar_frame` 的坐标变换，从而将 `global_pcd_map` 转换到 `odom` 坐标系中。

    如果未设置，将使用空变换。

3. 运行

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```
```