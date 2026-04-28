# COD_RMUL2026_Navigation
## 项目简介

- **运行环境**
  - Ubuntu 22.04
  - ROS2 Humble
  - Livox MID-360
- 基于Nav2框架开发的导航功能包
- 控制器选用MPPI
- slam_toolbox同时导航建图
- 单点导航/多点导航
- 定位与地图处理
- 全局/局部路径规划
- 导航控制与状态切换

## 仓库结构
```bash
.
├── cod_bringup                     #Navigation2导航启动文件、机器人运动参数、地图存储、csv多点文件
├── cpp_lidar_filter                #剪裁去除机器人自身点云，spatio-temporal-voxel-layer没有半径滤波
├── fake_vel_transform              #TF转换
├── goal_approach_controller        #Nav2控制器wrapper：在接近目标时限制线速度，防止高速冲过目标点
├── pb_nav2_plugins                 #Navigation2插件库，控制机器人执行后退行为
├── pb_omni_pid_pursuit_controller  #PID控制器
├── pointcloud_to_laserscan         #点云转换，pointcloud->laserscan
├── ros2_simple_serial              #串口通信
├── small_point_lio                 #point_lio提供里程计，odom->base_link
└── waypoint_editor                 #多点航点编辑
```

## 使用说明
### 前置工作
- 安装[Livox SDK2] https://github.com/Livox-SDK/Livox-SDK2 
- 雷达启动包 https://github.com/Livox-SDK/livox_ros_driver2.git 

- 安装 `rosdep`  
   参考官方文档或使用如下命令进行安装：

   ```shell
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```
- 安装依赖
  ```shell
  mkdir ~/COD26
  git clone https://gitee.com/codnavgation/cod_-rm2026_-navigation.git
  cd cod_-rm2026_-navigation 
  rosdep install --from-paths src --ignore-src -r -y
  ```
- 安装 `spatio-temporal-voxel-layer`  ,根据时间清除动态点云
  ```shell
  sudo apt-get install -y ros-humble-spatio-temporal-voxel-layer
  ```
- 构建方式
  ```shell
  colcon build --symlink
  source install/setup.bash
  ```
- 运行方式（启动导航前先启动雷达）
  - 多点导航
  ```shell
  ros2 launch cod_bringup multiplenav_launch.py
  ```
  - 单点导航
  ```shell
  sudo apt-get install -y ros-humble-spatio-temporal-voxel-layer
- 构建方式
  ```shell
  colcon build --symlink
  source install/setup.bash
- 运行方式（启动导航前先启动雷达）
  - 多点导航
  ```shell
  ros2 launch cod_bringup multiplenav_launch.py
  ```
  - 单点导航
  ```shell
  ros2 launch cod_bringup singlenav_launch.py
  ```
## 注意
  - 启动导航程序之前还应发布一个雷达坐标系到底盘坐标系的TF（可写在雷达驱动中）
  - 此导航程序的设计原理保持Keep It Simple Stupid的原则，发布坐标系静态转换比运用urdf维护更简单易操作
  - cod_bringup中的auto_save_map.launch.py是运用在slam_toolbox上的自动保存地图程序，注意修改文件路径，可通过修改intervals中的参数修改保存地图的时间间隔
  - slam_toolbox建图参数可在mapper_params_online_async.yaml中调试修改
  - 使用slam_toolbox建出的地图仅有.pgm和.yaml生成，无需PCD
  - wps中csv文件的航点路径依次为“去增益点”“回启动区”“增益点后半区巡逻”“增益点内四角巡逻”“增益点前半区巡逻”“前压巡逻”
  - 使用单点导航时记得修改参数文件中的地图路径和localization_launch.py中的yaml文件名
  - 导航启动文件中有深度相机节点，无深度相机可注释或删除
  - 该导航程序详细解说教程可关注b站
  【RM COD导航分享一】 https://www.bilibili.com/video/BV1XSXZBUEYL/?share_source=copy_web&vd_source=d111e9a34ed9fbf816fa2277f4e3d017
  【RM COD导航分享二 loopback sim和waypoint editor】 https://www.bilibili.com/video/BV1r79cBME6Y/?share_source=copy_web&vd_source=d111e9a34ed9fbf816fa2277f4e3d017


