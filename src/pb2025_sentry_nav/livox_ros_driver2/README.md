# Livox ROS Driver 2

Livox ROS驱动器2是用于连接Livox生产的LiDAR产品的第二代驱动程序包，适用于ROS2（Foxy或Humble推荐）。

## ATTENTION

本仓库为深圳北理莫斯科大学北极熊战队内部修改版，非原版 livox_ros_driver2 。

相较于原版，修改了消息发布机制和 ```MID360_config.json``` 的 ip

```ros2 launch msg_MID360_launch.py``` 时会同时发布 ```CustomMsg``` 和 ```PointCloud2``` 两种类型消息。

 **Topic name**           | **Type**                        | **Note**
:------------------------:|:-------------------------------:|:--------------:
 /livox/lidar             | livox_ros_driver2/msg/CustomMsg | mid360 自定义消息类型
 /livox/lidar/pointcloud | sensor_msgs/msg/PointCloud2     | ROS2 点云消息格式
 /livox/imu               | sensor_msgs/msg/Imu             | mid360 机内 imu

本功能包已内置预编译的 Livox SDK2，无需再次克隆编译安装。

## 1. Preparation

  **Note :**
作为调试工具，不建议将Livox ROS驱动器用于批量生产，而是限于测试场景。您应该根据原始来源优化代码，以满足您的各种需求。

### 1.1 OS requirements

* Ubuntu 22.04 for ROS2 Humble;

  **Tips:**

  Colcon is a build tool used in ROS2.

  How to install colcon: [Colcon installation instructions](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

### 1.2 Install ROS2 Humble

For ROS2 Humble installation, please refer to:
[ROS Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Desktop-Full installation is recommend.

## 2. Build & Run Livox ROS Driver 2

### 2.1 Clone Livox ROS Driver 2 source code

```shell
git clone https://github.com/SMBU-PolarBear-Robotics-Team/livox_ros_driver2.git
```

  **Note :**

请确保将源代码克隆在“ [Work_space]/src/'文件夹（如上所示）中，否则由于编译工具限制而发生汇编错误。
### 2.2 Build & install the Livox-SDK2

  **Note :**

  Please follow the guidance of installation in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)

### 2.3 Build the Livox ROS Driver 2

### For ROS2 Humble

```shell
colcon build --symlink-install
```

### 2.4 Run Livox ROS Driver 2

```shell
source install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

in which,  

* **[launch file]** : 是您要使用的ROS2启动文件； “ lashen_ros2”文件夹包含几个启动示例供您参考。

HAP LIDAR的RVIZ发射示例将是：
```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. 启动文件和livox_ros_driver2内部参数配置指令
### 3.1 启动文件配置指令

ROS的启动文件位于“ WS_LIVOX/SRC/LIVOX_ROS_DRIVER2/launch_ros1”目录中，ROS2的启动文件位于“ WS_LIVOX/SRC/LIVOX/LIVOX_ROS_DRIVER2/launch_ros2”目录中。不同的启动文件具有不同的配置参数值，并在不同的情况下使用：
HAP是禾赛雷达，Mid360是大疆雷达，mixed是两个雷达混合使用

| launch file name          | Description                                                  |
| ------------------------- | ------------------------------------------------------------ |
| rviz_HAP.launch   | 连接到HAP LIDAR设备<br>发布PointCloud2格式数据<br>自动加载RVIZ |
| msg_HAP.launch     | 连接到HAP LIDAR设备<br>发布Livox自定义PointCloud数据|
|rviz_mid360.launch |连接到Mid 360 LiDAR设备<br> Publish PointCloud2格式数据<br>自动加载RVIZ |
| msg_mid360.launch |连接到Mid 360 LiDAR设备<br>发布Livox自定义PointCloud数据|
| rviz_mixed.launch |连接到HAP和Mid 360 LiDAR设备<br>发布PointCloud2格式数据<br>自动加载RVIZ |
| msg_mixed.launch |连接到HAP和360 MID 360 LIDAR设备<br>发布Livox自定义PointCloud数据|

### 3.2 livox ros驱动程序2内部主参数配置说明

livox_ros_driver2的所有内部参数都在启动文件中。以下是三个常用参数的详细描述：

| Parameter    | Detailed description                                         | Default |
| ------------ | ------------------------------------------------------------ | ------- |
| publish_freq | 设置Point Cloud Publish的频率<br>浮点数据类型，推荐值5.0、10.0、20.0、50.0等。最大发布频率为100.0 Hz| 10.0 |
| multi_topic  | 如果LiDAR设备具有独立主题来发布PointCloud数据<br> 0-所有LIDAR设备都使用同一主题发布PointCloud数据<br> 1-每个LIDAR设备都有其自己的主题来发布点云数据 | 0       |
| xfer_format  | Set pointcloud format<br>0 -- Livox pointcloud2(PointXYZRTLT) pointcloud format<br>1 -- Livox customized pointcloud format<br>2 -- Standard pointcloud2 (pcl :: PointXYZI) pointcloud format in the PCL library (just for ROS) | 0       |

  **Note :**

除非完全理解，否则建议不建议更改此表中未提及的其他参数。
&ensp;&ensp;&ensp;&ensp;***Livox_ros_driver2 pointcloud data detailed description :***

1. Livox pointcloud2 (PointXYZRTLT) point cloud format, as follows :

    ```c
    float32 x               # X axis, unit:m
    float32 y               # Y axis, unit:m
    float32 z               # Z axis, unit:m
    float32 intensity       # the value is 反射率, 0.0~255.0
    uint8   tag             # livox tag
    uint8   line            # laser number in lidar
    float64 timestamp       # Timestamp of point
    ```

      **Note :**

      The number of points in the frame may be different, but each point provides a timestamp.

2. Livox customized data package format, as follows :

    ```c
    std_msgs/Header header     # ROS standard message header
    uint64          timebase   # The time of first point
    uint32          point_num  # Total number of pointclouds
    uint8           lidar_id   # Lidar device id number
    uint8[3]        rsvd       # Reserved use
    CustomPoint[]   points     # Pointcloud data
    ```

    &ensp;&ensp;&ensp;&ensp;Customized Point Cloud (CustomPoint) format in the above customized data package :

    ```c
    uint32  offset_time     # offset time relative to the base time
    float32 x               # X axis, unit:m
    float32 y               # Y axis, unit:m
    float32 z               # Z axis, unit:m
    uint8   reflectivity    # reflectivity, 0~255
    uint8   tag             # livox tag
    uint8   line            # laser number in lidar
    ```

3. The standard pointcloud2 (pcl :: PointXYZI) format in the PCL library (only ROS can publish):

&ensp;&ensp;&ensp;&ensp;Please refer to the pcl :: PointXYZI data structure in the point_types.hpp file of the PCL library.

## 4. LiDAR config
LIDAR配置（例如IP，端口，数据类型...等）可以通过JSON风格的配置文件设置。单HAP，MID 360和混合LIDAR的配置文件在“配置”文件夹中。启动文件中的参数命名 *'user_config_path' *表示JSON文件路径。

1. Follow is a configuration example for HAP LiDAR (located in config/HAP_config.json):

    ```json
    {
      "lidar_summary_info" : {
        "lidar_type": 8  # protocol type index, please don't revise this value
      },
      "HAP": {
        "device_type" : "HAP",
        "lidar_ipaddr": "",
        "lidar_net_info" : {
          "cmd_data_port": 56000,  # command port
          "push_msg_port": 0,
          "point_data_port": 57000,
          "imu_data_port": 58000,
          "log_data_port": 59000
        },
        "host_net_info" : {
          "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
          "cmd_data_port": 56000,
          "push_msg_ip": "",
          "push_msg_port": 0,
          "point_data_ip": "192.168.1.5",  # host ip
          "point_data_port": 57000,
          "imu_data_ip" : "192.168.1.5",  # host ip
          "imu_data_port": 58000,
          "log_data_ip" : "",
          "log_data_port": 59000
        }
      },
      "lidar_configs" : [
        {
          "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
          "pcl_data_type" : 1,
          "pattern_mode" : 0,
          "blind_spot_set" : 50,
          "extrinsic_parameter" : {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "x": 0,
            "y": 0,
            "z": 0
          }
        }
      ]
    }
    ```

    The parameter attributes in the above json file are described in the following table :

    LiDAR configuration parameter

    | Parameter                  | Type    | Description                                                  | Default         |
    | :------------------------- | ------- | ------------------------------------------------------------ | --------------- |
    | ip             | String  | Ip of the LiDAR you want to config | 192.168.1.100 |
    | pcl_data_type             | Int | Choose the resolution of the point cloud data to send<br>1 -- Cartesian coordinate data (32 bits)<br>2 -- Cartesian coordinate data (16 bits) <br>3 --Spherical coordinate data| 1           |
    | pattern_mode                | Int     | Space scan pattern<br>0 -- non-repeating scanning pattern mode<br>1 -- repeating scanning pattern mode <br>2 -- repeating scanning pattern mode (low scanning rate) | 0               |
    | blind_spot_set (Only for HAP LiDAR)                 | Int     | Set blind spot<br>Range from 50 cm to 200 cm               | 50               |
    | extrinsic_parameter |      | Set extrinsic parameter<br> The data types of "roll" "picth" "yaw" are float <br>  The data types of "x" "y" "z" are int<br>               |

    For more infomation about the HAP config, please refer to:
    [HAP Config File Description](https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description)

2. When connecting multiple LiDARs, add objects corresponding to different LiDARs to the "lidar_configs" array. Examples of mixed-LiDARs config file contents are as follows :

    ```json
    {
      "lidar_summary_info" : {
        "lidar_type": 8  # protocol type index, please don't revise this value
      },
      "HAP": {
        "lidar_net_info" : {  # HAP ports, please don't revise these values
          "cmd_data_port": 56000,  # HAP command port
          "push_msg_port": 0,
          "point_data_port": 57000,
          "imu_data_port": 58000,
          "log_data_port": 59000
        },
        "host_net_info" : {
          "cmd_data_ip" : "192.168.1.5",  # host ip
          "cmd_data_port": 56000,
          "push_msg_ip": "",
          "push_msg_port": 0,
          "point_data_ip": "192.168.1.5",  # host ip
          "point_data_port": 57000,
          "imu_data_ip" : "192.168.1.5",  # host ip
          "imu_data_port": 58000,
          "log_data_ip" : "",
          "log_data_port": 59000
        }
      },
      "MID360": {
        "lidar_net_info" : {  # Mid360 ports, please don't revise these values
          "cmd_data_port": 56100,  # Mid360 command port
          "push_msg_port": 56200,
          "point_data_port": 56300,
          "imu_data_port": 56400,
          "log_data_port": 56500
        },
        "host_net_info" : {
          "cmd_data_ip" : "192.168.1.5",  # host ip
          "cmd_data_port": 56101,
          "push_msg_ip": "192.168.1.5",  # host ip
          "push_msg_port": 56201,
          "point_data_ip": "192.168.1.5",  # host ip
          "point_data_port": 56301,
          "imu_data_ip" : "192.168.1.5",  # host ip
          "imu_data_port": 56401,
          "log_data_ip" : "",
          "log_data_port": 56501
        }
      },
      "lidar_configs" : [
        {
          "ip" : "192.168.1.100",  # ip of the HAP you want to config
          "pcl_data_type" : 1,
          "pattern_mode" : 0,
          "blind_spot_set" : 50,
          "extrinsic_parameter" : {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "x": 0,
            "y": 0,
            "z": 0
          }
        },
        {
          "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
          "pcl_data_type" : 1,
          "pattern_mode" : 0,
          "extrinsic_parameter" : {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "x": 0,
            "y": 0,
            "z": 0
          }
        }
      ]
    }
    ```

3. when multiple nics on the host connect to multiple LiDARs, you need to add objects corresponding to different LiDARs to the lidar_configs array. Run different luanch files separately, and the following is an example of mixing lidar configuration file contents:

**MID360_config1:**

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.1.100"], # Lidar ip
                "host_ip": "192.168.1.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```

**MID360_config2:**

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.2.100"], # Lidar ip
                "host_ip": "192.168.2.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.2.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
            }
        }
    ]
}
```

**Launch1:**

```xml
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 
    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config1.json"/> # Mid360 MID360_config1 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>
    <node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>
    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>
    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>
</launch>
```

**Launch2:**

```xml
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 
    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config2.json"/> # Mid360 MID360_config2 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>
    <node name="livox_lidar_publisher1" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>
    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>
    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>
</launch>
```

## 5. Supported LiDAR list

* HAP
* Mid360
* (more types are comming soon...)

## 6. FAQ

### 6.1 launch with "livox_lidar_rviz_HAP.launch" but no point cloud display on the grid?

Please check the "Global Options - Fixed Frame" field in the RViz "Display" pannel. Set the field value to "livox_frame" and check the "PointCloud2" option in the pannel.

### 6.2 launch with command "ros2 launch livox_lidar_rviz_HAP_launch.py" but cannot open shared object file "liblivox_sdk_shared.so" ?

Please add '/usr/local/lib' to the env LD_LIBRARY_PATH.

* If you want to add to current terminal:

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

* If you want to add to current user:

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```
