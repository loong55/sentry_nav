# Copyright 2025 SMBU-PolarBear-Robotics-Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os # 拼接路径
from ament_index_python.packages import get_package_share_directory # 获取包的路径

from launch import LaunchDescription # launch主容器
from launch.actions import (
    DeclareLaunchArgument,      # 声明参数（launch内部形参）
    GroupAction,                # 组合多个launch
    IncludeLaunchDescription,   # 包含其他launch文件
    SetEnvironmentVariable,     # 设置环境变量
)
from launch.launch_description_sources import PythonLaunchDescriptionSource #指定python类型的launch文件,用于IncludeLaunchDescription包含其他launch文件
from launch.substitutions import LaunchConfiguration # 获取launch内部形参
from launch_ros.actions import Node, PushRosNamespace, SetRemap # ROS2节点，命名空间，重映射
from launch_ros.descriptions import ParameterFile # 参数文件
from nav2_common.launch import RewrittenYaml # 动态重写yaml文件


def generate_launch_description():
    # Get the launch directory
    pkg_standard_robot_pp_ros2_dir = get_package_share_directory(
        "standard_robot_pp_ros2"
    )
    pkg_pb2025_robot_description_dir = get_package_share_directory(
        "pb2025_robot_description"
    )

    # 先创建对参数的引用，作为占位符，实际值由后面的DeclareLaunchArgument声明赋值
    # 也可再命令行启动时覆盖默认值，如：ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py namespace:=my_namespace
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    robot_name = LaunchConfiguration("robot_name")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # 创建自己的临时YAML文件，包括替换
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,        # 源YAML文件路径
            root_key=namespace,              # 根key（命名空间）
            param_rewrites={},               # 参数重写
            convert_types=True,              # 转换类型
        ),
        allow_substs=True,                  # 允许替换
    )

    # 设置环境变量：缓冲日志输出（减少系统调用，提高性能）
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # 设置环境变量：彩色输出
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    #声明启动参数，这部分定义了可以在启动时传入的6个参数，每个都有默认值和描述。
    #命名空间（节点话题加前缀）
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    # 参数文件
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            pkg_standard_robot_pp_ros2_dir,
            "config",
            "standard_robot_pp_ros2.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # 机器人名称，选择机器人模型（URDF）
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="fjut2025_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )
    # 是否使用rviz
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RViz"
    )

    # 是否使用respawn，节点崩溃时是否自动重启，提高系统鲁棒性（如串口通信失败，自动重启节点）
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )
    # 日志级别，默认info
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # 组织启动任务，将多个独立的action组织在一起
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"), #话题重映射 ('原话题名', '目标话题名')，将绝对话题名转换为相对话题名，可以让/tf变为/namespace/tf
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_pb2025_robot_description_dir,
                        "launch",
                        "robot_description_launch.py", # 同时启动joint_state_publisher，robot_state_publisher节点
                    )
                ),
                launch_arguments={
                    "params_file": params_file,
                    "robot_name": robot_name,
                    "use_rviz": use_rviz,
                    "use_respawn": use_respawn,
                    "log_level": log_level,
                }.items(),
            ),
            Node(
                package="standard_robot_pp_ros2",
                executable="standard_robot_pp_ros2_node", #启动串口通信节点
                name="standard_robot_pp_ros2",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="standard_robot_pp_ros2",
                executable="gimbal_manager_node", #启动云台管理节点（角度控制，订阅视觉目标识别信息）
                name="gimbal_manager",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
    )
    # 创建launch描述对象
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # 声明启动参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 添加启动所有节点的actions
    ld.add_action(bringup_cmd_group)

    return ld
