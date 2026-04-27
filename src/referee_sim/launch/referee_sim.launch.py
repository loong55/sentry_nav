from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for simulated referee topics",
            ),
            Node(
                package="referee_sim",
                executable="referee_sim_node",
                name="referee_sim",
                namespace=namespace,
                output="screen",
            )
        ]
    )
