from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="referee_sim",
                executable="referee_sim_node",
                name="referee_sim",
                output="screen",
            )
        ]
    )
