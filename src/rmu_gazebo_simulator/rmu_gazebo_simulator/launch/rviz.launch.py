import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace="red_standard_robot1",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("rmu_gazebo_simulator"),
                "rviz",
                "visualize.rviz",
            ),
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(start_rviz2)

    return ld
