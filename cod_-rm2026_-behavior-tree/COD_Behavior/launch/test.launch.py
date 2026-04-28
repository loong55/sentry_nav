from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    nav_pose_yaml = PathJoinSubstitution([
        FindPackageShare("cod_behavior"),
        "launch",
        "test_pose.yaml"
    ])

    serial_node = Node(
        package="cod_serial",
        executable="cod_serial",
        name="cod_serial",
        output="screen"
    )

    behavior_node = Node(
        package="cod_behavior",
        executable="tree_1",
        name="cod_behavior",
        output="screen",
        parameters=[nav_pose_yaml]
    )

    return LaunchDescription([
        serial_node,
        behavior_node
    ])