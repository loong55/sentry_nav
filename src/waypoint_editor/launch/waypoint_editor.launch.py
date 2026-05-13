from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory


def resolve_default_map_yaml(package_share_dir):
    candidates = [
        os.path.join(package_share_dir, 'data', 'RMUC2026_0.05.yaml'),
        os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..', 'RMUC2026_0.05.yaml')),
    ]

    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate

    return candidates[0]


def generate_launch_description():
    pkg = get_package_share_directory('waypoint_editor')
    rviz_config = os.path.join(pkg, 'rviz', 'rviz_waypoint_editor.rviz')
    default_map_yaml = resolve_default_map_yaml(pkg)

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value=default_map_yaml,
        description='Full path to map yaml file')
    map_yaml_file = LaunchConfiguration('map_yaml')

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
        )
    
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace='',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml)
    ld.add_action(map_server)
    ld.add_action(lifecycle_mgr)
    ld.add_action(rviz2)
    return ld