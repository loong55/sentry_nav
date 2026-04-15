from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_share_dir = FindPackageShare('location')
    script_path = PathJoinSubstitution([package_share_dir, 'scripts', 'map_visualizer.py'])
    map_file = PathJoinSubstitution([package_share_dir, 'map', 'RMUC2025_real.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value=map_file,
            description='Path to the map YAML file'
        ),
        ExecuteProcess(
            cmd=['python3', script_path, LaunchConfiguration('map_file')],
            output='screen'
        )
    ])
