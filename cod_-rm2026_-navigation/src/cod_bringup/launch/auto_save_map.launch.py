# auto_save_map.launch.py
from pathlib import Path
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    bringup_share_dir = Path(get_package_share_directory('cod_bringup'))
    workspace_root = bringup_share_dir.parents[3]
    install_setup = workspace_root / 'install' / 'setup.bash'
    auto_save_dir = workspace_root / 'src' / 'cod_bringup' / 'maps' / 'auto_save'

    def create_save_command(suffix: str) -> list:
        return [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            f'source {install_setup} && '
            f'mkdir -p {auto_save_dir} && '
            f'ros2 run nav2_map_server map_saver_cli -f {auto_save_dir}/auto_map_{suffix}'
        ]

    intervals = [ 30, 60, 90, 120, 150, 180, 210, 240, 270, 300]    #保存时间间隔
    for t in intervals:
        action = TimerAction(
            period=float(t),
            actions=[ExecuteProcess(cmd=create_save_command("$(date +%H%M%S)"), output='screen')]
        )
        ld.add_action(action)

    return ld