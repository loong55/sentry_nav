import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def _build_spawn_actions(context):
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro_path = os.path.join(
        pkg_pb2025_robot_description,
        "resource",
        "xmacro",
        "simulation_robot.sdf.xmacro",
    )
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    gz_world_path = LaunchConfiguration("gz_world_path").perform(context)
    selected_world_arg = LaunchConfiguration("world").perform(context)

    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = selected_world_arg or config.get("world")
        robots = config["robots"].get(selected_world)

    if not robots:
        raise RuntimeError(
            f"No robots configured for world '{selected_world}' in {gz_world_path}"
        )

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    actions = []

    for robot in robots:
        # Generate SDF from xmacro
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        # replace the <robot_name> in the bridge config file
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            remappings=remappings,
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        # Execute service call after spawning robots
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        actions.append(spawn_robot)
        actions.append(robot_base)
        actions.append(robot_state_publisher)
        actions.append(robot_ign_bridge)
        actions.append(set_performer_service)

    return actions


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value="",
        description="World key used to pick robot spawn poses from gz_world.yaml",
    )

    declare_gz_world_path = DeclareLaunchArgument(
        "gz_world_path",
        default_value=os.path.join(pkg_simulator, "config", "gz_world.yaml"),
        description="Path to gz_world.yaml",
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(declare_gz_world_path)
    ld.add_action(OpaqueFunction(function=_build_spawn_actions))

    return ld
