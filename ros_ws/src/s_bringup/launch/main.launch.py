import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share = get_package_share_directory("s_bringup")

    config_path = os.path.join(pkg_share, "config", "s_config.yaml")

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    config_params = config.get("my_node", {}).get("ros__parameters", {})
    namespace = config_params.get("namespace", "default_world")
    max_speed = config_params.get("max_speed", 1.0)

    # Exempel: använd world_name som default för ett launch-argument
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_name,
        description="World to load in Gazebo",
    )

    # Exempel: skicka hela params-filen till noden som vanligt
    my_node = Node(
        package=pkg_name,
        executable="my_node_exe",
        name="my_node",
        parameters=[config_path],
        output="screen",
    )

    # Exempel: använd world_name i ett kommando till Gazebo
    start_gz = ExecuteProcess(
        cmd=["gz", "sim", LaunchConfiguration("world")],
        output="screen",
    )

    # Eller max_speed vidare som environment-variabel, argument etc
    # (beroende på vad mottagaren förstår)
    print(f"Max speed from config: {max_speed}")  # debug

    return LaunchDescription([
        world_arg,
        my_node,
        start_gz,
    ])
