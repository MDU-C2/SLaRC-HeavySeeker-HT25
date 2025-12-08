import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():



    config_dir = PathJoinSubstitution(
        [
            get_package_share_directory("s_perception"),
            "config"]
    )

    livox_config_arg = DeclareLaunchArgument(
        "livox_config",
        default_value="",
        description="Name of config file for Livox, located the the config folder this package",
    )

    ardu_config_arg = DeclareLaunchArgument(
        "ardu_config",
        default_value="ardu_config.yaml",
        description="Name of config file for Ardu, located the the config folder this package",
    )

    cameras_config_arg = DeclareLaunchArgument(
        "cameras_config",
        default_value="",
        description="Name of config file for Cameras, located the the config folder this package",
    )

    camera_server_config_arg = DeclareLaunchArgument(
        "camera_server_config",
        default_value="",
        description="Name of config file for Camera server, located the the config folder this package",
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
        )



    ld = LaunchDescription()

    return LaunchDescription([
        
    ])
