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


    control_mode_arg = DeclareLaunchArgument(
        "Control_mode",
        default_value="",
        description="Control mode for the robot, e.g., manual or autonomous",
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
        )

    

    ld = LaunchDescription()

    return LaunchDescription([
        
    ])
