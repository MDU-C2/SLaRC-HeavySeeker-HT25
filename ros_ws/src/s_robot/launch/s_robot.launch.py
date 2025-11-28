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

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
        )

    

    ld = LaunchDescription()

    return LaunchDescription([
        
    ])
