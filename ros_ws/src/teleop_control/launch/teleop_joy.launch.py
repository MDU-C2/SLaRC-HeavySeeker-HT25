#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get directories
    robot_dir = get_package_share_directory('s_robot')
    config_dir = os.path.join(robot_dir, 'config')
    
    # Declare argument

    twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[os.path.join(config_dir, 'joy.yaml')],
        output='both',
        remappings=[
            ('/cmd_vel', 'telop_joy/cmd_vel'),
            ('/joy', 'telop_joy/joy')
        ]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_driver',
        output='both',
        remappings=[
            ('/joy', 'telop_joy/joy')
        ]
    )



    nodes = [
        twist_joy_node,
        joy_node,
    ]


    

    # Create the launch description and populate
    return LaunchDescription(nodes)
