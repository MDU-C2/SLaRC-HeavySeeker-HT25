#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory


    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')

    start_health_checker_cmd = Node(
        package='hs_robot',
        executable='health_check',
        name='health_checker',
        namespace=namespace,
        remappings=[
            ('/emergency_stop', '/a200_0309/platform/emergency_stop'),
            ('/allowed_operation_modes', 'allowed_operation_modes')
        ]
    )

    start_robot_base_cmd = Node(
        package='hs_robot',
        executable='robot_node',
        name='robot_node',
        namespace=namespace,
        remappings=[
            ('/cmd_vel', '/a200_0309/platform/cmd_vel'),
            ('/allowed_operation_modes', 'allowed_operation_modes'),
            ('/telop_cmd_vel', 'telop_cmd_vel'),
            ('/auto_cmd_vel', 'auto_cmd_vel')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    
    #ld.add_action(start_zenoh_router) # For lightseeker platform
    ld.add_action(start_health_checker_cmd)
    ld.add_action(start_robot_base_cmd)

    return ld
