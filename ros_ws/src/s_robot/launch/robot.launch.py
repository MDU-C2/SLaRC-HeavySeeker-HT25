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
    # Get directories
    robot_dir = get_package_share_directory('s_robot')
    params_dir = os.path.join(robot_dir, 'params')
    

    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Robot namespace')

    cmd_start_health_checker = Node(
        package='s_robot',
        executable='health_check',
        name='health_checker',
        namespace=namespace,
        parameters=[os.path.join(params_dir, 'robot.yaml')],
        remappings=[
            ('/safety_stop', 'platform/safety_stop'),
            ('/allowed_operation_modes', 'allowed_operation_modes') # to be removed
        ]
    )

    cmd_start_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        namespace=namespace,
        parameters=[os.path.join(params_dir, 'twist_mux.yaml')],
        output='screen',
        remappings=[
            ('/cmd_vel_out', 'platform/cmd_vel')
        ]
    )

    cmd_start_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        namespace=namespace,
        parameters=[os.path.join(params_dir, 'joy.yaml')],
        output='screen',
        remappings=[
            ('/cmd_vel', 'local_joy/cmd_vel'),
            ('/joy', 'local_joy/joy')
        ]
    )

    

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    
    #ld.add_action(start_zenoh_router) # For lightseeker platform
    ld.add_action(cmd_start_health_checker)
    ld.add_action(cmd_start_twist_mux)
    ld.add_action(cmd_start_twist_joy)

    return ld
