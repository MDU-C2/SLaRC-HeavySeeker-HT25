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
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', 
            default_value='', 
            description='Robot namespace'
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    namespace = LaunchConfiguration('namespace')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("s_robot"), "description", "model.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("s_robot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # Nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        namespace=namespace,
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=namespace,
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=platform/cmd_vel",
        ],
    )
    health_checker_node = Node(
        package='s_robot',
        executable='health_check',
        name='health_checker',
        namespace=namespace,
        parameters=[os.path.join(config_dir, 'robot.yaml')],
        output='both',
        remappings=[
            ('/safety_stop', 'platform/safety_stop'),
            ('/allowed_operation_modes', 'allowed_operation_modes') # to be removed
        ]
    )
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        namespace=namespace,
        parameters=[os.path.join(config_dir, 'twist_mux.yaml')],
        output='both',
        remappings=[
            ('/cmd_vel_out', 'platform/cmd_vel')
        ]
    )
    twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        namespace=namespace,
        parameters=[os.path.join(config_dir, 'joy.yaml')],
        output='both',
        remappings=[
            ('/cmd_vel', 'local_joy/cmd_vel'),
            ('/joy', 'local_joy/joy')
        ]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_driver',
        output='both',
        namespace=namespace,
        remappings=[
            ('/joy', 'local_joy/joy')
        ]
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    dealyed_nodes = [
        health_checker_node,
        twist_mux_node,
        twist_joy_node,
        joy_node,
    ]

    # Delay start of health checker and joy_controller to after 'robot_controller_node'
    delay_health_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=dealyed_nodes,
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_health_after_robot_controller
    ]


    

    # Create the launch description and populate
    return LaunchDescription(declared_arguments + nodes)
