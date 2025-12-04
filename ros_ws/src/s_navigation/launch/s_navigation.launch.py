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
            get_package_share_directory("s_navigation"),
            "config"]
    )

    nav2_config_arg = DeclareLaunchArgument(
        "nav2_config",
        default_value="",
        description="Name of config file for Nav2, located the the config folder this package",
    )

    navsat_config_arg = DeclareLaunchArgument(
        "navsat_config",
        default_value="",
        description="Name of config file for Navsat, located the the config folder this package",
    )

    use_simtime_arg = DeclareLaunchArgument(
        "use_simtime",
        default_value="",
        description="Use simulation (Gazebo) clock if true, should be used for simulation only",
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
        )

    waypoint_bridge_node = Node(
        package="hs_navigation",
        executable="waypoint_bridge_node.py",
        name="waypoint_bridge",
        output="screen",
        parameters=[],
        remappings=[
            ("waypoints", "waypoints"),
            ("waypoint_status", "waypoint_status"),
        ],
    )
    

    actions = [
        PushROSNamespace(namespace),
        robot_localization_node,
        waypoint_bridge_node,
        TimerAction(period=5.0, actions=[slam_toolbox_cmd]),
        TimerAction(period=10.0, actions=[nav2_bringup_cmd])
    ]
    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)

    return LaunchDescription([
        
    ])
