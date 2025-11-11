#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushROSNamespace

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory("hs_navigation")

    dual_ekf_navsat_config = PathJoinSubstitution(
        [nav_dir, "config", "dual_ekf_navsat.yaml"]
    )

    ARGUMENTS = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Robot namespace"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time from simulation",
            choices=["True", "False"],
        ),
    ]

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rl_ekf_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[dual_ekf_navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/filtered")],
    )

    rl_ekf_map_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[dual_ekf_navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/global")],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[dual_ekf_navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )



    actions = [
        PushROSNamespace(namespace),
        rl_ekf_odom_node,
        rl_ekf_map_node,
        navsat_transform_node,
    ]
    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)
    return ld
