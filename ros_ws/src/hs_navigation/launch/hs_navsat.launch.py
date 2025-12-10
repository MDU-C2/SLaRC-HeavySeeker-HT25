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
            "namespace", default_value="/", description="Robot namespace"
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

    gps_odom_node = Node(
        package="hs_navigation",
        executable="gps_odom_node.py",
        name="gps_extract_odom",
        output="screen",
        remappings=[("pose", "gps/pose"),
                    ("twist", "gps/twist"),
                    ("odom", "gps/odom")]
    )

    rl_ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom_local",
        output="screen",
        parameters=[dual_ekf_navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    rl_ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map_global",
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
            ("imu", "gps/heading/imu"),
            ("gps/fix", "navsatfix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    gps_heading_node = Node(
        package="hs_navigation",
        executable="gps_heading.py",
        name="gps_imu_heading",
        output="screen"
    )

    actions = [
        PushROSNamespace(namespace),
        # gps_odom_node,
        rl_ekf_local_node,
        # rl_ekf_global_node,
        # navsat_transform_node,
        # gps_heading_node,
    ]
    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)
    return ld
