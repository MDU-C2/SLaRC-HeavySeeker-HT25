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
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory("hs_navigation")
    slam_dir = get_package_share_directory("slam_toolbox")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    rviz_config = PathJoinSubstitution([nav_dir, "rviz", "config.rviz"])
    nav2_config = PathJoinSubstitution([nav_dir, "config", "nav2_params.yaml"])

    ARGUMENTS = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Robot namespace"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Turn on rviz2 visualization",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time from simulation",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "use_map",
            default_value="False",
            description="Use map viz",
            choices=["True", "False"],
        ),
    ]

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    imu_g_to_ms2_node = Node(
        package="hs_navigation",
        executable="imu_g_to_ms2.py",
        name="livox_imu_conv",
        output="screen",
        remappings=[("imu/data", "livox/imu_192_168_10_93"),
                    ("imu_conv/data", "livox/imu/data")]
    )

    mapviz_launch = PathJoinSubstitution(
          [
              get_package_share_directory("hs_navigation"),
              'launch',
              'hs_navigation_mapviz.launch.py',
          ]
      )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
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


    # gnss_to_rig_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     # x  y  z   roll pitch yaw   parent_frame   child_frame
    #     arguments=["0.0","0.0","0.0","0.0","0.0","0.0","gnss_link", "Rig5/gnss_link/gnss"],
    #     output="screen",
    #     name="gnss_to_rig_static_tf"
    # )

    mapviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapviz_launch),
        condition=IfCondition(LaunchConfiguration("use_map")),
    )


    # Robot localization node using world and map ekf
    robot_localization_node = IncludeLaunchDescription(
        PathJoinSubstitution([nav_dir, "launch", "hs_navsat.launch.py"]),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )

    slam_toolbox_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([slam_dir, "launch", "online_async_launch.py"]),
        launch_arguments=[("use_sim_time", use_sim_time), ("namespace", namespace)],
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([nav2_bringup_dir, "launch", "navigation_launch.py"]),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("namespace", namespace),
            ("params_file", nav2_config),
        ],
    )

    actions = [
        PushROSNamespace(namespace),
        imu_g_to_ms2_node,
        robot_localization_node,
        waypoint_bridge_node,
        rviz_node,
        mapviz_launch_description,
        TimerAction(period=5.0, actions=[slam_toolbox_cmd]),
        TimerAction(period=10.0, actions=[nav2_bringup_cmd])
    ]
    hs = GroupAction(actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(hs)
    # ld.add_action(gnss_to_rig_tf)
    return ld
