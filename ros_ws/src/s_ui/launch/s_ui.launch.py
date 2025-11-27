import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------ Launch arguments ------------
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="seeker_sim_config.rviz",
        description=(
            "Config filename located under s_ui/config/ "
            "directory (e.g. seeker_sim_config.rviz)."
        ),
    )

    use_foxglove = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Start foxglove_bridge",
    )

    use_map = DeclareLaunchArgument(
        "use_map",
        default_value="false",
        description="Start Mapviz with HS navigation config",
    )

    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with HS description config",
    )

    # ------------ RViz config path ------------
    rviz_config_root = PathJoinSubstitution(
        [
            get_package_share_directory("s_ui"),
            "config",
            LaunchConfiguration("rviz_config"),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_root],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ------------ Foxglove bridge ------------
    foxglove_xml_path = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_xml_path),
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    # ------------ Mapviz ------------
    mapviz_launch = PathJoinSubstitution(
        [
            get_package_share_directory("s_ui"),
            "launch",
            "mapviz.launch.py",
        ]
    )

    mapviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapviz_launch),
        condition=IfCondition(LaunchConfiguration("use_map")),
    )

    # ------------ Build LaunchDescription ------------
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(rviz_config_arg)
    ld.add_action(use_foxglove)
    ld.add_action(use_map)
    ld.add_action(use_rviz)

    # Add actions (UI components)
    ld.add_action(rviz_node)
    ld.add_action(foxglove_bridge_launch)
    ld.add_action(mapviz_launch_description)

    return ld
