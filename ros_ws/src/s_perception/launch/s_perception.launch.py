#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    perception_dir = get_package_share_directory('s_perception')

    config_dir = PathJoinSubstitution(
        [perception_dir, "config"])

    livox_config_arg = DeclareLaunchArgument(
        "livox_config",
        default_value="MID360_config.json",
        description="Name of config file for Livox, located in the config folder this package",
    )

    ardu_config_arg = DeclareLaunchArgument(
        "ardu_config",
        default_value="ardu_config.yaml",
        description="Name of config file for Ardu, located in the config folder this package",
    )

    cameras_config_arg = DeclareLaunchArgument(
        "cameras_config",
        default_value="",
        description="Name of config file for Cameras, located in the config folder this package",
    )

    camera_server_config_arg = DeclareLaunchArgument(
        "camera_server_config",
        default_value="",
        description="Name of config file for Camera server, located in the config folder this package",
    )

    namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/',
        description='Robot namespace'
    )

    ARGUMENTS = [livox_config_arg,
                 ardu_config_arg,
                 cameras_config_arg,
                 camera_server_config_arg,
                 namespace
                 ]

    livox_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [perception_dir, "launch", "livox_launch.py"])),
        launch_arguments=[
            ("user_config_path", [config_dir,
                                   LaunchConfiguration("livox_config")])
        ],
    )

    ardu_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [perception_dir, "launch", "ardu.launch.py"])),
        launch_arguments=[
            ("ardu_config", LaunchConfiguration("ardu_config"))
        ],
    )

    cloud2scan_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [perception_dir, "launch", "cloud2scan.launch.py"])),
    )

    ardu_heading_node = Node(
        executable="gps_heading.py",
        package="s_perception",
        name="gps_heading",
        output="screen",
    )

    livox_imu_g_to_ms2_node = Node(
        executable="imu_g_to_ms2.py",
        package="s_perception",
        name="livox_g_to_ms2",
        output="screen",
        remappings=[
            ("imu/data", "livox/imu_192_168_10_24"),
            ("imu_conv/data", "livox/imu/data")
        ]
    )

    actions = [
        PushRosNamespace(LaunchConfiguration('namespace')),
        ardu_launch,
        livox_launch,
        cloud2scan_launch,
        ardu_heading_node,
        livox_imu_g_to_ms2_node,
    ]
    launch_actions = GroupAction(actions=actions)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(launch_actions)

    return ld
