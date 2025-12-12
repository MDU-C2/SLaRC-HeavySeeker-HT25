import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

################### configs start ###################
autostart = 'true'
use_lifecycle_manager = 'false'
use_sim_time = 'false'
slam_params_file = PathJoinSubstitution([get_package_share_directory(
    "hs_robot"), "conf", "slam_async_config.yaml"])
nav2_params_file = PathJoinSubstitution(
    [get_package_share_directory("hs_robot"), "conf", "nav2_params.yaml"])
output_velocity_topic = "platform/cmd_vel"
################### configs end #####################


def generate_launch_description():
    # Directories
    hs_bringup_dir = get_package_share_directory('hs_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    hs_description_dir = get_package_share_directory('hs_description')
    hs_robot_dir = get_package_share_directory('hs_robot')
    hs_nav_dir = get_package_share_directory('hs_navigation')
    fast_lio_dir = get_package_share_directory('fast_lio')
    l_t_pcl_dir = get_package_share_directory("livox_to_pointcloud2")

    ekf_conf = PathJoinSubstitution(
        [hs_robot_dir, 'conf', 'localization.yaml'])

    # Descriptions

    # Robot_State_Publisher
    description_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([hs_description_dir, 'launch', 'hs_description.launch.py'])))

    desc_group = GroupAction([SetRemap(
        src='joint_states', dst='/a200_0309/platform/joint_states'), description_launch])

    # EKF
    # ekf_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_test_node",
    #     output="screen",
    #     parameters=[ekf_conf],
    # )

    # Lidar
    livox_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hs_bringup_dir, 'launch', 'livox_launch.py'])
        ),
    )

    l_to_pcl_launch = IncludeLaunchDescription(
        PathJoinSubstitution([l_t_pcl_dir, 'launch', 'livox_to_pointcloud2.launch.yml'])
        )

    l_to_pcl_group = GroupAction([SetRemap(src="/livox_pointcloud", dst="livox/lidar_192_168_10_24"),
        l_to_pcl_launch])

    septentrio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [hs_bringup_dir, 'launch', 'rover_node.launch.py']
            )
        )
    )

    # Cloud2Scan
    cloud2scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hs_bringup_dir, 'launch',
                                 'cloud2scan.launch.py'])
        ),
    )

    # OAK-D
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([hs_bringup_dir, 'launch', 'oakd.launch.py'])))

    # FAST_LIO
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [fast_lio_dir, 'launch', 'mapping.launch.py']),
        ),
    )

    fast_lio_group = GroupAction([
        SetRemap(src='tf', dst='tf_unused'), fast_lio_launch])

    # SLAM
    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([slam_toolbox_dir, 'launch',
    #                              'online_sync_launch.py'])
    #     ),
    #     launch_arguments={
    #         'autostart': autostart,
    #         'use_lifecycle_manager': use_lifecycle_manager,
    #         'use_sim_time': use_sim_time,
    #         'slam_params_file': slam_params_file,
    #     }.items(),
    # )

    # NAV2
    # nav2_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [nav2_bringup_dir, 'launch', 'bringup_launch.py'])
    #     ),
    #     launch_arguments={
    #         'autostart': autostart,
    #         'use_sim_time': use_sim_time,
    #         'params_file': nav2_params_file,
    #     }.items(),
    # )

    # nav2_group = GroupAction([SetRemap(
    #     src='platform/cmd_vel', dst='/a200_0309/platform/cmd_vel'), nav2_bringup_launch])

    nav2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([hs_nav_dir, 'launch', 'hs_navigation.launch.py'])),
                                           launch_arguments={
                                               'use_map': 'False',
                                               'use_sim_time': 'False'
    }.items())

    # Group
    group = GroupAction([
        desc_group,
        # ekf_node,
        septentrio_launch,
        livox_lidar_launch,
        l_to_pcl_group,
        cloud2scan_launch,
        oak_launch,
        fast_lio_group,
        nav2_launch,
        # slam_toolbox_launch,
        # nav2_group,
    ])

    return LaunchDescription([
        group
    ])
