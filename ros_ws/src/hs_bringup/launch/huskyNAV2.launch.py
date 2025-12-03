import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

################### configs start ###################
autostart = 'true'
use_lifecycle_manager = 'false'
use_sim_time = 'false'
slam_params_file = os.path.join(get_package_share_directory("hs_bringup"), "config", "slam_async_config.yaml")
nav2_params_file = os.path.join(get_package_share_directory("hs_bringup"), "config", "nav2_params.yaml")
output_velocity_topic = "platform/cmd_vel"
################### configs end #####################


def generate_launch_description():
    # Directories
    s_cameras_dir = get_package_share_directory('s_cameras')
    fast_lio_dir = get_package_share_directory('fast_lio')
    hs_bringup_dir = get_package_share_directory('hs_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Descriptions
    # Lidar
    livox_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hs_bringup_dir, 'launch', 'livox_launch.py')
        ),
    )

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(s_cameras_dir, 'launch', 'cameras.launch.py')
        ),
    )

    # Cloud2Scan
    cloud2scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hs_bringup_dir, 'launch', 'cloud2scan.launch.py')
        ),
    )

    # SLAM
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'autostart': autostart,
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # Fast Lio
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
    )

    # NAV2
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'autostart': autostart,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )

    # Group
    group = GroupAction([
        livox_lidar_launch,
        camera_launch,
        cloud2scan_launch,
        slam_toolbox_launch,
        fast_lio_launch,
        nav2_bringup_launch,
    ])

    # Lidar Frame
    livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_static_broadcaster',
        output='screen',
        arguments=['0.17', '0.0', '0.18', '3.1415926', '0.0', '0.0', 'top_plate_rear_mount', 'livox_frame'],
    )
    
    # Oak-D Frame
    oakd_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oakd_frame_static_broadcaster',
        output='screen',
        arguments=['0.02', '-0.12', '0.13', '0.0', '0.0', '0.0', 'top_plate_front_mount', 'oak-d-base-frame'],
    )

    return LaunchDescription([
        group,
        livox_frame,
        oakd_frame,
    ])
