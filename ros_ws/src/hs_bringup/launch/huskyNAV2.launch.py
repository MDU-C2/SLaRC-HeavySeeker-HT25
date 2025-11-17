import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

################### configs start ###################
#SLAM:
autostart = True
use_lifecycle_manager = False
use_sim_time = True
slam_params_file = os.path.join(get_package_share_directory("hs_bringup"), "config", "slam_sync_config.yaml")

#NAV2:
output_velocity_topic = "platform/cmd_vel"
################### configs end #####################

slam_params = [
    {"autostart": autostart},
    {"use_lifecycle_manager": use_lifecycle_manager},
    {"use_sim_time": use_sim_time},
    {"slam_params_file": slam_params_file},
]


def generate_launch_description():
    livox_lidar = Node(
        package='hs_bringup',
        executable='livox_launch.py',
        name='livox_lidar_publisher',
        output='screen',
        )
    
    cloud2scan = Node(
        package='hs_bringup',
        executable='cloud2scan.launch.py',
        name='cloud2scan_node',
        output='screen',
        )
    
    livox_frame = ExecuteProcess(cmd=[
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '0.17', '0.0', '0.18',
        '3.1415926', '0.0', '0.0',
        'top_plate_rear_mount', 'livox_frame'],
        output='screen'),

    slam_node = Node(
        package='slam_toolbox',
        executable='online_sync_launch.py',
        name='slam_bringup',
        output='screen',
        parameters=slam_params,
        )
    
    nav2_node = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        remappings=[('cmd_vel', output_velocity_topic)]
        )

    return LaunchDescription([
        livox_lidar,
        cloud2scan,
        livox_frame,
        slam_node,
        nav2_node,
    ])
