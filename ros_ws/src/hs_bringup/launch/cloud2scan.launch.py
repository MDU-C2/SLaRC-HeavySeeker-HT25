from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

################### user configure parameters for ros2 ###################
min_height=-0.4
max_height=0.1
angle_min=-3.1416
angle_max=3.1416
angle_increment=0.005
queue_size=5
scan_time=0.1
range_min=0.3
range_max=100.0
idk="frame"
transform_tolerance=0.01
use_inf=True

cloud2scan_params = [
    {"min_height": min_height},
    {"max_height": max_height},
    {"angle_min": angle_min},
    {"angle_max": angle_max},
    {"angle_increment": angle_increment},
    {"queue_size": queue_size},
    {"scan_time": scan_time},
    {"range_min": range_min},
    {"range_max": range_max},
    {"target_frame": LaunchConfiguration("target_frame")},
    {"transform_tolerance": transform_tolerance},
    {"use_inf": use_inf},
]

def generate_launch_description():
    # add launch argument for cloud_topic instead of hardcording it
    cloud_topic_arg = DeclareLaunchArgument(
        name='cloud_topic',
        default_value="livox/lidar_192_168_10_93",
        description='Topic name for the input point cloud'
    )
    #add another launch argument for the target_frame
    target_frame_arg = DeclareLaunchArgument(
        name='target_frame',
        default_value="livox_frame",
        description='Target frame for the point cloud'
    )

    cloud2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=cloud2scan_params,
        remappings=[('cloud_in', LaunchConfiguration("cloud_topic"))]
        )

    return LaunchDescription([
        cloud_topic_arg,
        target_frame_arg,
        cloud2scan
    ])
