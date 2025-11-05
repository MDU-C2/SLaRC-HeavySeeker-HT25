from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
import os


################### user configure parameters for ros2 ###################
min_height=2.0
max_height=10.0
angle_min=-1.5708
angle_max=1.5708
angle_increment=0.005
queue_size=5
scan_time=0.1
range_min=0.1
range_max=100.0
target_frame="livox_frame"
transform_tolerance=0.01
use_inf=True
cloud_in="cloud"

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
    {"target_frame": target_frame},
    {"transform_tolerance": transform_tolerance},
    {"use_inf": use_inf},
]

def generate_launch_description():
    cloud2scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=cloud2scan_params,
        remappings=[('cloud_in', cloud_in)]
        )

    return LaunchDescription([
        cloud2scan,
    ])
