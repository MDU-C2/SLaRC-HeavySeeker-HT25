import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

################### user configure parameters for ros2 start ###################


cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')


with open(os.path.join(cur_config_path, 'param_config.json')) as f:
    param_config = json.load(f)

################### user configure parameters for ros2 end #####################

livox_ros2_default_params = {
    "xfer_format": param_config.get("xfer_format", 0),
    "multi_topic": param_config.get("multi_topic", 1),
    "data_src": param_config.get("data_src", 0),
    "publish_freq": param_config.get("publish_freq", 10.0),
    "output_data_type": param_config.get("output_type", 0),
    "frame_id": param_config.get("frame_id", "livox_frame"),
    "lvx_file_path": param_config.get("lvx_file_path", "/home/livox/livox_test.lvx"),
    "user_config_path": user_config_path,
    "cmdline_input_bd_code": param_config.get("cmdline_bd_code", "livox0000000001")
}


def generate_launch_description():

    # Declare the launch arguments that can be overwritten from outside
    declare_xfer_format_cmd = DeclareLaunchArgument(
        'xfer_format', default_value=str(livox_ros2_default_params['xfer_format']),
        description='Transfer format: 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format')
    
    declare_multi_topic_cmd = DeclareLaunchArgument(
        'multi_topic', default_value=str(livox_ros2_default_params['multi_topic']),
        description='Multi topic: 0-All LiDARs share the same topic, 1-One LiDAR one topic')
    
    declare_data_src_cmd = DeclareLaunchArgument(
        'data_src', default_value=str(livox_ros2_default_params['data_src']),
        description='Data source: 0-lidar, others-Invalid data src')
    
    declare_publish_freq_cmd = DeclareLaunchArgument(
        'publish_freq', default_value=str(livox_ros2_default_params['publish_freq']),
        description='Publish frequency')
    
    declare_output_type_cmd = DeclareLaunchArgument(
        'output_data_type', default_value=str(livox_ros2_default_params['output_data_type']),
        description='Output data type')
    
    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value=livox_ros2_default_params['frame_id'],
        description='Frame ID')
    
    declare_lvx_file_path_cmd = DeclareLaunchArgument(
        'lvx_file_path', default_value=livox_ros2_default_params['lvx_file_path'],
        description='LVX file path')
    
    declare_user_config_path_cmd = DeclareLaunchArgument(
        'user_config_path', default_value=livox_ros2_default_params['user_config_path'],
        description='User config path')
    
    declare_cmdline_bd_code_cmd = DeclareLaunchArgument(
        'cmdline_input_bd_code', default_value=livox_ros2_default_params['cmdline_input_bd_code'],
        description='Command line BD code')

    livox_ros2_params = [
        {"xfer_format": LaunchConfiguration('xfer_format')},
        {"multi_topic": LaunchConfiguration('multi_topic')},
        {"data_src": LaunchConfiguration('data_src')},
        {"publish_freq": LaunchConfiguration('publish_freq')},
        {"output_data_type": LaunchConfiguration('output_data_type')},
        {"frame_id": LaunchConfiguration('frame_id')},
        {"lvx_file_path": LaunchConfiguration('lvx_file_path')},
        {"user_config_path": LaunchConfiguration('user_config_path')},
        {"cmdline_input_bd_code": LaunchConfiguration('cmdline_input_bd_code')}
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([        
        declare_xfer_format_cmd,
        declare_multi_topic_cmd,
        declare_data_src_cmd,
        declare_publish_freq_cmd,
        declare_output_type_cmd,
        declare_frame_id_cmd,
        declare_lvx_file_path_cmd,
        declare_user_config_path_cmd,
        declare_cmdline_bd_code_cmd,
        livox_driver
    ])
