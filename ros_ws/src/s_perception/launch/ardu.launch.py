import os
from zipfile import Path
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"
# Verbose log:
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'


def generate_launch_description():
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 base_link imu".split(" "),
    )

    tf_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 imu gnss".split(" "),
    )

    tf_vsm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 imu vsm".split(" "),
    )

    tf_aux1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 imu aux1".split(" "),
    )



    config_arg = DeclareLaunchArgument(
         "ardu_config",
         default_value="ardu_config.yaml",
         description="World filename located under this packages worlds/ directory.",
     )

    config_path= PathJoinSubstitution([
        FindPackageShare("s_perception"),
        "config",
        LaunchConfiguration("ardu_config"),
    ])



    node = Node(
        package="septentrio_gnss_driver",
        executable="septentrio_gnss_driver_node",
        name="septentrio_gnss_driver",
        emulate_tty=True,
        sigterm_timeout="20",
        parameters=[config_path],
    )

    return launch.LaunchDescription(
        [
            config_arg,
            node,
            tf_imu,
            tf_gnss,
            tf_vsm,
            tf_aux1,
        ]
    )
