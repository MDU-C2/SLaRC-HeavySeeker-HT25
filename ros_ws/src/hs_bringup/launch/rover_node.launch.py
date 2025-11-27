import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from ament_index_python.packages import get_package_share_directory

os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{time}: [{name}] [{severity}]\t{message}"
# Verbose log:
# os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

# Start as component:


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

    rover_node_params = LaunchConfiguration("rover_node")
    bringup_dir = get_package_share_directory("hs_bringup")

    arg_file_name = "rover_node.yaml"

    rover_node_params_path = DeclareLaunchArgument(
        "rover_node",
        default_value=[PathJoinSubstitution([bringup_dir, "config", arg_file_name])],
    )

    node = Node(
        package="septentrio_gnss_driver",
        executable="septentrio_gnss_driver_node",
        name="septentrio_gnss_driver",
        emulate_tty=True,
        sigterm_timeout="20",
        parameters=[rover_node_params],
        remappings=[("pose", "gps/pose"), ("twist", "gps/twist")]
    )

    return launch.LaunchDescription(
        [
            rover_node_params_path,
            node,
            tf_imu,
            tf_gnss,
            tf_vsm,
            tf_aux1,
        ]
    )
