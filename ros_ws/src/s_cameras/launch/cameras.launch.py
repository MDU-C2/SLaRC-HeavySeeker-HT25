#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("s_cameras")

    multi_cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "multi_cameras.launch.py")
        )
    )

    camera_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "camera_server.launch.py")
        )
    )

    return LaunchDescription([
        LogInfo(msg="Launching camera drivers + camera server."),

        multi_cameras_launch,
        camera_server_launch,
    ])
