#!/usr/bin/env python3
import os
import sys
import logging
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from init_cameras.manage_cameras import CameraManager

logger = logging.getLogger("s_camera_launch")
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("s_cameras"),
        "config",
        "cameras.yaml",
    )

    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()
    summary = manager.build_summary_string(cameras)
    nodes = []

    # --- Launch all camera drivers ---
    for cam in cameras:
        params = cam["params"]
        node_name = cam["name"]

        if cam["type"] == "usb":
            nodes.append(Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name=node_name,
                namespace=node_name,
                parameters=[params],
                output="screen",
            ))
            
        elif cam["type"] == "oak":

            depthai_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('depthai_ros_driver'),
                        'launch',
                        'pointcloud.launch.py'
                    )
                ]),
                launch_arguments={
                    'name': node_name,
                    'namespace' : node_name,
                    'enable_depth': 'true',
                    'pointcloud.enable': 'true',
                    'i_enable_imu' : 'false',
                    'enable_color': 'true',
                    'rs_compat': 'false',
                    'nn_enable': 'false',
                    'i_nn_type': '0',
                    'spatial_detection.enable': 'false',
                    'params_file': config_path
                }.items()
            )

            nodes.append(depthai_launch)

        else:
            nodes.append(LogInfo(msg=f"Unknown camera type: {cam['type']}"))


    if not cameras:
        nodes.append(LogInfo(msg="No cameras detected or configured — nothing to launch."))

    # Force-kill usb_cam if it misbehaves on shutdown (use -f due to 15-char truncation)
    kill_process = ExecuteProcess(
        cmd=["pkill", "-9", "-f", "usb_cam_node_exe"],
        shell=False
    )

    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[kill_process])
    )

    return LaunchDescription([
        LogInfo(msg=summary),
        cleanup,
        *nodes,
    ])

