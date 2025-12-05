#!/usr/bin/env python3
import os
import sys
import json
import logging
from launch.event_handlers import OnShutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess
from launch_ros.descriptions import ParameterValue
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

    # --- Launch FPV Server ---
    nodes.append(Node(
        package="s_cameras",
        executable="server_node",
        name="server",
        output="screen",
        parameters=[
            {"cameras_json": ParameterValue(json.dumps(cameras), value_type=str)},
            {"encoder.prefer_hevc": False},
            {"encoder.quality": 5},
            {"encoder.latency": "ultra_low"},
            {"encoder.bitrate_mode": "CRF"},
            {"encoder.bitrate": "12M"},
            {"encoder.maxrate": "12M"},
            {"encoder.bufsize": "24M"},
            {"encoder.crf": 23},
            {"encoder.gop": 1},
            {"encoder.bframes": 0},
            {"encoder.mux": "mpegts"},
            {
                "encoder.mux_flags":
                "-flush_packets 1 -fflags nobuffer -max_delay 0 "
                "-muxdelay 0 -muxpreload 0"
            },
        ]
    ))

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
