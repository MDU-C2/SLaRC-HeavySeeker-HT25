#!/usr/bin/env python3
import os
import sys
import json
import logging
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from init_cameras.manage_cameras import CameraManager
from launch_ros.descriptions import ParameterValue

logger = logging.getLogger("camera_server_launch")
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory("s_cameras"),
        "config",
        "cameras.yaml",
    )

    # Load same config so server knows about camera list
    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()
    summary = manager.build_summary_string(cameras)

    server_node = Node(
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
    )

    return LaunchDescription([
        LogInfo(msg="Launching FPV camera server"),
        LogInfo(msg=summary),
        server_node
    ])
