#!/usr/bin/env python3
import os
import sys
import json
import logging
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from init_cameras.manage_cameras import CameraManager

#-----------------Setup logging--------------------------
logger = logging.getLogger("hs_camera_launch")
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")
logging.getLogger().addHandler(logging.StreamHandler(sys.stdout))

#=================== Launch description ==================
def generate_launch_description():
    # Path to camera config file
    config_path = os.path.join(
        get_package_share_directory("hs_cameras"),
        "config",
        "cameras.yaml",
    )

    # Run camera manager to detect all cameras and manage them-
    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()

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
                output="log",
            ))

        elif cam["type"] == "oak":
            nodes.append(Node(
                package='depthai_ros_driver',
                executable="camera_node",
                name=node_name,
                namespace='',
                parameters=[params],
                output="screen",
            ))
        else:
            print(f"Unknown camera type: {cam['type']}")

    # --- Launch FPV Server ---
    nodes.append(Node(
        package="hs_cameras",
        executable="server_node",
        name="server",
        output="screen",
        parameters=[
            {"cameras_json": ParameterValue(json.dumps(cameras), value_type=str)},

            # -------------------------------
            # Encoder use defaults unless changed here. 
            # -------------------------------
            {"encoder.prefer_hevc": False},
            {"encoder.quality": 5},
            {"encoder.latency": "ultra_low"},
            {"encoder.bitrate_mode": "CRF"},
            {"encoder.bitrate": "8M"},
            {"encoder.maxrate": "8M"},
            {"encoder.bufsize": "16M"},
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

    if not nodes:
        print(" No cameras detected or configured — nothing to launch.")

    return LaunchDescription(nodes)
