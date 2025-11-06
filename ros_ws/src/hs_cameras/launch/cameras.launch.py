#!/usr/bin/env python3
import os
import logging
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from cameras.manage_cameras import CameraManager

logger = logging.getLogger("hs_camera_launch")
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("hs_cameras"),
        "config",
        "cameras.yaml",
    )
    logger.info(f"Using camera configuration: {config_path}")

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
                output="screen",
            ))

        elif cam["type"] == "oak":
            nodes.append(Node(
                package="depthai_ros_driver",
                executable="camera_node",
                name=node_name,
                namespace=node_name,
                parameters=[params],
                output="screen",

            ))
        else:
            logger.warning(f"Unknown camera type: {cam['type']}")

    # --- Launch FPV Server (idle until a client requests a stream) ---
    nodes.append(Node(
        package="hs_cameras",
        executable="fpv_server",
        name="fpv_server",
        output="screen",
    ))
    if not nodes:
        logger.warning("No cameras detected or configured — nothing to launch.")

    return LaunchDescription(nodes)
