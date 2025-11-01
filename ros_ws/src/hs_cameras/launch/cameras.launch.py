#!/usr/bin/env python3
import os
import logging
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from src.manage_cameras import CameraManager


# ----------------------------------------------------------------------
# Configure logger for this launch file
# ----------------------------------------------------------------------
logger = logging.getLogger("hs_camera_launch")
logging.basicConfig(
    level=logging.INFO,
    format="[%(name)s] %(levelname)s: %(message)s"
)


# ----------------------------------------------------------------------
# Generate launch description
# ----------------------------------------------------------------------
def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("hs_cameras"),
        "config",
        "cameras.yaml"
    )

    logger.info(f"Using camera configuration: {config_path}")

    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()

    for cam in cameras:
        logger.info(f"Assigned {cam['name']} ({cam['type']}) → {cam['port']}")

    # --- Create camera nodes ---
    nodes = []
    for cam in cameras:
        if cam["type"] == "usb":
            params = {**cam["params"], "camera_name": cam["name"]}
            nodes.append(Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name=cam["name"],
                namespace=cam["name"],
                parameters=[params],
                output="screen",
            ))

        elif cam["type"] == "oak":
            params = {**cam["params"], "camera_name": cam["name"]}
            nodes.append(Node(
                package="depthai_ros_driver",
                executable="camera_node",
                name=cam["name"],
                namespace=cam["name"],
                parameters=[params],
                output="log",
            ))


    if not nodes:
        logger.warning("No cameras detected or configured. Nothing to launch.")

    return LaunchDescription(nodes)
