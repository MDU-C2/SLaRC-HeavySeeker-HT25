#!/usr/bin/env python3
import os
import sys
import yaml
import logging
import tempfile
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


# ---- FIX: Flatten DepthAI parameter dictionary ----
def flatten_params(prefix, d):
    flat = {}
    for k, v in d.items():
        if isinstance(v, dict):
            flat.update(flatten_params(f"{prefix}{k}.", v))
        else:
            flat[f"{prefix}{k}"] = v
    return flat


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

    depthai_pointcloud_launch = os.path.join(
        get_package_share_directory("depthai_ros_driver"),
        "launch",
        "pointcloud.launch.py"
    )

    generated_yaml_files = []  # prevent GC during launch

    # --- Launch all cameras ---
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

            node_full_name = f"{node_name}/{node_name}"

            # ---- FIX: Flatten parameters for DepthAI ----
            flat_params = flatten_params("", params)

            yaml_dict = {
                node_full_name: {
                    "ros__parameters": flat_params
                }
            }

            # Write temporary YAML
            fd, yaml_path = tempfile.mkstemp(suffix=".yaml")
            with os.fdopen(fd, "w") as f:
                yaml.dump(yaml_dict, f, default_flow_style=False)

            print(f"[CameraManager] Generated YAML for {node_name}: {yaml_path}")
            generated_yaml_files.append(yaml_path)

            # Include DepthAI pointcloud launch
            depthai_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([depthai_pointcloud_launch]),
                launch_arguments={
                    "name": node_name,
                    "namespace": node_name,
                    "params_file": yaml_path,
                    "enable_depth": "true",
                    "pointcloud.enable": "true",
                    "enable_color": "true",
                    "rs_compat": "false",
                    "rectify_rgb": "true",
                }.items()
            )

            nodes.append(depthai_launch)

        else:
            nodes.append(LogInfo(msg=f"Unknown camera type: {cam['type']}"))

    if not cameras:
        nodes.append(LogInfo(msg="No cameras detected or configured — nothing to launch."))

    kill_process = ExecuteProcess(
        cmd=["pkill", "-9", "-f", "usb_cam_node_exe"],
        shell=False,
    )
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[kill_process]))

    return LaunchDescription([
        LogInfo(msg="Launching camera drivers + camera server."),
        LogInfo(msg=summary),
        cleanup,
        *nodes,
    ])
