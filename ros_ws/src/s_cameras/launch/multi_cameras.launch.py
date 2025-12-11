#!/usr/bin/env python3
import os
import sys
import yaml
import logging
import tempfile

os.environ["IMAGE_TRANSPORT_DISABLE_PLUGINS"] = "1"
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


def flatten_params(prefix, d):
    """Flatten only standard DepthAI parameter dictionaries."""
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

    generated_yaml_files = []   # keep alive

    # -------- CAMERA LOOP ----------
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
            continue

        elif cam["type"] == "oak":
            # full name required by DepthAI
            node_full_name = f"{node_name}/{node_name}"

            # extract override section (oak0:, oak2:)
            override_block = params.get(node_name, {})

            # base params without the override section
            base_params = {k: v for k, v in params.items() if k != node_name}

            # flatten base params (camera, rgb, stereo, imu)
            flat_base = flatten_params("", base_params)

            # final YAML structure
            yaml_dict = {
                node_full_name: {
                    "ros__parameters": {
                        **flat_base,            # flattened standard params
                        node_name: override_block  # unflattened override block
                    }
                }
            }

            # write temp yaml
            fd, yaml_path = tempfile.mkstemp(suffix=".yaml")
            with os.fdopen(fd, "w") as f:
                yaml.dump(yaml_dict, f, default_flow_style=False)

            print(f"[CameraManager] Generated YAML for {node_name}: {yaml_path}")
            generated_yaml_files.append(yaml_path)

            # launch depthai
            depthai_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([depthai_pointcloud_launch]),
                launch_arguments={
                    "name": node_name,
                    "namespace": node_name,
                    "params_file": yaml_path,
                    #"enable_depth": "true",
                    "pointcloud.enable": "true",
                    #"enable_color": "true",
                    #"rs_compat": "false",
                    #"rectify_rgb": "true",
                }.items(),
            )

            nodes.append(depthai_launch)

        else:
            nodes.append(LogInfo(msg=f"Unknown camera type: {cam['type']}"))

    # -------- Shutdown behavior --------
    if not cameras:
        nodes.append(LogInfo(msg="No cameras detected — nothing to launch."))

    kill_process = ExecuteProcess(
        cmd=["pkill", "-9", "-f", "usb_cam_node_exe"],
        shell=False
    )
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[kill_process]))

    return LaunchDescription([
        LogInfo(msg="Launching camera drivers + camera server."),
        LogInfo(msg=summary),
        cleanup,
        *nodes,
    ])
