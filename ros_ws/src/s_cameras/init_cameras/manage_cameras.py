#!/usr/bin/env python3
import os
import sys
import logging

try:
    # ROS / package execution
    from utils.camera_utils import (
        load_camera_config,
        choose_pixel_format,
        assign_names,
        DEFAULT_OAK_PARAMS,
        DEFAULT_USB_PARAMS,
    )
    from init_cameras.detect_cameras import DetectCameras, USBCamera, DepthAICamera

except ModuleNotFoundError:
    # Direct script execution → add project root
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(current_dir, ".."))
    sys.path.insert(0, project_root)

    # Retry imports (to be run as standalone program)
    from utils.camera_utils import (
        load_camera_config,
        choose_pixel_format,
        assign_names,
        DEFAULT_OAK_PARAMS,
        DEFAULT_USB_PARAMS,
    )
    from init_cameras.detect_cameras import DetectCameras, USBCamera, DepthAICamera


logging.basicConfig(
    level=logging.INFO,
    format="[%(name)s] %(levelname)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)


# ---------------------------------------------------------------------------
# Camera Manager
# ---------------------------------------------------------------------------
class CameraManager:

    def __init__(self, config_path: str):
        self.config_path = config_path
        self.logger = logging.getLogger("CameraManager")
    
    # Used for logging
    def build_summary_string(self, cameras):
        lines = ["\n=== Managed Cameras ==="]
        for cam in cameras:
            lines.append(
                f"- [{cam['type'].upper()}] {cam['name']} "
                f"({'configured' if cam['configured'] else 'new'})"
            )
            lines.append(f"  ID: {cam['id']}")
            lines.append(f"  Port: {cam['port']}")
            lines.append(f"  Params: {cam['params']}\n")
        return "\n".join(lines)


    # Full detection + config pipeline
    def get_camera_configurations(self):
        oak_cfg, usb_cfg = load_camera_config(self.config_path)
        detector = DetectCameras()
        detected = detector.detect()

        # Split into types
        usb_detected = [c for c in detected if isinstance(c, USBCamera)]
        oak_detected = [c for c in detected if isinstance(c, DepthAICamera)]

        # Assign names
        oak_assigned = assign_names(oak_detected, oak_cfg, prefix="oak", id_attr="id")
        usb_assigned = assign_names(usb_detected, usb_cfg, prefix="camera", id_attr="id")

        # Fix USB params
        for cam in usb_assigned:
            dev_path = getattr(cam, "device", None)
            params = getattr(cam, "params", {})

            params.setdefault("video_device", dev_path)
            params.setdefault("pixel_format", choose_pixel_format(dev_path))

            cam.params = params

        # Build final unified structure
        cameras = []
        for cam in oak_assigned + usb_assigned:
            cam_type = "oak" if isinstance(cam, DepthAICamera) else "usb"
            config_dict = oak_cfg if cam_type == "oak" else usb_cfg

            cameras.append(
                {
                    "type": cam_type,
                    "name": getattr(cam, "assigned_name", cam.name),
                    "id": cam.id,
                    "port": getattr(cam, "port_path", getattr(cam, "device", None)),
                    "params": cam.params,
                    "configured": cam.id in config_dict,
                }
            )

        for cam in cameras:
            self.logger.info(f"Assigned {cam['name']} ({cam['type']}) → {cam['port']}")

        # Log summary for standalone usage
        self.logger.info(self.build_summary_string(cameras))

        return cameras



# ---------------------------------------------------------------------------
# Standalone Test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))

    config_path = os.path.abspath(
        os.path.join(current_dir, "..", "config", "cameras.yaml")
    )

    print(f"[CameraManager Test] Using config file: {config_path}")

    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()
