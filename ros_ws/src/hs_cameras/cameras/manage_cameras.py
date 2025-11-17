#!/usr/bin/env python3
import os
import logging

try:
    # --- Works when built and launched via ROS 2 ---
    from cameras.utils import (
        load_camera_config,
        choose_pixel_format,
        assign_names,
        DEFAULT_OAK_PARAMS,
        DEFAULT_USB_PARAMS,
    )
    from cameras.detect_cameras import DetectCameras, USBCamera, DepthAICamera
except ModuleNotFoundError:
    # --- Works when running directly from VS Code or src/ ---
    from utils import (
        load_camera_config,
        choose_pixel_format,
        assign_names,
        DEFAULT_OAK_PARAMS,
        DEFAULT_USB_PARAMS,
    )
    from detect_cameras import DetectCameras, USBCamera, DepthAICamera

# ---------------------------------------------------------------------------
# Camera Manager
# ---------------------------------------------------------------------------
class CameraManager:
    """Detects and configures all connected cameras (USB + OAK)."""

    def __init__(self, config_path: str):
        self.config_path = config_path
        self.logger = logging.getLogger("CameraManager")

    def get_camera_configurations(self):
        """Detect and return structured list of camera configurations."""

        oak_cfg, usb_cfg = load_camera_config(self.config_path)
        detector = DetectCameras()
        detected = detector.detect()

        # Split into types
        usb_detected = [c for c in detected if isinstance(c, USBCamera)]
        oak_detected = [c for c in detected if isinstance(c, DepthAICamera)]

        # Assign names
        oak_assigned = assign_names(oak_detected, oak_cfg, prefix="oak", id_attr="id")
        usb_assigned = assign_names(usb_detected, usb_cfg, prefix="camera", id_attr="id")

        # Update USB params
        for cam in usb_assigned:
            dev_path = getattr(cam, "device", None)
            params = getattr(cam, "params", {})

            # Make sure the device path and pixel format are correct
            params.setdefault("video_device", dev_path)
            params.setdefault("pixel_format", choose_pixel_format(dev_path))
            cam.params = params

        # Build unified configuration output
        cameras = []
        for cam in oak_assigned + usb_assigned:
            cam_type = "oak" if isinstance(cam, DepthAICamera) else "usb"
            config_dict = oak_cfg if cam_type == "oak" else usb_cfg

            cameras.append({
                "type": cam_type,
                "name": getattr(cam, "assigned_name", cam.name),
                "id": cam.id,
                "port": getattr(cam, "port_path", getattr(cam, "device", None)),
                "params": cam.params,
                "configured": cam.id in config_dict,
            })

        # Log results
        for cam in cameras:
            self.logger.info(f"Assigned {cam['name']} ({cam['type']}) → {cam['port']}")

        return cameras

# ---------------------------------------------------------------------------
# Test main (for running directly in VS Code or terminal)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    current_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.abspath(os.path.join(current_dir, "../.."))
    if package_root not in sys.path:
        sys.path.insert(0, package_root)
        print(f"[Info] Added package root to sys.path: {package_root}")

    config_path = os.path.abspath(os.path.join(current_dir, "..", "config", "cameras.yaml"))
    print(f"[CameraManager Test] Using config file: {config_path}")

    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()

    print("\n=== Managed Cameras ===")
    for cam in cameras:
        print(f"- [{cam['type'].upper()}] {cam['name']} ({'configured' if cam['configured'] else 'new'})")
        print(f"  ID: {cam['id']}")
        print(f"  Port: {cam['port']}")
        print(f"  Params: {cam['params']}\n")

