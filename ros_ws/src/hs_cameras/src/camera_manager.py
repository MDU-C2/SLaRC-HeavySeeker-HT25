import os
import re
import logging
import copy
from src.utils import load_camera_config, choose_best_pixel_format
from src.detect_cameras.detect_cameras import DetectCameras



logger = logging.getLogger("CameraManager")

DEFAULT_OAK_PARAMS = {
    "camera": {
        "i_enable_imu": False,
        "i_enable_ir": False,
        "i_floodlight_brightness": 0,
        "i_laser_dot_brightness": 100,
        "i_nn_type": "none",
        "i_pipeline_type": "RGBD",
        "i_usb_speed": "SUPER_PLUS",
    },
    "rgb": {
        "i_board_socket_id": 0,
        "i_fps": 30.0,
        "i_height": 720,
        "i_interleaved": False,
        "i_max_q_size": 10,
        "i_preview_size": 250,
        "i_enable_preview": True,
        "i_low_bandwidth": True,
        "i_keep_preview_aspect_ratio": True,
        "i_publish_topic": False,
        "i_resolution": '1080P',
        "i_width": 1280,
    },
    "use_sim_time": False,
}


DEFAULT_USB_PARAMS = {
    "image_width": 1280,
    "image_height": 720,
    "pixel_format": 'mjpeg2rgb', 
    "auto_white_balance": False,
    "autoexposure": False,
    "auto_focus": False,
    "framerate": 15.0,
}

# ------------------------------------------------------------
# Helper: Assign names deterministically based on config + ports
# ------------------------------------------------------------
def assign_names(detected, config, prefix, port_key="port_path", id_key="mxid"):
    configured = config or {}
    name_slots = [f"{prefix}{i}" for i in range(10)]
    used_names = set()
    assigned = []

    # Split into known and unknown
    known, unknown = [], []
    for cam in detected:
        cam_id = str(cam.get(id_key)).strip()
        if cam_id in configured:
            known.append(cam)
        else:
            unknown.append(cam)

    # Assign known names (from config)
    for cam in known:
        cam_id = cam[id_key]
        cfg = configured.get(cam_id, {})
        name = cfg.get("name", f"{prefix}_unknown_{cam_id}")
        cam["assigned_name"] = name
        cam["params"] = cfg.get("params", {})
        used_names.add(name)
        assigned.append(cam)

    # Sort unknown by port number (ascending order)
    def port_sort(cam):
        port = str(cam.get(port_key, ""))
        nums = re.findall(r"\d+", port)
        return tuple(map(int, nums)) if nums else (999,)
    unknown.sort(key=port_sort)

    # Determine available names
    available = [n for n in name_slots if n not in used_names]

    # Assign names to unknown devices
    for cam in unknown:
        if available:
            name = available.pop(0)
        else:
            name = f"{prefix}_extra_{cam[id_key]}"
        cam["assigned_name"] = name
        cam["params"] = copy.deepcopy(DEFAULT_OAK_PARAMS if prefix == "oak" else DEFAULT_USB_PARAMS)
        assigned.append(cam)
    return assigned

# ------------------------------------------------------------
# Camera Manager (no ROS dependencies)
# ------------------------------------------------------------
class CameraManager:
    def __init__(self, config_path: str):
        self.config_path = config_path

    def get_camera_configurations(self):
        """Detect all cameras and return structured list of configurations."""
        oak_cfg, usb_cfg = load_camera_config(self.config_path)
        detector = DetectCameras()
        usb_detected, oak_detected = detector.detect()

       # Assign names
        oak_assigned = assign_names(oak_detected, oak_cfg, prefix="oak", id_key="mxid", port_key="port_path")
        usb_assigned = assign_names(usb_detected, usb_cfg, prefix="camera", id_key="id", port_key="port_path")

        # Inject the correct video_device path from detection
        for cam in usb_assigned:
            dev_path = cam.get("device")
            if "video_device" not in cam["params"] or cam["params"]["video_device"] != dev_path:
                cam["params"]["video_device"] = dev_path

        # Handle pixel format detection for unknown USB cameras
        for cam in usb_assigned:
            if "pixel_format" not in cam["params"]:
                dev = cam.get("device", "")
                cam["params"]["pixel_format"] = choose_best_pixel_format(dev)


        # Build structured results
        cameras = []
        for cam in oak_assigned:
            cameras.append({
                "type": "oak",
                "name": cam["assigned_name"],
                "id": cam["mxid"],
                "port": cam.get("port_path"),
                "params": cam["params"],
                "configured": cam["mxid"] in oak_cfg,
            })
        for cam in usb_assigned:
            cameras.append({
                "type": "usb",
                "name": cam["assigned_name"],
                "id": cam["id"],
                "port": cam.get("device"),
                "params": cam["params"],
                "configured": cam["id"] in usb_cfg,
            })
        for cam in cameras:
            logger.info(f"Assigned {cam['name']} ({cam['type']}) → {cam['port']}")

        return cameras


# ------------------------------------------------------------
# Test Main — print the managed camera info
# ------------------------------------------------------------
if __name__ == "__main__":
    print("[CameraManager Test] Detecting and managing cameras...")
    config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "cameras.yaml"))

    manager = CameraManager(config_path)
    cameras = manager.get_camera_configurations()

    print("\n=== Detected Cameras ===")
    for cam in cameras:
        print(f"- [{cam['type'].upper()}] {cam['name']}  ({'configured' if cam['configured'] else 'new'})")
        print(f"  ID: {cam['id']}")
        print(f"  Port: {cam['port']}")
        print(f"  Params: {cam['params']}")
        print()