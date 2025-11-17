import subprocess
import yaml
import os
import re
import logging
import copy

# ---------------------------------------------------------------------------------
#------------------------ Camera config utils -------------------------------------
# ---------------------------------------------------------------------------------

#===================== Default camera parameters =====================
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
        "i_resolution": "1080P",
        "i_width": 1280,
    },
    "use_sim_time": False,
}

DEFAULT_USB_PARAMS = {
    "image_width": 640,
    "image_height": 480,
    "pixel_format": "mjpeg2rgb",
    "auto_white_balance": False,
    "autoexposure": False,
    "auto_focus": False,
    "framerate": 30.0,
}

logger = logging.getLogger("CameraManager")
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_camera_config(yaml_path: str):
    logger = logging.getLogger("CameraConfig")

    if not os.path.exists(yaml_path):
        logger.warning("Camera configuration file not found: %s", yaml_path)
        return {}, {}

    try:
        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        logger.error("Error parsing YAML: %s", e)
        return {}, {}

    oak_cameras = cfg.get("oak_cameras", {})
    usb_cameras = cfg.get("usb_cameras", {})

    logger.info(
        "Loaded configuration file: %d OAK, %d USB cameras.",
        len(oak_cameras),
        len(usb_cameras),
    )
    return oak_cameras, usb_cameras


def choose_pixel_format(device: str) -> str:
    logger = logging.getLogger("PixelFormat")

    try:
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats", "-d", device],
            capture_output=True, text=True, check=True, timeout=3
        )
        output = result.stdout.lower()

        if "mjpeg" in output or "mjpg" in output:
            return "mjpeg2rgb"
        elif "yuyv" in output:
            return "yuyv"
        else:
            logger.warning("%s: no common pixel formats found, defaulting to YUYV", device)
            return "yuyv"

    except Exception as e:
        logger.warning("%s: could not determine pixel format (%s) — defaulting to YUYV", device, e)
        return "yuyv"


def assign_names(cameras, config, prefix, id_attr="id", port_attr="port_path"):

    configured = config or {}
    name_slots = [f"{prefix}{i}" for i in range(10)]
    used_names = set()
    assigned = []

    # Split known vs unknown
    known, unknown = [], []
    for cam in cameras:
        cam_id = str(getattr(cam, id_attr))
        if cam_id in configured:
            known.append(cam)
        else:
            unknown.append(cam)

    # Assign known names (from YAML)
    for cam in known:
        cam_id = getattr(cam, id_attr)
        cfg = configured.get(cam_id, {})
        name = cfg.get("name", f"{prefix}_unknown_{cam_id}")
        cam.assigned_name = name
        cam.params = cfg.get("params", {})
        used_names.add(name)
        assigned.append(cam)

    # Sort unknown by port
    def port_sort(cam):
        port = str(getattr(cam, port_attr, ""))
        nums = re.findall(r"\d+", port)
        return tuple(map(int, nums)) if nums else (999,)

    unknown.sort(key=port_sort)
    available = [n for n in name_slots if n not in used_names]

    # Assign names to unknown devices
    for cam in unknown:
        if available:
            name = available.pop(0)
        else:
            name = f"{prefix}_extra_{getattr(cam, id_attr)}"
        cam.assigned_name = name
        cam.params = copy.deepcopy(DEFAULT_OAK_PARAMS if prefix == "oak" else DEFAULT_USB_PARAMS)
        assigned.append(cam)

    return assigned


