import subprocess
import yaml
import os
import re
import logging
import copy

#-------------------------Configure logging -------------------------
logger = logging.getLogger("CameraConfig")

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


#--------------------Load camera configuration yaml file----------------------------
def load_camera_config(yaml_path: str):
    
    # Load camera configuration file and return dicts for OAK and USB cameras.
    if not os.path.exists(yaml_path):
        logger.error("Camera configuration file not found: %s", yaml_path)
        return {}, {}
    logger.info("Loading camera configuration from: %s", yaml_path)

    try:
        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        logger.error("Error in YAML file %s: %s", yaml_path, e)
        return {}, {}

    oak_cameras = cfg.get("oak_cameras", {})
    usb_cameras = cfg.get("usb_cameras", {})

    logger.info("Loaded configuration: %d OAK, %d USB cameras.",len(oak_cameras), len(usb_cameras))
    return oak_cameras, usb_cameras

#----------- Detect best pixel format-----------------------
# Can be used always but for now it is only used for unkown devices (devices not in the config file)
def choose_pixel_format(device: str) -> str:
    # Detects available formats provided by the device and returns the best format. Priority yuyv over mjpeg. Fallback to yuyv.
    logger.debug("Checking available pixel formats for %s", device)

    try:
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats", "-d", device],
            capture_output=True, text=True, check=True, timeout=3
        )
        output = result.stdout.lower()


        # We prefere mjpeg over yuyv, though if not avaialbel we will choose yuyv instead.
        if "mjpeg" in output or "mjpg" in output:
            logger.info("%s: selected pixel format = MJPEG", device)
            return "mjpeg2rgb"
        elif "yuyv" in output:
            logger.info("%s: selected pixel format = YUYV", device)
            return "yuyv"
        else:
            logger.warning("%s: no common pixel formats found, defaulting to YUYV", device)
            return "yuyv"

    except subprocess.TimeoutExpired:
        logger.warning("%s: timeout while listing formats — defaulting to YUYV", device)
        return "yuyv"
    except subprocess.CalledProcessError as e:
        logger.error("%s: v4l2-ctl failed (%s) — defaulting to YUYV", device, e)
        return "yuyv"
    except Exception as e:
        logger.error("%s: unexpected error while checking pixel format: %s", device, e)
        return "yuyv"
    


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