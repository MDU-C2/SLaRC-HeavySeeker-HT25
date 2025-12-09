# utils/camera_config.py
import subprocess
import yaml
import os
import logging

logger = logging.getLogger("CameraConfig")

# ---------------------------------------------------------------------------
# Default parameter templates
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Load YAML camera config
# ---------------------------------------------------------------------------

def load_camera_config(yaml_path: str):

    if not os.path.exists(yaml_path):
        logger.warning("Camera configuration file not found: %s", yaml_path)
        return {}, {}

    try:
        with open(yaml_path, "r") as f:
            cfg = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        logger.error("Error parsing YAML: %s", e)
        return {}, {}

    oak = cfg.get("oak_cameras", {})
    usb = cfg.get("usb_cameras", {})

    logger.info("Loaded config: %d OAK, %d USB cameras", len(oak), len(usb))
    return oak, usb


# ---------------------------------------------------------------------------
# Pixel format selection
# ---------------------------------------------------------------------------

def choose_pixel_format(device: str) -> str:

    logger = logging.getLogger("PixelFormat")

    try:
        proc = subprocess.run(
            ["v4l2-ctl", "--list-formats", "-d", device],
            capture_output=True, text=True, check=True, timeout=3
        )
        out = proc.stdout.lower()

        if "mjpeg" in out or "mjpg" in out:
            return "mjpeg2rgb"
        if "yuyv" in out:
            return "yuyv"

        logger.warning("%s: no common pixel formats found, defaulting to YUYV", device)
        return "yuyv"

    except Exception as e:
        logger.warning("%s: pixel format detection failed (%s), defaulting YUYV", device, e)
        return "yuyv"
