import subprocess
import yaml
import os
import logging
import os

#-------------------------Configure logging -------------------------
logger = logging.getLogger("CameraConfig")


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
def choose_best_pixel_format(device: str) -> str:
    # Detects available formats provided by the device and returns the best format. Priority yuyv over mjpeg. Fallback to yuyv.
    logger.debug("Checking available pixel formats for %s", device)

    try:
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats", "-d", device],
            capture_output=True, text=True, check=True, timeout=3
        )
        output = result.stdout.lower()


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
    


