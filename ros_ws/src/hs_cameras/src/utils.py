import subprocess
import yaml
import os
import re
import logging
import copy
import shutil

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
        "i_resolution": '1080P',
        "i_width": 1280,
    },
    "use_sim_time": False,
}


DEFAULT_USB_PARAMS = {
    "image_width": 640,
    "image_height": 480,
    "pixel_format": 'mjpeg2rgb', 
    "auto_white_balance": False,
    "autoexposure": False,
    "auto_focus": False,
    "framerate": 15.0,
}


#=================== Load camera configuration yaml file =====================
# Load camera configuration file and return dicts for OAK and USB cameras.
def load_camera_config(yaml_path: str):

    logger = logging.getLogger("CameraConfig")
    
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




#===================== Choose pixel format =========================
# Only used if the camera is not in the cameras.yaml.
# Detects available formats provided by the device and returns the best format. Priority yuyv over mjpeg. Fallback to yuyv.
def choose_pixel_format(device: str) -> str:
    
    logger = logging.getLogger("PixelFormat")
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
    

#======================= Assing camera names ====================================
# Assign names deterministically based on config + ports (if not in cameras.yaml, then sort by port number).

logger = logging.getLogger("CameraNaming")


def assign_names(detected, config, prefix, port_key="port_path", id_key="mxid"):
    logger = logging.getLogger("AssignNames")
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
        logger.debug("Assigned known camera %s → %s", name, cam_id)

    # Sort unknown by port number (ascending order)
    def port_sort(cam):
        port = str(cam.get(port_key, ""))
        nums = re.findall(r"\d+", port)
        return tuple(map(int, nums)) if nums else (999,)
    unknown.sort(key=port_sort)

    # Determine available names (only used by connected configured cameras)
    available = [n for n in name_slots if n not in used_names]
    logger.debug("Available name slots: %s", available)

    # Assign names to unknown devices
    for cam in unknown:
        if available:
            name = available.pop(0)
        else:
            name = f"{prefix}_extra_{cam[id_key]}"
        cam["assigned_name"] = name
        cam["params"] = copy.deepcopy(
            DEFAULT_OAK_PARAMS if prefix == "oak" else DEFAULT_USB_PARAMS
        )
        assigned.append(cam)
        logger.debug("Assigning new name %s to device %s", name, cam.get(id_key))

    return assigned


# ---------------------------------------------------------------------------------
# ----------------------- Image encoder utils -------------------------------------
# ---------------------------------------------------------------------------------

logger = logging.getLogger("image_encoding")

# ======================= Encoder helpers =========================================

# Check if a given command-line tool is available on the system PATH.
def _has_cmd(cmd):
    return shutil.which(cmd) is not None

# Check if gpu is present in the system. (Intel, NVIDIA, AMD, ..etc )
def _has_gpu(keyword):
    out = os.popen("lspci | grep -i 'vga\\|3d'").read().lower()
    return keyword.lower() in out

# Check if the systems ffmpeg supports a specific encoder.
_FFMPEG_ENCODERS_CACHE = None

def _ffmpeg_supports(encoder_name):
    global _FFMPEG_ENCODERS_CACHE
    if _FFMPEG_ENCODERS_CACHE is None:
        try:
            _FFMPEG_ENCODERS_CACHE = subprocess.getoutput("ffmpeg -hide_banner -encoders")
        except Exception:
            _FFMPEG_ENCODERS_CACHE = ""
    return encoder_name in _FFMPEG_ENCODERS_CACHE

#========================= Detect available encoder ===============================
# 1. First we search the system for the most common GPUs:
#    NVIDIA, Intel, AMD, VAAPI, Rasberry-Pi
# 2. Then for the detected GPU we check if the ffmpeg support a specifi encoder.
# 3. Then we always choose H.265 if available, otherwise we choos H.264.
# 4. If no encoder was detected we fallback to CPU, though this can become very heavy on the CPU as it is not designed for grafics.
def detect_encoder(prefer_hevc=True):
    
    def make_encoder(encoder, desc):
        return {
            "name": encoder,
            "description": desc,
            "hardware_accelerated": not encoder.startswith("libx"),
        }
    
    # --- NVIDIA GPUs ---
    if os.path.exists('/dev/nvidia0') or _has_cmd("nvidia-smi"):
        if prefer_hevc and _ffmpeg_supports("hevc_nvenc"):
            return make_encoder("hevc_nvenc", "NVIDIA NVENC (H.265)")
        return make_encoder("h264_nvenc", "NVIDIA NVENC (H.264)")

    # --- Intel Quick Sync ---
    if os.path.exists('/dev/dri/renderD128') and _has_gpu("intel"):
        if prefer_hevc and _ffmpeg_supports("hevc_qsv"):
            return make_encoder("hevc_qsv", "Intel Quick Sync (H.265)")
        return make_encoder("h264_qsv", "Intel Quick Sync (H.264)")

    # --- AMD GPUs (AMF) ---
    if _has_gpu("amd") and _ffmpeg_supports("h264_amf"):
        if prefer_hevc and _ffmpeg_supports("hevc_amf"):
            return make_encoder("hevc_amf", "AMD AMF (H.265)")
        return make_encoder("h264_amf", "AMD AMF (H.264)")

    # --- VAAPI (Generic GPU acceleration) ---
    if os.path.exists('/dev/dri/renderD128') and _ffmpeg_supports("h264_vaapi"):
        if prefer_hevc and _ffmpeg_supports("hevc_vaapi"):
            return make_encoder("hevc_vaapi", "VAAPI (H.265)")
        return make_encoder("h264_vaapi", "VAAPI (H.264)")

    # --- Raspberry Pi / ARM (V4L2M2M) ---
    if _has_cmd("v4l2-ctl") and _ffmpeg_supports("h264_v4l2m2m"):
        if prefer_hevc and _ffmpeg_supports("hevc_v4l2m2m"):
            return make_encoder("hevc_v4l2m2m", "V4L2M2M (H.265)")
        return make_encoder("h264_v4l2m2m", "V4L2M2M (H.264)")

    # --- CPU fallback ---
    if prefer_hevc and _ffmpeg_supports("libx265"):
        return make_encoder("libx265", "CPU Software (H.265)")
    return make_encoder("libx264", "CPU Software (H.264)")
