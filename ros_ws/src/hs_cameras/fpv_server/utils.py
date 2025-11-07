import subprocess
import os
import logging
import shutil


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
# 3. Then we always prefer H.265(hevc) if available, otherwise we choos H.264.
# 4. If no encoder was detected we fallback to CPU, though this can become very heavy on the CPU as it is not designed for grafics.
def detect_encoder(prefer_hevc=True):
    """Detect best available encoder and return its configuration, including optimal options."""
    
    # --- NVIDIA GPUs (NVENC) ---
    if os.path.exists('/dev/nvidia0') or _has_cmd("nvidia-smi"):
        if prefer_hevc and _ffmpeg_supports("hevc_nvenc"):
            return {
                "name": "hevc_nvenc",
                "description": "NVIDIA NVENC (H.265)",
                "codec": "H265",
                "options": {"preset": "p6", "tune": "ull"}  # Ultra low latency preset
            }
        return {
            "name": "h264_nvenc",
            "description": "NVIDIA NVENC (H.264)",
            "codec": "H264",
            "options": {"preset": "p6", "tune": "ull"}
        }

    # --- Intel Quick Sync (QSV) ---
    if os.path.exists('/dev/dri/renderD128') and _has_gpu("intel"):
        if prefer_hevc and _ffmpeg_supports("hevc_qsv"):
            return {
                "name": "hevc_qsv",
                "description": "Intel Quick Sync (H.265)",
                "codec": "H265",
                "options": {"load_plugin": "hevc_hw"}  # Light optimization
            }
        return {
            "name": "h264_qsv",
            "description": "Intel Quick Sync (H.264)",
            "codec": "H264",
            "options": {"load_plugin": "h264_hw"}
        }

    # --- AMD (AMF) ---
    if _has_gpu("amd") and _ffmpeg_supports("h264_amf"):
        if prefer_hevc and _ffmpeg_supports("hevc_amf"):
            return {
                "name": "hevc_amf",
                "description": "AMD AMF (H.265)",
                "codec": "H265",
                "options": {"usage": "ultralowlatency"}
            }
        return {
            "name": "h264_amf",
            "description": "AMD AMF (H.264)",
            "codec": "H264",
            "options": {"usage": "ultralowlatency"}
        }

    # --- VAAPI (Intel/AMD open driver path) ---
    if os.path.exists('/dev/dri/renderD128') and _ffmpeg_supports("h264_vaapi"):
        if prefer_hevc and _ffmpeg_supports("hevc_vaapi"):
            return {
                "name": "hevc_vaapi",
                "description": "VAAPI (H.265)",
                "codec": "H265",
                "options": {"quality": "speed"}
            }
        return {
            "name": "h264_vaapi",
            "description": "VAAPI (H.264)",
            "codec": "H264",
            "options": {"quality": "speed"}
        }

    # --- Raspberry Pi / ARM V4L2 ---
    if _has_cmd("v4l2-ctl") and _ffmpeg_supports("h264_v4l2m2m"):
        if prefer_hevc and _ffmpeg_supports("hevc_v4l2m2m"):
            return {
                "name": "hevc_v4l2m2m",
                "description": "V4L2M2M (H.265)",
                "codec": "H265",
                "options": {"num_output_buffers": "6"}
            }
        return {
            "name": "h264_v4l2m2m",
            "description": "V4L2M2M (H.264)",
            "codec": "H264",
            "options": {"num_output_buffers": "6"}
        }

    # --- CPU fallback ---
    if prefer_hevc and _ffmpeg_supports("libx265"):
        return {
            "name": "libx265",
            "description": "CPU Software (H.265)",
            "codec": "H265",
            "options": {"preset": "ultrafast", "tune": "zerolatency"}
        }
    return {
        "name": "libx264",
        "description": "CPU Software (H.264)",
        "codec": "H264",
        "options": {"preset": "ultrafast", "tune": "zerolatency"}
    }
