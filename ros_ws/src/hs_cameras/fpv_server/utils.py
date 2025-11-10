#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import os
import logging
import shutil

logger = logging.getLogger("fpv_utils")

# ------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------

def _has_cmd(cmd: str) -> bool:
    return shutil.which(cmd) is not None

def _has_gpu(keyword: str) -> bool:
    try:
        out = subprocess.getoutput("lspci | grep -i 'vga\\|3d'")
        return keyword.lower() in out.lower()
    except Exception:
        return False

# ------------------------------------------------------------------------------
# FFmpeg Encoder Support Cache & Test
# ------------------------------------------------------------------------------

_FFMPEG_ENCODERS_CACHE = None

def _ffmpeg_supports(name: str) -> bool:
    global _FFMPEG_ENCODERS_CACHE
    if _FFMPEG_ENCODERS_CACHE is None:
        try:
            _FFMPEG_ENCODERS_CACHE = subprocess.getoutput(
                "ffmpeg -hide_banner -encoders 2>/dev/null"
            )
        except Exception:
            _FFMPEG_ENCODERS_CACHE = ""
    return name in _FFMPEG_ENCODERS_CACHE

def _test_ffmpeg_encoder(name: str) -> bool:
    """Try to encode one test frame using the given encoder."""
    try:
        # HEVC NVENC requires at least 128x128
        test_size = "128x128" if "hevc_nvenc" in name else "64x64"

        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "lavfi", "-i", f"testsrc=size={test_size}:rate=1",
            "-frames:v", "1", "-c:v", name, "-f", "null", "-"
        ]
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        stderr = proc.stderr.decode(errors="ignore")

        if proc.returncode == 0 and not any(
            e in stderr.lower()
            for e in [
                "no such device",
                "initialization failed",
                "not supported",
                "failed to create",
                "driver not found",
                "could not load"
            ]
        ):
            return True

        logger.warning(f"Encoder test failed for {name}: {stderr.strip()[:200]}")
        return False

    except Exception as e:
        logger.warning(f"Encoder test exception for {name}: {e}")
        return False



# ------------------------------------------------------------------------------
# Unified Encoder Settings Mapper
# ------------------------------------------------------------------------------

def map_encoder_settings(name: str, codec: str, settings: dict) -> tuple[dict, list]:

    opts = {}
    extra = []

    # --- Universal defaults ---
    gop = str(settings.get("gop", 1))
    bframes = str(settings.get("bframes", 0))
    extra += ["-g", gop, "-bf", bframes]

    # Extract settings (with defaults)
    quality = int(settings.get("quality", 7))   # 1–7
    latency = settings.get("latency", "ultra_low").lower()
    bitrate_mode = settings.get("bitrate_mode", "VBR").upper()
    bitrate = settings.get("bitrate", "8M")
    maxrate = settings.get("maxrate", "10M")
    bufsize = settings.get("bufsize", "16M")
    crf = str(settings.get("crf", 23))

    quality = max(1, min(quality, 7))  # clamp

    # --- NVIDIA NVENC ---
    if "nvenc" in name:
        opts["preset"] = f"p{quality}"
        if quality >= 6 or latency in ("ultra_low", "low"):
            opts["tune"] = "ull"
            extra += ["-rc-lookahead", "0", "-no-scenecut", "1"]
        extra += [
            "-b:v", bitrate,
            "-maxrate", maxrate,
            "-bufsize", bufsize,
        ]
        if bitrate_mode in ("CBR", "VBR", "CQP"):
            extra += ["-rc", bitrate_mode.lower()]
        else:
            extra += ["-rc", "vbr"]

    # --- Intel Quick Sync (QSV) ---
    elif "qsv" in name:
        async_depth = str(max(1, 8 - quality))
        opts["async_depth"] = async_depth
        if quality >= 6 or latency in ("ultra_low", "low"):
            opts["low_power"] = "1"
            extra += ["-look_ahead", "0"]
        extra += [
            "-b:v", bitrate,
            "-maxrate", maxrate,
            "-bufsize", bufsize,
        ]

    # --- VAAPI (Intel/AMD unified) ---
    elif "vaapi" in name:
        qual_map = {
            1: "quality", 2: "quality", 3: "balanced",
            4: "normal", 5: "fast", 6: "speed", 7: "speed"
        }
        opts["quality"] = qual_map[quality]
        if quality >= 6 or latency in ("ultra_low", "low"):
            opts["low_power"] = "1"
        extra += [
            "-b:v", bitrate,
            "-maxrate", maxrate,
            "-bufsize", bufsize,
        ]

    # --- V4L2 (Raspberry Pi / ARM) ---
    elif "v4l2" in name:
        opts["num_output_buffers"] = "6"
        extra += [
            "-b:v", bitrate,
            "-maxrate", maxrate,
            "-bufsize", bufsize,
        ]

    # --- CPU Software Encoders (libx264 / libx265) ---
    elif "libx26" in name:
        preset_map = {
            1: "veryslow", 2: "slower", 3: "slow",
            4: "medium", 5: "fast", 6: "superfast", 7: "ultrafast"
        }
        opts["preset"] = preset_map[quality]
        opts["crf"] = crf
        if quality >= 6 or latency in ("ultra_low", "low"):
            opts["tune"] = "zerolatency"
        if codec == "HEVC":
            extra += ["-x265-params", "repeat-headers=1:annexb=1"]
        else:
            extra += ["-x264-params", "repeat-headers=1:annexb=1"]

    return opts, extra

# ------------------------------------------------------------------------------
# Encoder Detection
# ------------------------------------------------------------------------------

def detect_encoder(prefer_hevc: bool = True, settings: dict = None) -> dict:
    """Return the best available and working encoder configuration for this system."""
    if settings is None:
        settings = {}

    candidates = []

    def _add_candidate(name, desc, codec, hw):
        if not _ffmpeg_supports(name):
            return
        opts, extra = map_encoder_settings(name, codec, settings)

        # Normalize codec info
        if codec.lower() in ("hevc", "h265"):
            codec_name = "h265"
            container_name = "hevc"
            bitstream_filter = "hevc_mp4toannexb"  # <-- fixed here
        else:
            codec_name = "h264"
            container_name = "h264"
            bitstream_filter = "h264_mp4toannexb"

        mux_format = settings.get("mux", "mpegts")
        mux_flags_str = settings.get("mux_flags", "")
        mux_flags = mux_flags_str.split() if isinstance(mux_flags_str, str) else list(mux_flags_str)

        candidates.append({
            "name": name,
            "description": desc,
            "codec": codec_name,
            "container": container_name,
            "bitstream_filter": bitstream_filter,
            "hw_accel": hw,
            "options": opts,
            "extra_args": extra,
            "mux": mux_format,
            "mux_flags": mux_flags,
        })

    # --- GPU and encoder detection ---
    if os.path.exists("/dev/nvidia0") or _has_cmd("nvidia-smi"):
        _add_candidate("hevc_nvenc", "NVIDIA NVENC (H.265)", "h265", True)
        _add_candidate("h264_nvenc", "NVIDIA NVENC (H.264)", "h264", True)

    if os.path.exists("/dev/dri/renderD128") and _has_gpu("intel"):
        _add_candidate("hevc_qsv", "Intel Quick Sync (H.265)", "h265", True)
        _add_candidate("h264_qsv", "Intel Quick Sync (H.264)", "h264", True)

    if os.path.exists("/dev/dri/renderD128") and (_has_gpu("intel") or _has_gpu("amd")):
        _add_candidate("hevc_vaapi", "VAAPI (H.265)", "h265", True)
        _add_candidate("h264_vaapi", "VAAPI (H.264)", "h264", True)

    if _has_cmd("v4l2-ctl"):
        _add_candidate("hevc_v4l2m2m", "V4L2 M2M (H.265)", "h265", True)
        _add_candidate("h264_v4l2m2m", "V4L2 M2M (H.264)", "h264", True)

    # --- Always include software encoders ---
    _add_candidate("libx265", "CPU Software (H.265)", "h265", False)
    _add_candidate("libx264", "CPU Software (H.264)", "h264", False)
    
    if not prefer_hevc:
        # Move all H.264 encoders before H.265/HEVC ones
        candidates.sort(key=lambda e: 0 if e["codec"] == "h264" else 1)
    
    # --- Test candidates in order ---
    for enc in candidates:
        if _test_ffmpeg_encoder(enc["name"]):
            logger.info(f"✅ Using encoder: {enc['description']} ({enc['name']})")
            return enc

    # --- Fallback ---
    logger.error("❌ No working encoder found, falling back to libx264 (CPU).")
    opts, extra = map_encoder_settings("libx264", "h264", settings)
    return {
        "name": "libx264",
        "description": "CPU Software (H.264)",
        "codec": "h264",
        "container": "h264",
        "bitstream_filter": "h264_mp4toannexb",
        "hw_accel": False,
        "options": opts,
        "extra_args": extra,
        "mux": settings.get("mux", "mpegts"),
        "mux_flags": settings.get("mux_flags", "").split(),
    }
