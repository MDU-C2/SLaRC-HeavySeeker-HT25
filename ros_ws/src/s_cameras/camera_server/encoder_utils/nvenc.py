
import os, shutil
from .base import EncoderBackend, EncoderInfo, normalize_settings, common_extra_args

# Nividia encoder class
class NVENC(EncoderBackend):
    requires = [
        lambda: os.path.exists("/dev/nvidia0") or shutil.which("nvidia-smi") is not None
    ]
    def encoders(self):
        return [
            EncoderInfo("hevc_nvenc", "NVIDIA NVENC (H.265)", "h265", True),
            EncoderInfo("h264_nvenc", "NVIDIA NVENC (H.264)", "h264", True),
        ]

    def map_settings(self, info, settings):
        c = normalize_settings(settings)
        opts = {
            "preset": f"p{c['quality']}",
        }
        extra = common_extra_args(info.codec, c)

        # low latency
        if c["quality"] >= 6 or c["latency"] in ("ultra_low", "low"):
            opts["tune"] = "ull"
            extra += ["-rc-lookahead", "0", "-no-scenecut", "1"]

        rc_mode = c["bitrate_mode"].lower()
        extra += ["-rc", rc_mode if rc_mode in ("vbr", "cbr", "cqp") else "vbr"]

        return opts, extra
