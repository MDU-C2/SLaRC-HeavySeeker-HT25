
import os
from .base import EncoderBackend, EncoderInfo, normalize_settings, common_extra_args

class QSV(EncoderBackend):
    requires = [
        lambda: os.path.exists("/dev/dri/renderD128")
    ]

    def encoders(self):
        return [
            EncoderInfo("hevc_qsv", "Intel QSV (H.265)", "h265", True),
            EncoderInfo("h264_qsv", "Intel QSV (H.264)", "h264", True),
        ]

    def map_settings(self, info, settings):
        c = normalize_settings(settings)
        opts = {
            "async_depth": str(max(1, 8 - c["quality"]))
        }
        extra = common_extra_args(info.codec, c)

        if c["quality"] >= 6 or c["latency"] in ("ultra_low", "low"):
            opts["low_power"] = "1"
            extra += ["-look_ahead", "0"]

        return opts, extra
