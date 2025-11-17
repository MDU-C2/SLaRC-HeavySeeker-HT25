
import os
from .base import EncoderBackend, EncoderInfo, normalize_settings, common_extra_args

class VAAPI(EncoderBackend):
    requires = [
        lambda: os.path.exists("/dev/dri/renderD128")
    ]

    def encoders(self):
        return [
            EncoderInfo("hevc_vaapi", "VAAPI (H.265)", "h265", True),
            EncoderInfo("h264_vaapi", "VAAPI (H.264)", "h264", True),
        ]

    def map_settings(self, info, settings):
        c = normalize_settings(settings)
        opts = {
            "quality": str(max(1, min(8, c["quality"])))
        }
        extra = common_extra_args(info.codec, c)

        if c["quality"] >= 6:
            opts["low_power"] = "1"

        return opts, extra
