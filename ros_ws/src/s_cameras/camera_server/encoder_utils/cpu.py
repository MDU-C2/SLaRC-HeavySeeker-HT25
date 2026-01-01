
from .base import EncoderBackend, EncoderInfo, normalize_settings, common_extra_args

CPU_PRESETS = {
    1: "veryslow", 2: "slower", 3: "slow",
    4: "medium",  5: "fast",   6: "superfast", 7: "ultrafast"
}

# This is the fallback section. If we havent installed the GPU hardware drivers we fallback to this. 
# Hence the program will allways work but encoding will be done on the CPU.
# CPU endcoding is very heavy and should be avoided (tests showed ~47% CPU usage only for encoding)
class CPU(EncoderBackend):

    def encoders(self):
        return [
            EncoderInfo("libx265", "CPU (H.265)", "h265", False),
            EncoderInfo("libx264", "CPU (H.264)", "h264", False),
        ]

    def map_settings(self, info, settings):
        c = normalize_settings(settings)
        opts = {
            "preset": CPU_PRESETS[c["quality"]],
            "crf": c["crf"]
        }
        if c["quality"] >= 6:
            opts["tune"] = "zerolatency"

        extra = common_extra_args(info.codec, c)
        extra += [f"-x{info.codec[1:]}-params", "repeat-headers=1:annexb=1"]

        return opts, extra
