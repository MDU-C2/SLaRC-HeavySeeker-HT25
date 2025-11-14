
from .cpu import CPU
from .nvenc import NVENC
from .qsv import QSV
from .vaapi import VAAPI
from .ffmpeg_probe import FFmpegProbe

CODEC_OUTPUT = {
    "h264": {"container": "h264", "bsf": "h264_mp4toannexb"},
    "h265": {"container": "hevc", "bsf": "hevc_mp4toannexb"},
}

class EncoderDetector:

    def __init__(self, prefer_hevc=True):
        self.backends = [NVENC(), QSV(), VAAPI(), CPU()]
        self.probe = FFmpegProbe()
        self.prefer_hevc = prefer_hevc

    def candidates(self):
        out = []
        for b in self.backends:
            if b.available():
                for info in b.encoders():
                    out.append((b, info))
        return out

    def sort(self, items):
        if not self.prefer_hevc:
            return sorted(items, key=lambda p: 0 if p[1].codec == "h264" else 1)
        return items

    def detect(self, settings):
        for backend, info in self.sort(self.candidates()):
            if self.probe.is_usable(info.ffmpeg_name):
                opts, extra = backend.map_settings(info, settings)
                c = CODEC_OUTPUT[info.codec]
                return {
                    "name": info.ffmpeg_name,
                    "description": info.description,
                    "codec": info.codec,
                    "hw_accel": info.hw_accel,
                    "container": c["container"],
                    "bitstream_filter": c["bsf"],
                    "options": opts,
                    "extra_args": extra,
                }

        # fallback
        cpu = CPU()
        info = cpu.encoders()[1]  # libx264
        opts, extra = cpu.map_settings(info, settings)
        c = CODEC_OUTPUT["h264"]
        return {
            "name": info.ffmpeg_name,
            "description": info.description,
            "codec": "h264",
            "hw_accel": False,
            "container": c["container"],
            "bitstream_filter": c["bsf"],
            "options": opts,
            "extra_args": extra,
        }
