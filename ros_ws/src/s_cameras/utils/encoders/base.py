
from abc import ABC, abstractmethod
from dataclasses import dataclass

@dataclass
class EncoderInfo:
    ffmpeg_name: str
    description: str
    codec: str        # "h264" or "h265"
    hw_accel: bool

class EncoderBackend(ABC):
    """Abstract base class for all encoder backends."""
    requires = []     # List of callables returning True/False

    @classmethod
    def available(cls) -> bool:
        return all(check() for check in cls.requires)

    @abstractmethod
    def encoders(self) -> list[EncoderInfo]:
        pass

    @abstractmethod
    def map_settings(self, info: EncoderInfo, settings: dict):
        """
        Returns:
            (options_dict, ffmpeg_extra_args_list)
        """
        pass

def normalize_settings(s: dict) -> dict:
    q = max(1, min(int(s.get("quality", 7)), 7))
    return {
        "quality": q,
        "latency": s.get("latency", "ultra_low").lower(),
        "bitrate": s.get("bitrate", "8M"),
        "maxrate": s.get("maxrate", "10M"),
        "bufsize": s.get("bufsize", "16M"),
        "bitrate_mode": s.get("bitrate_mode", "VBR").upper(),
        "crf": str(s.get("crf", 23)),
        "gop": str(s.get("gop", 1)),
        "bframes": str(s.get("bframes", 0)),
    }

def common_extra_args(codec: str, c: dict):
    return [
        "-g", c["gop"],
        "-bf", c["bframes"],
        "-b:v", c["bitrate"],
        "-maxrate", c["maxrate"],
        "-bufsize", c["bufsize"],
    ]
