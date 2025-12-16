#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Some default values that will work as fallback if no other values is defined. Tested with different outcomes, always works but not optimized. 
DEFAULT_ENCODER_PARAMS = {
    "encoder.prefer_hevc": False,
    "encoder.quality": 4,
    "encoder.latency": "ultra_low",
    "encoder.bitrate_mode": "CRF",
    "encoder.bitrate": "12M",
    "encoder.maxrate": "12M",
    "encoder.bufsize": "24M",
    "encoder.crf": 23,
    "encoder.gop": 1,
    "encoder.bframes": 0,
    "encoder.mux": "mpegts",
    "encoder.mux_flags": "-flush_packets 1 -fflags nobuffer -max_delay 0 -muxdelay 0 -muxpreload 0",
}


def load_encoder_settings(node, defaults: dict) -> dict:
    for key, default_value in defaults.items():
        if not node.has_parameter(key):
            node.declare_parameter(key, default_value)

    # return settings without "encoder." prefix
    return {
        k.split("encoder.")[1]: node.get_parameter(k).value
        for k in defaults
    }
