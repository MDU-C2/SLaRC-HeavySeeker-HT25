from rclpy.node import Node
from .encoder_utils import (
    DEFAULT_ENCODER_PARAMS,
    load_encoder_settings,
    EncoderDetector,
)

class EncoderManager:

    def __init__(self, node: Node):
        self.node = node

        # Collect parameters from ROS2 + defaults
        self.settings = load_encoder_settings(node, DEFAULT_ENCODER_PARAMS)

        prefer_hevc = self.settings["prefer_hevc"]

        detector = EncoderDetector(prefer_hevc=prefer_hevc)
        self.info = detector.detect(self.settings)
        self.info["settings"] = self.settings

        node.get_logger().info(
            f"Selected encoder: {self.info['description']} "
            f"({self.info['name']}) codec={self.info['codec']} "
            f"hw_accel={self.info['hw_accel']}"
        )
