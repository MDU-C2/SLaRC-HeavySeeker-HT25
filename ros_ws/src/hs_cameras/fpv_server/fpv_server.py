#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import json
import re
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from sensor_msgs.msg import Image
from std_msgs.msg import String

from fpv_server.utils import detect_encoder
from fpv_server.encoder import CameraEncoder

from hs_cameras.action import StopCamera   # type: ignore
from hs_cameras.action import StartCamera  # type: ignore
from hs_cameras.srv import GetCameras      # type: ignore


# --------------------------------------------------------------------
# 🎞️ Encoder Manager
# --------------------------------------------------------------------
class EncoderManager:
    """Handles encoder detection and parameter configuration."""

    DEFAULTS = {
            "encoder.prefer_hevc": True,

            # Encoder quality preset (speed vs. quality trade-off)
            # Typical range: 1 (best quality, slowest) → 7 (fastest, lowest quality)
            "encoder.quality": 6,

            # Latency mode: affects buffering, frame reordering, and lookahead
            # "ultra_low"  → no B-frames, smallest delay (for FPV, robotics)
            # "low"        → small delay, better efficiency (for live streaming)
            # "medium"     → moderate latency, best compression (for recording)
            "encoder.latency": "ultra_low",

            # Bitrate control mode
            # "CBR" → Constant Bitrate (stable, predictable latency)
            # "VBR" → Variable Bitrate (better quality but may vary)
            # "CRF" → Constant Rate Factor (target visual quality, bitrate floats)
            "encoder.bitrate_mode": "CRF",

            # Target video bitrate (e.g., "4M", "8M", "12M", "20M")
            # Use higher for higher resolutions or motion scenes
            "encoder.bitrate": "12M",

            # Maximum bitrate allowed for encoder in VBR or CBR mode
            # Typically set equal or slightly above encoder.bitrate
            "encoder.maxrate": "12M",

            # Encoder buffer size (controls bitrate smoothing)
            # Usually 2×–3× the bitrate value, e.g. 24M for 12M bitrate
            "encoder.bufsize": "24M",

            # Constant Rate Factor (only used in CRF mode)
            # Range: 0–51 → lower = better quality
            # 18–20 = visually lossless, 23 = default medium, 28+ = low quality
            "encoder.crf": 23,

            # GOP length = distance between I-frames (keyframes)
            # 1 = every frame is I-frame (no inter prediction, lowest latency)
            # 10–30 = recommended for low-latency streaming
            # 60–120 = typical for recording / non-realtime
            "encoder.gop": 1,

            # Number of B-frames (bidirectional frames) per GOP
            # 0 = disable (lowest latency)
            # 1–3 = higher compression efficiency (small latency penalty)
            # Only valid if latency != "ultra_low"
            "encoder.bframes": 0,

            # Output container / muxer format
            # "mpegts" = transport stream (common for real-time streaming)
            # "mp4"    = good for saved files
            # "matroska" or "mkv" = flexible container
            "encoder.mux": "mpegts",

            # Muxer flags: control packet buffering & output timing
            # -flush_packets 1 → flush packets immediately
            # -fflags nobuffer → minimize input buffering (ultra-low latency)
            # -max_delay 0     → zero mux buffer delay
            # -muxdelay        → how long muxer waits before writing packets (sec)
            # -muxpreload      → initial delay for first packets (sec)
            #
            # For "ultra_low" latency, keep everything 0
            # For "low" latency, use: "-flush_packets 1 -max_delay 0 -muxdelay 0.1 -muxpreload 0.1"
            "encoder.mux_flags": "-flush_packets 1 -fflags nobuffer -max_delay 0 -muxdelay 0 -muxpreload 0",
        }

    

    def __init__(self, node: Node):
        self.node = node
        for k, v in self.DEFAULTS.items():
            node.declare_parameter(k, v)

        self.settings = {
            k.split("encoder.")[1]: node.get_parameter(k).value
            for k in self.DEFAULTS
        }

        prefer_hevc = node.get_parameter("encoder.prefer_hevc").value
        self.info = detect_encoder(prefer_hevc=prefer_hevc, settings=self.settings)
        self.info["settings"] = self.settings

        node.get_logger().info(
            f"Selected encoder: {self.info['description']} ({self.info['name']}) "
            f"codec={self.info['codec']} hw_accel={self.info['hw_accel']}"
        )


# --------------------------------------------------------------------
# 📸 Camera Registry
# --------------------------------------------------------------------

class CameraRegistry:
    """Tracks and publishes available camera topics."""


    def __init__(self, node: Node, encoder_info: dict, on_disconnected=None):
        self.node = node
        self.encoder_info = encoder_info
        self.cameras: Dict[str, float] = {}
        self.active_cameras: List[str] = []
        self.on_disconnected = on_disconnected  # <--- NEW callback hook

        self.publisher = node.create_publisher(String, "/available_cameras", 10)
        node.create_timer(1.0, self._check_activity)
        node.create_timer(3.0, self._refresh_topics)
    def _refresh_topics(self):
        topics = self.node.get_topic_names_and_types()
        seen = {name.strip("/").split("/")[0] for name, _ in topics if re.search(r"/image_raw$", name)}

        # New cameras
        for cam in seen - self.cameras.keys():
            self.cameras[cam] = time.time()
            self.node.create_subscription(
                Image,
                f"/{cam}/image_raw",
                lambda msg, n=cam: self._on_image(n),
                10,
            )
            self.node.get_logger().info(f"Camera {cam} discovered ✅")

        # Removed cameras
        for cam in list(self.cameras.keys()):
            if cam not in seen:
                del self.cameras[cam]
                self.node.get_logger().info(f"Camera {cam} removed ❌")

        self._publish_active()

    def _on_image(self, cam: str):
        self.cameras[cam] = time.time()

    def _check_activity(self):
        self._publish_active()

    def _publish_active(self):
        now = time.time()
        active = sorted([n for n, t in self.cameras.items() if now - t <= 4.0])

        # Detect lost cameras (previously active but now gone)
        disconnected = [c for c in self.active_cameras if c not in active]

        if active != self.active_cameras:
            self.active_cameras = active
            msg = String()
            msg.data = json.dumps({
                "available_cameras": active,
                "encoder": self.encoder_info,
            })
            self.publisher.publish(msg)
            self.node.get_logger().info(f"Updated available cameras: {active}")

            # Notify handler about disconnected cameras
            if disconnected and self.on_disconnected:
                self.node.get_logger().warn(f"Cameras disconnected: {disconnected}")
                self.on_disconnected(disconnected)


# --------------------------------------------------------------------
# 🎬 Camera Action Handler (Start + Stop)
# --------------------------------------------------------------------
class CameraActionHandler:
    """Encapsulates ROS 2 ActionServer logic for starting and stopping camera encoders."""

    def __init__(self, node: Node, registry: CameraRegistry, encoder_info: dict, camera_configs: dict):
        self.node = node
        self.registry = registry
        self.encoder_info = encoder_info
        self.camera_configs = camera_configs
        self.encoders: Dict[str, CameraEncoder] = {}

        # Action servers for Start and Stop
        self.start_server = ActionServer(
            node,
            StartCamera,
            "start_camera_encoding",
            execute_callback=self.start_execute_callback,
            goal_callback=self.start_goal_callback,
            cancel_callback=self.start_cancel_callback,
        )

        self.stop_server = ActionServer(
            node,
            StopCamera,
            "stop_camera_encoding",
            execute_callback=self.stop_execute_callback,
            goal_callback=self.stop_goal_callback,
        )

    # ===============================================================
    # 🎬 START CAMERA ACTION
    # ===============================================================
    def start_goal_callback(self, goal_request):
        cam = goal_request.camera_name
        if cam not in self.registry.active_cameras:
            self.node.get_logger().warn(f"Camera '{cam}' not active → rejecting")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def start_cancel_callback(self, goal_handle):
        cam = goal_handle.request.camera_name
        if cam in self.encoders:
            self.encoders[cam].stop()
            self.encoders.pop(cam)
            self.node.get_logger().info(f"Encoder for {cam} stopped on cancel.")
        return CancelResponse.ACCEPT
    
    def handle_disconnected(self, lost_cameras: List[str]):
        """Called by CameraRegistry when one or more cameras go offline."""
        for cam in lost_cameras:
            if cam in self.encoders:
                try:
                    self.encoders[cam].stop()
                    self.encoders.pop(cam, None)
                    self.node.get_logger().warn(f"🛑 Camera '{cam}' disconnected — encoder stopped.")
                except Exception as e:
                    self.node.get_logger().error(f"❌ Failed to stop encoder for '{cam}' on disconnect: {e}")

    async def start_execute_callback(self, goal_handle):
        cam = goal_handle.request.camera_name
        fb = StartCamera.Feedback()

        if cam not in self.registry.active_cameras:
            fb.status = f"Camera '{cam}' not active — cannot start encoder."
            goal_handle.publish_feedback(fb)
            goal_handle.abort()
            return self._start_result(False, fb.status, "")

        if cam in self.encoders and self.encoders[cam].is_running():
            fb.status = f"Encoder for '{cam}' already running."
            goal_handle.publish_feedback(fb)
            goal_handle.succeed()
            return self._start_result(True, fb.status, f"/{cam}/encoded/{self.encoder_info['codec'].lower()}")

        fb.status = f"Starting encoder for {cam}"
        goal_handle.publish_feedback(fb)

        codec = self.encoder_info["codec"].lower()
        encoded_topic = f"/{cam}/encoded/{codec}"
        fps = float(self.camera_configs.get(cam, {}).get("params", {}).get("framerate", 30.0))
        self.node.get_logger().info(f"[{cam}] Using framerate: {fps:.1f} FPS")

        encoder = CameraEncoder(
            node=self.node,
            camera_name=cam,
            encoder_info=self.encoder_info,
            fps=fps,
            output_topic=encoded_topic,
        )

        try:
            encoder.start()
            self.encoders[cam] = encoder
            fb.status = f"Encoder for {cam} started on topic {encoded_topic}"
            goal_handle.publish_feedback(fb)
        except Exception as e:
            msg = f"Failed to start encoder for {cam}: {e}"
            goal_handle.publish_feedback(StartCamera.Feedback(status=msg))
            goal_handle.abort()
            return self._start_result(False, msg, "")

        goal_handle.succeed()
        return self._start_result(True, f"Encoder started for {cam}", encoded_topic)

    def _start_result(self, success: bool, message: str, topic: str):
        result = StartCamera.Result()
        result.success = success
        result.message = message
        result.topic = topic
        return result

    # ===============================================================
    # 🛑 STOP CAMERA ACTION
    # ===============================================================
    def stop_goal_callback(self, goal_request):
        cam = goal_request.camera_name
        if cam not in self.encoders:
            self.node.get_logger().warn(f"No encoder running for '{cam}' → rejecting stop request.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def stop_execute_callback(self, goal_handle):
        cam = goal_handle.request.camera_name
        result = StopCamera.Result()

        if cam not in self.encoders:
            msg = f"No encoder running for '{cam}'."
            result.success = False
            result.message = msg
            goal_handle.abort()
            self.node.get_logger().warn(msg)
            return result

        encoder = self.encoders.pop(cam)
        try:
            encoder.stop()
            msg = f"Encoder for '{cam}' stopped successfully."
            result.success = True
            result.message = msg
            goal_handle.succeed()
            self.node.get_logger().info(msg)
        except Exception as e:
            msg = f"Failed to stop encoder for '{cam}': {e}"
            result.success = False
            result.message = msg
            goal_handle.abort()
            self.node.get_logger().error(msg)

        return result

    # ===============================================================
    # 🧹 Utility
    # ===============================================================
    def stop_all(self):
        for enc in self.encoders.values():
            enc.stop()


# --------------------------------------------------------------------
# 🧠 FPV Server (composition root)
# --------------------------------------------------------------------
class FPVServer(Node):
    def __init__(self):
        super().__init__("fpv_server")

        # 1️⃣ Encoder setup
        self.encoder_mgr = EncoderManager(self)

        # 2️⃣ Camera configs
        self.camera_configs = self._load_camera_configs()

        # 3️⃣ Registry and handlers
        self.action_handler = CameraActionHandler(self, None, self.encoder_mgr.info, self.camera_configs)
        self.registry = CameraRegistry(self, self.encoder_mgr.info, on_disconnected=self.action_handler.handle_disconnected)
        self.action_handler.registry = self.registry  # late bind after registry created


        # 4️⃣ Service
        self.create_service(GetCameras, "get_available_cameras", self.handle_get_cameras)

        self.get_logger().info("FPV Server initialized ✅")

    def _load_camera_configs(self) -> Dict[str, dict]:
        self.declare_parameter("cameras_json", "")
        raw = self.get_parameter("cameras_json").value
        if not raw:
            self.get_logger().warn("No cameras_json parameter provided.")
            return {}
        try:
            data = json.loads(raw)
            self.get_logger().info(f"Loaded {len(data)} camera configurations.")
            return {c["name"]: c for c in data}
        except Exception as e:
            self.get_logger().error(f"Failed to parse cameras_json: {e}")
            return {}

    def handle_get_cameras(self, request, response):
        response.cameras = list(self.registry.active_cameras)
        response.active_cameras = [
            name for name, enc in self.action_handler.encoders.items() if enc.is_running()
        ]
        self.get_logger().info(f"Client requested camera list → {response.cameras}")
        return response

    def destroy_node(self):
        self.action_handler.stop_all()
        super().destroy_node()


# --------------------------------------------------------------------
# 🚀 Main entrypoint
# --------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FPVServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("FPV Server interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
