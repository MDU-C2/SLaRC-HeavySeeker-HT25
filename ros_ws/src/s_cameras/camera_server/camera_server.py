#!/usr/bin/env python3
from rclpy.node import Node

from s_msgs.srv import GetCameras, SetOutputMode

from .encoder_manager import EncoderManager
from .camera_registry import CameraRegistry
from .action_handler import CameraActionHandler
from .server_utils import load_camera_configs


class CameraServer(Node):

    def __init__(self):
        super().__init__("camera_server")

        # Encoder auto-detection
        self.encoder_mgr = EncoderManager(self)

        # Camera configs from YAML / launch file
        self.camera_configs = load_camera_configs(self)
        
        # Default output mode
        self.output_mode = "mpegts"

        # Service for switching stream output mode
        self.create_service(
            SetOutputMode,
            "set_output_mode",
            self.handle_set_output_mode,
        )

        # Action handler (registry injected later)
        self.action_handler = CameraActionHandler(
            node=self,
            registry=None,
            encoder_info=self.encoder_mgr.info,
            camera_configs=self.camera_configs,
        )

        # Camera discovery + availability
        self.registry = CameraRegistry(
            node=self,
            encoder_info=self.encoder_mgr.info,
            on_disconnected=self.action_handler.handle_disconnected,
        )

        # Late-link registry into action handler
        self.action_handler.registry = self.registry

        # Service: query camera list
        self.create_service(
            GetCameras,
            "get_available_cameras",
            self.handle_get_cameras,
        )

        self.get_logger().info("FPV Server initialized")

    def handle_get_cameras(self, request, response):
        response.cameras = list(self.registry.active_cameras)
        response.active_cameras = [
            cam for cam, enc in self.action_handler.encoders.items()
            if enc.is_running()
        ]
        return response
    def handle_set_output_mode(self, request, response):
        mode = request.mode.lower()

        if mode not in ("mpegts", "foxglove", "headless"):
            response.success = False
            response.message = f"Invalid mode '{mode}'"
            return response

        self.output_mode = mode
        self.get_logger().info(f"Switched encoder output mode to: {mode}")

        # Forward mode change to all active encoders
        for cam, enc in self.action_handler.encoders.items():
            enc.apply_output_mode(mode)

        response.success = True
        response.message = f"Output mode set to {mode}"
        return response


    def destroy_node(self):
        self.action_handler.stop_all()
        super().destroy_node()
