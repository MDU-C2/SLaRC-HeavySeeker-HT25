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

        # --------------------------------------------------
        # Encoder auto-detection
        # --------------------------------------------------
        self.encoder_mgr = EncoderManager(self)

        # Camera configs from YAML / launch file
        self.camera_configs = load_camera_configs(self)

        # Default output mode
        self.output_mode = "mpegts"

        # --------------------------------------------------
        # Services
        # --------------------------------------------------
        self.create_service(
            SetOutputMode,
            "set_output_mode",
            self.handle_set_output_mode,
        )

        # --------------------------------------------------
        # Action handler (registry injected later)
        # --------------------------------------------------
        self.action_handler = CameraActionHandler(
            node=self,
            registry=None,
            encoder_info=self.encoder_mgr.info,
            camera_configs=self.camera_configs,
        )

        # --------------------------------------------------
        # Camera discovery + availability
        # --------------------------------------------------
        self.registry = CameraRegistry(
            node=self,
            encoder_info=self.encoder_mgr.info,
            on_disconnected=self.action_handler.handle_disconnected,
        )

        # --------------------------------------------------
        # ✅ REGISTER NETWORK CAMERAS (guarded)
        # --------------------------------------------------
        net_cams = self.camera_configs.get("h264_network_cameras", {})

        # Fallback: register any entries with type == h264_network
        if not net_cams:
            net_cams = {
                name: cfg for name, cfg in self.camera_configs.items()
                if isinstance(cfg, dict) and cfg.get("type") == "h264_network"
            }

        for cam_name, cfg in net_cams.items():
            if "url" not in cfg:
                self.get_logger().warn(
                    f"Network camera '{cam_name}' has no URL configured"
                )
                continue

            topic = f"/{cam_name}/encoded/h264"
            self.registry.register_network_camera(cam_name, topic)

            self.get_logger().info(
                f"Registered network camera '{cam_name}' ({topic})"
            )

        # Late-link registry into action handler
        self.action_handler.registry = self.registry

        # Auto-start listeners for configured network cameras so late-arriving streams are detected
        for cam_name in net_cams.keys():
            ok, msg, topic = self.action_handler.start_network_receiver(cam_name)
            if ok:
                self.get_logger().info(
                    f"Auto-started network camera '{cam_name}' ({topic})"
                )
            else:
                self.get_logger().warn(
                    f"Failed to auto-start network camera '{cam_name}': {msg}"
                )

        # --------------------------------------------------
        # Query service
        # --------------------------------------------------
        self.create_service(
            GetCameras,
            "get_available_cameras",
            self.handle_get_cameras,
        )

        self.get_logger().info("FPV Server initialized")

    # --------------------------------------------------
    def handle_get_cameras(self, request, response):
        # cameras = currently detected/heartbeat-active cameras
        # For network cameras, hide them when publishing is paused (stop command).
        active = []
        for cam in self.registry.active_cameras:
            rx = self.action_handler.network_receivers.get(cam)
            if rx and not rx.is_publishing:
                continue
            active.append(cam)
        response.cameras = active

        active_heartbeat = set(self.registry.active_cameras)
        active_encoders = {
            cam for cam, enc in self.action_handler.encoders.items()
            if enc.is_running()
        }
        # Only count network receivers as active when heartbeats/data are seen
        active_network = {
            cam for cam, rx in self.action_handler.network_receivers.items()
            if cam in active_heartbeat and rx.is_publishing
        }

        response.active_cameras = sorted(active_encoders | active_network)
        return response

    # --------------------------------------------------
    def handle_set_output_mode(self, request, response):
        mode = request.mode.lower()

        if mode not in ("mpegts", "foxglove", "headless"):
            response.success = False
            response.message = f"Invalid mode '{mode}'"
            return response

        self.output_mode = mode
        self.get_logger().info(f"Switched encoder output mode to: {mode}")

        # Forward mode change to all active encoders / receivers
        for enc in self.action_handler.encoders.values():
            enc.apply_output_mode(mode)

        for rx in self.action_handler.network_receivers.values():
            rx.apply_output_mode(mode)

        response.success = True
        response.message = f"Output mode set to {mode}"
        return response

    # --------------------------------------------------
    def destroy_node(self):
        self.action_handler.stop_all()
        super().destroy_node()
