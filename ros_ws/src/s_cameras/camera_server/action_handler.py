from rclpy.action import ActionServer, GoalResponse, CancelResponse
from s_msgs.action import StartCamera, StopCamera

from camera_server.camera_encoder import CameraEncoder
from camera_server.network_ts_receiver import NetworkTSReceiver


class CameraActionHandler:

    def __init__(self, node, registry, encoder_info, camera_configs):
        self.node = node
        self.registry = registry
        self.encoder_info = encoder_info
        self.camera_configs = camera_configs

        self.encoders = {}           # local CameraEncoder
        self.network_receivers = {}  # NetworkTSReceiver

        self.start_server = ActionServer(
            node,
            StartCamera,
            "start_camera_encoding",
            execute_callback=self._start_exec,
            goal_callback=self._start_goal,
            cancel_callback=self._start_cancel,
        )

        self.stop_server = ActionServer(
            node,
            StopCamera,
            "stop_camera_encoding",
            execute_callback=self._stop_exec,
            goal_callback=self._stop_goal,
        )
        

    # ------------------------------
    # START
    # ------------------------------
    def start_network_receiver(self, cam: str):
        
        #Idempotently start a NetworkTSReceiver for a configured H.264 camera.
        #Returns (ok: bool, message: str, client_topic: str | None)

        if self.registry is None:
            return False, "Registry not ready", None

        # Already running? Return success so callers can be idempotent.
        if cam in self.network_receivers:
            rx = self.network_receivers[cam]

            # If the underlying process died, replace it
            if rx.is_dead:
                self.node.get_logger().warn(
                    f"[{cam}] Network receiver was dead — restarting"
                )
                try:
                    rx.stop()
                except Exception:
                    pass
            else:
                if not rx.is_publishing:
                    rx.resume_publishers()
                    msg = f"Network camera {cam} resumed"
                else:
                    msg = f"Network camera {cam} already running"
                topic = rx.foxglove_topic if rx.output_mode == "foxglove" else rx.output_topic
                return True, msg, topic

        cfg_block = self.camera_configs.get("h264_network_cameras", {})
        if cam not in cfg_block:
            return False, f"No config found for network camera {cam}", None

        net_cfg = cfg_block[cam]
        if "url" not in net_cfg:
            return False, f"No URL configured for network camera {cam}", None

        url = net_cfg["url"]
        fps = float(net_cfg.get("params", {}).get("framerate", 30))
        width = net_cfg.get("params", {}).get("width")
        height = net_cfg.get("params", {}).get("height")
        mode = self.node.output_mode.lower()

        rx = NetworkTSReceiver(
            node=self.node,
            camera_name=cam,
            listen_url=url,
            fps=fps,
            width=width,
            height=height,
            output_mode=mode,
            registry=self.registry,
        )

        rx.start()
        self.network_receivers[cam] = rx

        client_topic = rx.foxglove_topic if mode == "foxglove" else rx.output_topic
        return True, f"Started network camera {cam}", client_topic

    def _start_goal(self, goal):
        cam = goal.camera_name
        if self.registry is None:
            return GoalResponse.REJECT

        # Allow network cameras as long as they are registered, even if not yet heartbeat-active
        if cam in self.camera_configs.get("h264_network_cameras", {}):
            return GoalResponse.ACCEPT if self.registry.is_registered(cam) else GoalResponse.REJECT

        # Local cameras must be active (recent heartbeat)
        if cam not in self.registry.active_cameras:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _start_cancel(self, goal_handle):
        cam = goal_handle.request.camera_name

        if cam in self.encoders:
            self.encoders.pop(cam).stop()

        if cam in self.network_receivers:
            self.network_receivers.pop(cam).stop()

        return CancelResponse.ACCEPT

    async def _start_exec(self, goal_handle):
        cam = goal_handle.request.camera_name
        mode = self.node.output_mode.lower()
        codec = self.encoder_info["codec"]

        # ----------------------------
        # Prevent double start
        # ----------------------------
        if cam in self.encoders or cam in self.network_receivers:
            # If network receiver was paused, ensure it resumes
            if cam in self.network_receivers and not self.network_receivers[cam].is_publishing:
                self.network_receivers[cam].resume_publishers()
                msg = f"Camera {cam} resumed"
            else:
                msg = f"Camera {cam} already running"

            goal_handle.succeed()
            result = StartCamera.Result()
            result.success = True
            result.message = msg
            if cam in self.encoders:
                base_topic = self.encoders[cam].output_topic
                result.topic = base_topic + "/foxglove" if mode == "foxglove" else base_topic
            else:
                rx = self.network_receivers[cam]
                base_topic = rx.output_topic
                result.topic = base_topic + "/foxglove" if mode == "foxglove" else base_topic
            return result

        # ----------------------------
        # NETWORK CAMERA (cameras in the network)
        # ----------------------------
        if cam in self.camera_configs.get("h264_network_cameras", {}):
            ok, message, topic = self.start_network_receiver(cam)
            if not ok:
                goal_handle.abort()
                result = StartCamera.Result()
                result.success = False
                result.message = message
                return result

            goal_handle.succeed()
            result = StartCamera.Result()
            result.success = ok
            result.message = message
            result.topic = topic
            return result

        # ----------------------------
        # LOCAL CAMERA (usb oak-d connected throug USB)
        # ----------------------------
        input_topic = self.registry.get_topic_for(cam)
        fps = float(
            self.camera_configs.get(cam, {})
            .get("params", {})
            .get("framerate", 30)
        )

        encoder_topic = f"/{cam}/encoded/{codec}"

        enc = CameraEncoder(
            node=self.node,
            camera_name=cam,
            encoder_info=self.encoder_info,
            fps=fps,
            input_topic=input_topic,
            output_topic=encoder_topic,
            output_mode=mode,
        )

        enc.apply_output_mode(mode)
        enc.start()
        self.encoders[cam] = enc

        client_topic = (
            encoder_topic + "/foxglove" if mode == "foxglove" else encoder_topic
        )

        goal_handle.succeed()
        result = StartCamera.Result()
        result.success = True
        result.message = f"Started encoder for {cam}"
        result.topic = client_topic
        return result


    # ------------------------------
    # STOP
    # ------------------------------
    def _stop_goal(self, goal):
        return GoalResponse.ACCEPT if (
            goal.camera_name in self.encoders
            or goal.camera_name in self.network_receivers
        ) else GoalResponse.REJECT

    async def _stop_exec(self, goal_handle):
        cam = goal_handle.request.camera_name

        if cam in self.encoders:
            self.encoders.pop(cam).stop()
        elif cam in self.network_receivers:
            # Do not kill the listener — just pause publishing so availability stays up
            rx = self.network_receivers[cam]
            rx.pause_publishers(lock=True)
        else:
            goal_handle.abort()
            result = StopCamera.Result()
            result.success = False
            result.message = f"No camera running for {cam}"
            return result

        goal_handle.succeed()
        result = StopCamera.Result()
        result.success = True
        result.message = f"Stopped camera {cam}"
        return result

    # ------------------------------
    def handle_disconnected(self, lost):
        for cam in lost:
            if cam in self.encoders:
                self.encoders.pop(cam).stop()
            if cam in self.network_receivers:
                # Keep the listener alive for when the stream comes back.
                rx = self.network_receivers[cam]
                rx.pause_publishers(lock=False)
                # If ffmpeg died for some reason, try to relaunch it.
                if rx.process_ts and rx.process_ts.poll() is not None:
                    rx.stop()
                    ok, msg, _ = self.start_network_receiver(cam)
                    if not ok:
                        self.node.get_logger().warn(
                            f"[{cam}] Failed to restart network receiver after disconnect: {msg}"
                        )

    def stop_all(self):
        for enc in self.encoders.values():
            enc.stop()
        for rx in self.network_receivers.values():
            rx.stop()

        self.encoders.clear()
        self.network_receivers.clear()
