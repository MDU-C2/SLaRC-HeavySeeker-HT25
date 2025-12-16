from rclpy.action import ActionServer, GoalResponse, CancelResponse
from s_msgs.action import StartCamera, StopCamera

from camera_server.camera_encoder import CameraEncoder


class CameraActionHandler:

    def __init__(self, node, registry, encoder_info, camera_configs):
        self.node = node
        self.registry = registry
        self.encoder_info = encoder_info
        self.camera_configs = camera_configs
        self.encoders = {}

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
    def _start_goal(self, goal):
        cam = goal.camera_name
        if self.registry is None or cam not in self.registry.active_cameras:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _start_cancel(self, goal_handle):
        cam = goal_handle.request.camera_name
        if cam in self.encoders:
            self.encoders[cam].stop()
            self.encoders.pop(cam)
        return CancelResponse.ACCEPT

    async def _start_exec(self, goal_handle):
        cam = goal_handle.request.camera_name

        input_topic = self.registry.get_topic_for(cam)
        fps = float(self.camera_configs.get(cam, {}).get("params", {}).get("framerate", 30))

        codec = self.encoder_info["codec"]
        output_topic = f"/{cam}/encoded/{codec}"

        enc = CameraEncoder(
            node=self.node,
            camera_name=cam,
            encoder_info=self.encoder_info,
            fps=fps,
            input_topic=input_topic,
            output_topic=output_topic,
        )

        try:
            enc.start()
            self.encoders[cam] = enc
            goal_handle.succeed()
            result = StartCamera.Result()
            result.success = True
            result.message = f"Started encoder for {cam}"
            result.topic = output_topic
            return result
        except Exception as e:
            goal_handle.abort()
            result = StartCamera.Result()
            result.success = False
            result.message = str(e)
            return result

    # ------------------------------
    # STOP
    # ------------------------------
    def _stop_goal(self, goal):
        return GoalResponse.ACCEPT if goal.camera_name in self.encoders else GoalResponse.REJECT

    async def _stop_exec(self, goal_handle):
        cam = goal_handle.request.camera_name

        enc = self.encoders.pop(cam, None)
        if enc:
            enc.stop()
            goal_handle.succeed()
            result = StopCamera.Result()
            result.success = True
            result.message = f"Stopped encoder for {cam}"
            return result

        goal_handle.abort()
        result = StopCamera.Result()
        result.success = False
        result.message = f"No encoder running for {cam}"
        return result

    def handle_disconnected(self, lost):
        for cam in lost:
            if cam in self.encoders:
                self.encoders[cam].stop()
                self.encoders.pop(cam)

    def stop_all(self):
        for enc in self.encoders.values():
            enc.stop()
