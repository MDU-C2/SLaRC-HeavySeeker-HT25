import json
from rclpy.node import Node
from std_msgs.msg import String
from typing import List
def log(msg: str):
    print(f"[Viewer] {msg}", flush=True)

class CameraMonitor:
    def __init__(self, node: Node):
        self.node = node
        self.available_cameras: List[str] = []
        self.subscription = node.create_subscription(
            String, "/available_cameras", self._on_camera_list, 10
        )
        self._last_available: List[str] = []

    def _on_camera_list(self, msg: String):
        try:
            data = json.loads(msg.data)
            available = data.get("available_cameras", [])
            if available != self.available_cameras:
                self.available_cameras = available

                if hasattr(self.node, "actions"):
                    # Cameras that are encoding but no longer in available list
                    lost_cams = [
                        cam for cam in self.node.actions.current_cameras
                        if cam not in available
                    ]
                    if lost_cams:
                        log(f"Lost cameras detected: {', '.join(lost_cams)} — stopping encoders.")
                        self.node.actions.stop_cameras(lost_cams)
                        # Update decoder view right after stop
                        self.node.actions.update_decoder()

                    # Redraw UI quietly
                    self.node.actions.list_cameras(silent=True)

        except Exception as e:
            log(f"Failed to parse camera list: {e}")




