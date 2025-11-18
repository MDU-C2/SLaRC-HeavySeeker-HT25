from rclpy.node import Node
from hs_cameras.srv import GetCameras                  # type: ignore

def log(msg: str):
    print(f"[Viewer] {msg}", flush=True)

class CameraServiceClient:
    def __init__(self, node: Node):
        self.node = node
        self.client = node.create_client(GetCameras, "get_available_cameras")

    def request_once(self, callback):
        if not self.client.wait_for_service(timeout_sec=5.0):
            log("Service /get_available_cameras not available.")
            return
        future = self.client.call_async(GetCameras.Request())
        future.add_done_callback(lambda fut: self._on_response(fut, callback))

    def _on_response(self, future, callback):
        try:
            result = future.result()
            if result:
                callback(list(result.cameras))
        except Exception as e:
            log(f"Failed service call: {e}")
