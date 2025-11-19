


import time

from typing import List, Dict, Set

from rclpy.node import Node
from rclpy.action import ActionClient

from hs_cameras.srv import GetCameras                  # type: ignore
from hs_cameras.action import StartCamera, StopCamera  # type: ignore
from fpv_client.client_service import CameraServiceClient
from fpv_client.image_decoder import DecoderProcess

def log(msg: str):
    print(f"[Viewer] {msg}", flush=True)
    
class CameraActionManager:
    def __init__(self, node: Node, decoder: DecoderProcess, client: CameraServiceClient):
        self.node = node
        self.decoder = decoder
        self.client = client

        self.start_client = ActionClient(node, StartCamera, "start_camera_encoding")
        self.stop_client = ActionClient(node, StopCamera, "stop_camera_encoding")

        self.camera_topics: Dict[str, str] = {}
        self.current_cameras: List[str] = []
        self.pending_starts: Set[str] = set()
        self.pending_stops: Set[str] = set()

    # ------------------------------------------------------------
    # Start Cameras
    # ------------------------------------------------------------
    def start_group(self, cameras: List[str], available: List[str]):
        # Always use latest list from monitor
        available = self.node.monitor.available_cameras
        valid = [c for c in cameras if c in available]
        if not valid:
            log("No valid camera names.")
            return

        # Treat cameras currently being stopped as *not* active
        effective_active = {c for c in self.current_cameras if c not in self.pending_stops}
        new_cams = [c for c in valid if c not in effective_active]

        if not new_cams:
            if not self.pending_stops.intersection(valid):
                log("All requested cameras are already active.")
            else:
                log("Cameras are currently stopping; ignoring duplicate start request.")
            return

        # Add cameras to tracking lists
        for cam in new_cams:
            if cam not in self.current_cameras:
                self.current_cameras.append(cam)
        self.pending_starts |= set(new_cams)

        log(f"Starting encoders for: {', '.join(new_cams)}")

        try:
            self.start_client.wait_for_server()
        except Exception as e:
            log(f"StartCamera action server not available: {e}")
            for cam in new_cams:
                if cam in self.current_cameras:
                    self.current_cameras.remove(cam)
                self.pending_starts.discard(cam)
            return

        for cam in new_cams:
            goal = StartCamera.Goal(camera_name=cam)
            future = self.start_client.send_goal_async(goal, feedback_callback=self._feedback)
            future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                log("Start goal rejected.")
                return
            log("Start goal accepted.")
            goal_handle.get_result_async().add_done_callback(self._result)
        except Exception as e:
            log(f"Start goal failed: {e}")

    def _feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        log(f"Feedback: {fb.status}")

    def _result(self, future):
        try:
            result = future.result().result
            msg, topic = result.message, getattr(result, "topic", "")
            log(f"Result: {msg}")
            if topic:
                log(f"Encoded topic: {topic}")

            cam = self._extract_name(msg)
            if cam and cam in self.pending_starts:
                self.camera_topics[cam] = topic
                self.pending_starts.discard(cam)

            if not self.pending_starts:
                log("All cameras ready — updating decoder view.")
                self._wait_for_topics([self.camera_topics.get(c) for c in self.current_cameras])
                self.update_decoder()
        except Exception as e:
            log(f"Result handling failed: {e}")

    # ------------------------------------------------------------
    # Stop Cameras
    # ------------------------------------------------------------
    def stop_cameras(self, cameras: List[str]):
        if not cameras:
            log("No camera names provided to stop.")
            return

        if not self.stop_client.wait_for_server(timeout_sec=2.0):
            log("Stop server unavailable — removing locally.")
            for cam in cameras:
                if cam in self.current_cameras:
                    self.current_cameras.remove(cam)
                    self.camera_topics.pop(cam, None)
            self.update_decoder()
            return

        self.pending_stops = set(cameras)
        for cam in cameras:
            log(f"Sending stop request for '{cam}'...")
            goal = StopCamera.Goal(camera_name=cam)
            self.stop_client.send_goal_async(goal).add_done_callback(
                lambda f, c=cam: self._stop_response(f, c)
            )

    def _stop_response(self, future, cam: str):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                log(f"Stop goal rejected for '{cam}' — removing locally.")
                # Cleanup local tracking immediately
                if cam in self.current_cameras:
                    self.current_cameras.remove(cam)
                    self.camera_topics.pop(cam, None)
                # Refresh decoder
                self.update_decoder()
                return

            goal_handle.get_result_async().add_done_callback(lambda f, c=cam: self._stop_result(f, c))
        except Exception as e:
            log(f"Failed to send stop goal for '{cam}': {e}")
            # Do local cleanup too in case of exception
            if cam in self.current_cameras:
                self.current_cameras.remove(cam)
                self.camera_topics.pop(cam, None)
            self.update_decoder()

    def _stop_result(self, future, cam: str):
        try:
            result = future.result().result
            status = "OK" if result.success else "FAILED"
            log(f"{status} Stop result for '{cam}': {result.message}")

            if result.success:
                self.pending_stops.discard(cam)
                if cam in self.current_cameras:
                    self.current_cameras.remove(cam)
                    self.camera_topics.pop(cam, None)

                if not self.current_cameras:
                    log("All encoders stopped — stopping local decoder.")
                    self.decoder.stop()

                    # Reset local tracking for clean restart
                    self.camera_topics.clear()
                    self.pending_starts.clear()
                    self.pending_stops.clear()
                else:
                    # Some cameras still running — just update decoder
                    self.update_decoder()

        except Exception as e:
            log(f"Failed to get stop result for '{cam}': {e}")

    def stop_all(self):
        if not self.current_cameras:
            return
        log("Stopping all active cameras...")
        # Just send stop goals and let the executor handle callbacks
        self.stop_cameras(self.current_cameras.copy())

    # ------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------
    def list_cameras(self, silent=False):
        def _on_resp(f=None):
            """Unified printout for both startup and list command."""
            self.node.clear_screen()
            print("=== [o] Camera Status ===\n")

            if f is None:
                # Service not available or not queried yet
                print("Available: (unknown)")
                print("Active:    (unknown)\n")
            else:
                try:
                    res = f.result()
                    if not res:
                        print("Available: (none)")
                        print("Active:    (none)\n")
                    else:
                        self.node.monitor.available_cameras = list(res.cameras)
                        print(f"Available: {', '.join(res.cameras) or '(none)'}")
                        print(f"Active:    {', '.join(res.active_cameras) or '(none)'}\n")
                except Exception as e:
                    log(f"Failed to get camera list: {e}")
                    print("Available: (error)")
                    print("Active:    (error)\n")

            print("Commands:")
            print("  list                      – refresh camera status")
            print("  start <cam1> [cam2 ...]   – start encoder(s)")
            print("  stop [cam1 cam2 ...]      – stop encoder(s) or all")
            print("  exit                      – stop all + quit\n")

        # Try to contact the service
        if not self.client.client.wait_for_service(timeout_sec=3.0):
            if not silent:
                # Only print error when user explicitly asks for "list"
                log("Service /get_available_cameras not available.")
            _on_resp(None)
            return

        # Service available → call and use same print function
        fut = self.client.client.call_async(GetCameras.Request())
        fut.add_done_callback(_on_resp)

    def update_decoder(self):
        topics = [self.camera_topics.get(c) for c in self.current_cameras if self.camera_topics.get(c)]
        self.decoder.launch(topics)

    def _wait_for_topics(self, topics: List[str], timeout=5.0):
        start = time.time()
        needed = {t for t in topics if t}
        if not needed:
            return
        while time.time() - start < timeout:
            available = {t for t, _ in self.node.get_topic_names_and_types()}
            if needed.issubset(available):
                return
            time.sleep(0.2)
        log("Encoded topics not ready after 5 s — continuing anyway.")

    @staticmethod
    def _extract_name(msg: str):
        if "for" not in msg:
            return None
        tail = msg.split("for", 1)[1].strip()
        if "'" in tail:
            parts = tail.split("'")
            if len(parts) >= 2:
                return parts[1]
        return tail.rstrip(".").split()[0]