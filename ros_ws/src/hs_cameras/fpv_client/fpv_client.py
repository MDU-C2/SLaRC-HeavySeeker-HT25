#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import json
import rclpy
import signal
import threading
import subprocess

from std_msgs.msg import String
from typing import List, Dict, Set

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from hs_cameras.srv import GetCameras                  # type: ignore
from hs_cameras.action import StartCamera, StopCamera  # type: ignore

print("[DEBUG] Loaded fpv_client from:", __file__)

# ================================================================
# Logging helper
# ================================================================
def log(msg: str):
    print(f"[Viewer] {msg}", flush=True)


# ================================================================
# Camera Monitor
# ================================================================
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


# ================================================================
# Camera Service Client
# ================================================================
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


# ================================================================
# Decoder Process
# ================================================================
class DecoderProcess:
    def __init__(self):
        self.proc: subprocess.Popen | None = None
        self.active_topics: List[str] = []

    def stop(self):
        """Stop the decoder subprocess if running."""
        if not self.proc or self.proc.poll() is not None:
            return
        log("Stopping decoder window")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except Exception:
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.proc = None
        self.active_topics = []

    def launch(self, topics: List[str]):
        """Start or restart the decoder for the given topic list."""
        if not topics:
            self.stop()
            return

        topics = sorted(set(topics))

        # Always restart if topics changed or process not running
        process_alive = self.proc and self.proc.poll() is None
        if process_alive and topics == self.active_topics:
            return  # already running same set

        # Stop any old process first
        if process_alive:
            log("Updating decoder view")
            self.stop()
        else:
            log("Launching decoder window")

        # Launch new decoder process
        cmd = ["ros2", "run", "hs_cameras", "decoder"] + topics
        log(f"Decoder command: {' '.join(cmd)}")

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self.active_topics = topics
        except Exception as e:
            log(f"Failed to start decoder: {e}")
            self.proc = None
            self.active_topics = []


# ================================================================
# Camera Action Manager
# ================================================================
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

        log(f"🚀 Starting encoders for: {', '.join(new_cams)}")

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
                log(f"🔗 Encoded topic: {topic}")

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
            print("=== [o]📸 Camera Status ===\n")

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


# ================================================================
# FPV Viewer Client (composition root)
# ================================================================
class FPVViewerClient(Node):
    def __init__(self):
        super().__init__("fpv_viewer_client")
        self._stop_event = threading.Event()

        self.monitor = CameraMonitor(self)
        self.service = CameraServiceClient(self)
        self.decoder = DecoderProcess()
        self.actions = CameraActionManager(self, self.decoder, self.service)

        self.timer = self.create_timer(1.0, self._initial_query_once)

    def _initial_query_once(self):
        # Always show the screen first (even if no service yet)
        self.actions.list_cameras(silent=True)
        self.service.request_once(self._on_initial_list)
        if self.timer:
            self.timer.cancel()

    def _on_initial_list(self, cameras: List[str]):
        self.monitor.available_cameras = cameras
        self.actions.list_cameras()

    @staticmethod
    def clear_screen():
        os.system("cls" if os.name == "nt" else "clear")

    def user_input_loop(self):
        while rclpy.ok() and not _shutdown_event.is_set():
            try:
                cmd = input("FPV> ").strip()
            except EOFError:
                break

            if cmd in ("exit", "quit"):
                log("Exiting...")
                self.shutdown()
                self._stop_event.set()
                rclpy.shutdown()
                break

            elif cmd == "list":
                self.actions.list_cameras()

            elif cmd.startswith("start "):
                parts = cmd.split()[1:]
                if parts:
                    self.actions.start_group(parts, self.monitor.available_cameras)
                else:
                    log("No camera names provided.")

            elif cmd.startswith("stop"):
                parts = cmd.split()[1:]
                if parts:
                    self.actions.stop_cameras(parts)
                else:
                    if self.actions.current_cameras:
                        self.actions.stop_all()
                    else:
                        log("No active cameras — stopping decoder only.")
                        self.decoder.stop()

            else:
                print("Unknown command. Use: start <camera...> | stop | list | exit", flush=True)

    def shutdown(self):
        try:
            if self.actions.current_cameras:
                log("Stopping all active cameras before shutdown...")
                self.actions.stop_all()
        except Exception as e:
            log(f"Error during encoder stop: {e}")
        finally:
            self.decoder.stop()


# ================================================================
# Entrypoint
# ================================================================
_shutdown_event = threading.Event()


def setup_sigint_handler():
    def handler(signum, frame):
        print("\n[Viewer] Ctrl+C detected — shutting down...", flush=True)
        _shutdown_event.set()
    signal.signal(signal.SIGINT, handler)


def main(args=None):
    rclpy.init(args=args)
    setup_sigint_handler()

    node = FPVViewerClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ui_thread = threading.Thread(target=node.user_input_loop, daemon=True)
    ui_thread.start()

    try:
        while rclpy.ok() and not _shutdown_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        log("Graceful shutdown requested...")
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        _shutdown_event.set()
        ui_thread.join(timeout=1.0)
        log("Shutdown complete.")


if __name__ == "__main__":
    main()
