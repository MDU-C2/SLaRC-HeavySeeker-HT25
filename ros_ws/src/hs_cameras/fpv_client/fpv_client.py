#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
import signal
import threading

from typing import List

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

#from hs_cameras.srv import GetCameras                  # type: ignore
#from hs_cameras.action import StartCamera, StopCamera  # type: ignore
from fpv_client.action_handler import CameraActionManager
from fpv_client.image_decoder import DecoderProcess
from fpv_client.client_service import CameraServiceClient
from fpv_client.camera_monitor import CameraMonitor
print("[DEBUG] Loaded fpv_client from:", __file__)

# ================================================================
# Logging helper
# ================================================================
def log(msg: str):
    print(f"[Viewer] {msg}", flush=True)






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
