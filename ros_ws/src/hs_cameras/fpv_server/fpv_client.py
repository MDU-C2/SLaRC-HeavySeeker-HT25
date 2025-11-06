#!/usr/bin/env python3

import json
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from hs_cameras.action import StartCamera # type: ignore


# --------------------------------------------------------------------
# FPV Viewer Client Node
# --------------------------------------------------------------------
class FPVViewerClient(Node):
    """Client that listens for available cameras and sends test goals."""

    def __init__(self):
        super().__init__("fpv_viewer_client")

        # Topic subscription for available cameras
        self.available_cameras: list[str] = []
        self.subscription = self.create_subscription(
            String, "/available_cameras", self._on_camera_list, 10
        )

        # Action client setup
        self.action_client = ActionClient(self, StartCamera, "start_camera_encoding")

        self.get_logger().info("FPV Viewer Client node initialized ✅")

    # ---------------------------------------------------------------
    def _on_camera_list(self, msg: String):
        """Handle camera list updates."""
        try:
            data = json.loads(msg.data)
            self.available_cameras = data.get("available_cameras", [])
            print(f"\n[Viewer] 📷 Cameras available: {self.available_cameras}\n")
        except Exception as e:
            print(f"[Viewer] ❌ Failed to parse camera list: {e}")

    # ---------------------------------------------------------------
    def send_goal(self, camera_name: str):
        """Send an action goal to start encoding a given camera."""
        if not self.available_cameras:
            print("[Viewer] No cameras available yet.")
            return
        if camera_name not in self.available_cameras:
            print(f"[Viewer] '{camera_name}' not available. Choose from {self.available_cameras}")
            return

        goal_msg = StartCamera.Goal(camera_name=camera_name)
        print(f"[Viewer] Sending goal for camera '{camera_name}'...")
        self.action_client.wait_for_server()
        send_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    # ---------------------------------------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[Viewer] ❌ Goal rejected.")
            return
        print("[Viewer] ✅ Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    # ---------------------------------------------------------------
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"[Viewer] 📡 Feedback: {feedback.status}")

    # ---------------------------------------------------------------
    def get_result_callback(self, future):
        result = future.result().result
        print(f"[Viewer] 🎯 Result: {result.message}")

    # ---------------------------------------------------------------
    def user_input_loop(self):
        """Simple interactive console input loop."""
        print("\n[Viewer] Commands:\n  list  – show cameras\n  start <name>  – send goal\n  exit  – quit\n")
        while rclpy.ok():
            try:
                cmd = input("FPV> ").strip()
            except EOFError:
                break
            if cmd in ("exit", "quit"):
                print("[Viewer] Exiting...")
                break
            elif cmd == "list":
                print(f"[Viewer] Cameras: {self.available_cameras}")
            elif cmd.startswith("start "):
                cam = cmd.split(" ", 1)[1]
                self.send_goal(cam)
            else:
                print("Unknown command. Use: list | start <camera> | exit")


# --------------------------------------------------------------------
# Main entry point
# --------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FPVViewerClient()

    # Run the input loop in a background thread
    executor = MultiThreadedExecutor()
    threading.Thread(target=node.user_input_loop, daemon=True).start()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("FPV Viewer interrupted by user.")
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().info("FPV Viewer shutting down cleanly.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
