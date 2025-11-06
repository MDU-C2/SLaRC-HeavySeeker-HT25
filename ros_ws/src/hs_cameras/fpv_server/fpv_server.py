#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FPV Server — ROS 2 Jazzy compatible Action Server node.
Monitors camera /image_raw topics and publishes /available_cameras.
Simulated ActionServer (StartCamera) for testing client communication.
"""

import asyncio
import json
import re
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from hs_cameras.action import StartCamera # type: ignore



# --------------------------------------------------------------------
# Camera discovery and tracking
# --------------------------------------------------------------------
class Camera:
    """Represents one camera and tracks activity from /<name>/image_raw."""

    def __init__(self, name: str, node: Node, on_first_frame):
        self.name = name
        self.node = node
        self.on_first_frame = on_first_frame
        self.last_seen = 0.0

        topic = f"/{name}/image_raw"
        self.sub = node.create_subscription(Image, topic, self._callback, 10)
        self.node.get_logger().info(f"Subscribed to {topic}")

    # ---------------------------------------------------------------
    def _callback(self, _msg):
        """Triggered whenever a new frame arrives."""
        first_seen = self.last_seen == 0.0
        self.last_seen = time.time()
        if first_seen:
            self.node.get_logger().info(f"Camera {self.name} became active ✅")
            self.on_first_frame(self.name)

    # ---------------------------------------------------------------
    def is_active(self, timeout: float = 2.0) -> bool:
        """Return True if the camera has published recently."""
        return self.last_seen != 0.0 and (time.time() - self.last_seen) <= timeout

    # ---------------------------------------------------------------
    def destroy(self):
        """Clean up ROS subscription."""
        if self.sub:
            self.node.destroy_subscription(self.sub)
            self.sub = None


# --------------------------------------------------------------------
# Watches all /image_raw topics and reports active cameras
# --------------------------------------------------------------------
class CameraActivityWatcher(Node):
    """Sub-node that monitors topics and updates the list of active cameras."""

    def __init__(self, on_update):
        super().__init__("camera_activity_watcher")
        self.on_update = on_update
        self.cameras: dict[str, Camera] = {}

        self.create_timer(1.0, self._check_activity)
        self.create_timer(3.0, self._refresh_topics)

    # ---------------------------------------------------------------
    def _refresh_topics(self):
        topics = self.get_topic_names_and_types()
        seen = {
            name.strip("/").split("/")[0]
            for name, _ in topics
            if re.search(r"/image_raw$", name)
        }

        # Add new cameras
        for cam_name in seen - self.cameras.keys():
            self.cameras[cam_name] = Camera(cam_name, self, self._on_update)

        # Remove missing ones
        for cam_name in list(self.cameras.keys()):
            if cam_name not in seen:
                self.get_logger().info(f"Camera {cam_name} topic disappeared → removing")
                self._remove_camera(cam_name)

    # ---------------------------------------------------------------
    def _check_activity(self):
        removed = [n for n, c in self.cameras.items() if not c.is_active()]
        for n in removed:
            self.get_logger().info(f"Camera {n} inactive → removed")
            self._remove_camera(n)
        if removed:
            self._on_update()

    # ---------------------------------------------------------------
    def _remove_camera(self, name):
        cam = self.cameras.pop(name, None)
        if cam:
            cam.destroy()

    # ---------------------------------------------------------------
    def _on_update(self, *_):
        self.on_update(self.get_active())

    # ---------------------------------------------------------------
    def get_active(self):
        return sorted([n for n, c in self.cameras.items() if c.is_active()])


# --------------------------------------------------------------------
# FPV Action Server node
# --------------------------------------------------------------------
class FPVServer(Node):
    """ROS 2 Action Server that publishes camera list and handles test goals."""

    def __init__(self):
        super().__init__("fpv_server")

        # Publisher for list of cameras
        self.active_cameras: list[str] = []
        self.pub = self.create_publisher(String, "/available_cameras", 10)

        # Action Server setup
        self.action_server = ActionServer(
            self,
            StartCamera,
            "start_camera_encoding",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Background watcher node running in its own executor
        self.watcher = CameraActivityWatcher(self._on_camera_update)
        self.watcher_exec = MultiThreadedExecutor()
        self.watcher_exec.add_node(self.watcher)
        threading.Thread(target=self.watcher_exec.spin, daemon=True).start()

        self.get_logger().info("FPV Server (Action Server) initialized ✅")

    # ---------------------------------------------------------------
    def _on_camera_update(self, cameras: list[str]):
        """Callback from watcher when active camera list changes."""
        self.active_cameras = cameras
        msg = String()
        msg.data = json.dumps({"available_cameras": cameras})
        self.pub.publish(msg)
        self.get_logger().info(f"Updated available cameras: {cameras}")

    # ---------------------------------------------------------------
    def goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        cam = goal_request.camera_name
        self.get_logger().info(f"Received goal for camera '{cam}'")
        if cam not in self.active_cameras:
            self.get_logger().warn(f"Camera '{cam}' not active → rejecting")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ---------------------------------------------------------------
    def cancel_callback(self, goal_handle):
        """Handle cancel requests."""
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    # ---------------------------------------------------------------
    async def execute_callback(self, goal_handle):
        """Main action execution (simulated)."""
        cam = goal_handle.request.camera_name
        fb = StartCamera.Feedback()

        fb.status = f"Preparing pipeline for {cam} (simulation)"
        goal_handle.publish_feedback(fb)
        await asyncio.sleep(2.0)

        fb.status = f"Pipeline setup complete for {cam} (simulated)"
        goal_handle.publish_feedback(fb)
        goal_handle.succeed()

        result = StartCamera.Result()
        result.success = True
        result.message = f"Ready to start encoding for {cam}."
        return result


# --------------------------------------------------------------------
# Main entrypoint
# --------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FPVServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("FPV Server interrupted by user.")
    except rclpy.executors.ExternalShutdownException:
        node.get_logger().info("FPV Server shutting down cleanly.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
