import time
from typing import Dict, List

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from utils.server import (
    find_camera_topics,
    make_camera_status_json,
)


class CameraRegistry:


    def __init__(self, node: Node, encoder_info: dict, on_disconnected=None):
        self.node = node
        self.encoder_info = encoder_info
        self.on_disconnected = on_disconnected

        self.cameras: Dict[str, float] = {}
        self.camera_topics: Dict[str, str] = {}
        self.active_cameras: List[str] = []

        self.publisher = node.create_publisher(String, "/available_cameras", 10)
        node.create_timer(3.0, self._refresh_topics)
        node.create_timer(1.0, self._check_activity)

    def _refresh_topics(self):
        topics = self.node.get_topic_names_and_types()
        found = find_camera_topics(topics)
        seen = set(found.keys())

        # New cameras
        for cam in seen - self.cameras.keys():
            topic = found[cam]
            self.camera_topics[cam] = topic
            self.cameras[cam] = time.time()
            self.node.create_subscription(Image, topic, lambda msg, n=cam: self._touch(n), 10)
            self.node.get_logger().info(f"Camera {cam} discovered on {topic}")

        # Removed cameras
        for cam in list(self.cameras.keys()):
            if cam not in seen:
                del self.cameras[cam]
                self.camera_topics.pop(cam, None)
                self.node.get_logger().info(f"Camera {cam} removed")

        self._publish_state()

    def _touch(self, cam):
        self.cameras[cam] = time.time()

    def _check_activity(self):
        self._publish_state()

    def _publish_state(self):
        now = time.time()
        active = sorted([cam for cam, ts in self.cameras.items() if now - ts < 4])

        lost = [c for c in self.active_cameras if c not in active]

        if active != self.active_cameras:
            self.active_cameras = active
            msg = String()
            msg.data = make_camera_status_json(active, self.encoder_info)
            self.publisher.publish(msg)

            if lost and self.on_disconnected:
                self.on_disconnected(lost)

    def get_topic_for(self, cam: str):
        return self.camera_topics.get(cam)
