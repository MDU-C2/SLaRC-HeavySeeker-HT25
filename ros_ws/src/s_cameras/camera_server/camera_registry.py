import time
from typing import Dict, List

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from .server_utils import (
    find_camera_topics,
    make_camera_status_json,
)


class CameraRegistry:


    def __init__(self, node: Node, encoder_info: dict, on_disconnected=None):
        self.node = node
        self.encoder_info = encoder_info
        self.on_disconnected = on_disconnected

        # cam_name -> last_seen_timestamp
        self.cameras: Dict[str, float] = {}

        # cam_name -> topic (raw or encoded)
        self.camera_topics: Dict[str, str] = {}

        # Track network cameras so they are startable even before heartbeat
        self.network_cameras = set()

        self.active_cameras: List[str] = []
        self._last_registered = set()

        self.publisher = node.create_publisher(String, "/available_cameras", 10)

        # Timers
        node.create_timer(3.0, self._refresh_topics)
        node.create_timer(1.0, self._check_activity)

    # ------------------------------------------------------------------
    # LOCAL CAMERA DISCOVERY (unchanged)
    # ------------------------------------------------------------------
    def _refresh_topics(self):
        topics = self.node.get_topic_names_and_types()
        found = find_camera_topics(topics)
        seen = set(found.keys())

        # New local cameras
        for cam in seen - self.cameras.keys():
            topic = found[cam]
            self.camera_topics[cam] = topic
            self.cameras[cam] = time.time()

            # Touch on Image receive
            self.node.create_subscription(
                Image,
                topic,
                lambda msg, n=cam: self._touch(n),
                10,
            )

            self.node.get_logger().info(
                f"Local camera {cam} discovered on {topic}"
            )

        # Removed local cameras
        for cam in list(self.cameras.keys()):
            if cam not in seen and cam not in self.camera_topics:
                continue  # network camera
            if cam not in seen and self.camera_topics.get(cam, "").endswith("image_raw"):
                del self.cameras[cam]
                self.camera_topics.pop(cam, None)
                self.node.get_logger().info(f"Local camera {cam} removed")

        self._publish_state()

    # ------------------------------------------------------------------
    # NETWORK CAMERA SUPPORT
    # ------------------------------------------------------------------
    def register_network_camera(self, cam_name: str, topic: str = None):

        if cam_name in self.camera_topics:
            return

        # Initialize timestamp in the past so it is not active until touched
        self.cameras[cam_name] = 0.0
        self.camera_topics[cam_name] = topic or f"/{cam_name}/encoded/h264"
        self.network_cameras.add(cam_name)

        self.node.get_logger().info(
            f"Network camera {cam_name} registered"
        )
        self._publish_state()


    def touch(self, cam: str):
        """
        Public heartbeat API (used by NetworkTSReceiver).
        """
        self._touch(cam)

    # ------------------------------------------------------------------
    # INTERNAL
    # ------------------------------------------------------------------
    def _touch(self, cam: str):
        self.cameras[cam] = time.time()

    def _check_activity(self):
        self._publish_state()

    def _publish_state(self):
        now = time.time()
        active = sorted([cam for cam, ts in self.cameras.items() if now - ts < 4])
        # Hide network cameras from "registered" until we've seen a heartbeat
        registered = {
            cam for cam in self.camera_topics.keys()
            if not (cam in self.network_cameras and self.cameras.get(cam, 0) <= 0)
        }

        lost = [c for c in self.active_cameras if c not in active]

        if active != self.active_cameras or registered != self._last_registered:
            self.active_cameras = active
            self._last_registered = registered

            msg = String()
            msg.data = make_camera_status_json(
                active_cameras=active,
                registered_cameras=sorted(registered),
                encoder_info=self.encoder_info,
            )
            self.publisher.publish(msg)

            if lost and self.on_disconnected:
                self.on_disconnected(lost)

    # ------------------------------------------------------------------
    def get_topic_for(self, cam: str):
        return self.camera_topics.get(cam)

    def is_registered(self, cam: str) -> bool:
        return cam in self.camera_topics

    def all_registered(self):
        return sorted(self.camera_topics.keys())
