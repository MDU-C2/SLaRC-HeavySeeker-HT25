#!/usr/bin/env python3

#Standalone dummy that mimics the Raspberry Pi by taking an existing
#MPEG-TS/H.264 stream from a ROS topic and sending it over UDP to the
#camera server. This keeps the server and client happy without real hardware.


import argparse
import socket
from urllib.parse import urlparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def parse_dest(dest: str):
    """
    Accepts formats like:
      - udp://127.0.0.1:5000
      - 127.0.0.1:5000
    Returns (host, port).
    """
    if dest.startswith("udp://"):
        parsed = urlparse(dest)
        host = parsed.hostname or "127.0.0.1"
        port = parsed.port or 5000
        return host, port

    if ":" in dest:
        host, port = dest.split(":", 1)
        return host, int(port)

    # Only a port provided
    return "127.0.0.1", int(dest)


class PiDummyStreamer(Node):
    def __init__(self, topic: str, dest_host: str, dest_port: int, heartbeat: float):
        super().__init__("pi_dummy_streamer")
        self.dest = (dest_host, dest_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(
            CompressedImage,
            topic,
            self._handle_frame,
            10,
        )
        self.heartbeat_period = heartbeat
        self.last_sent = 0.0
        if heartbeat > 0:
            self.create_timer(heartbeat, self._send_heartbeat)
        self.get_logger().info(
            f"Relaying '{topic}' → udp://{dest_host}:{dest_port}"
        )

    def _handle_frame(self, msg: CompressedImage):
        data = bytes(msg.data)
        try:
            self.sock.sendto(data, self.dest)
            self.last_sent = time.time()
        except Exception as exc:
            self.get_logger().error(f"Failed to send frame ({len(data)} bytes): {exc}")

    def _send_heartbeat(self):
        # Send a single MPEG-TS null packet (PID 0x1FFF) to keep receiver alive
        null_packet = bytes([0x47, 0x1F, 0xFF, 0x10] + [0xFF] * 184)
        try:
            self.sock.sendto(null_packet, self.dest)
        except Exception as exc:
            self.get_logger().warning(f"Heartbeat send failed: {exc}")


def main():
    parser = argparse.ArgumentParser(
        description="Relay a local encoded H264 topic to the camera server over UDP"
    )
    parser.add_argument(
        "--topic",
        default="/camera0/encoded/h264",
        help="Source topic carrying CompressedImage MPEG-TS/H264",
    )
    parser.add_argument(
        "--dest",
        default="udp://127.0.0.1:5000",
        help="Destination in form udp://host:port or host:port",
    )
    parser.add_argument(
        "--heartbeat",
        type=float,
        default=1.0,
        help="Seconds between null-packet heartbeats (0 to disable)",
    )

    args = parser.parse_args()
    host, port = parse_dest(args.dest)

    rclpy.init()
    node = PiDummyStreamer(args.topic, host, port, args.heartbeat)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
